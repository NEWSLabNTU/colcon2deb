#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,uid:,gid:,output:,log-dir:,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,skip-gen-debian,skip-build-deb
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@") || {
    echo "error: invalid options" >&2
    exit 1
}

# Evaluate the parsed options
eval set -- "$PARSED"

# Parse arguments
uid=
gid=
output=
log_dir=
skip_opts=""

print_usage() {
    echo "Usage: $0 --uid=UID --gid=GID [--output=OUTPUT_DIR] [--skip-*]"
}

while true; do
    case "$1" in
	-h|--help)
	    print_usage
	    exit 0
	    ;;
	--uid)
	    uid="$2"
	    shift 2
	    ;;
	--gid)
	    gid="$2"
	    shift 2
	    ;;
	--output)
	    output="$2"
	    shift 2
	    ;;
	--log-dir)
	    log_dir="$2"
	    shift 2
	    ;;
	--skip-rosdep-install)
	    skip_opts="$skip_opts --skip-rosdep-install"
	    shift 1
	    ;;
	--skip-copy-src)
	    skip_opts="$skip_opts --skip-copy-src"
	    shift 1
	    ;;
	--skip-gen-rosdep-list)
	    skip_opts="$skip_opts --skip-gen-rosdep-list"
	    shift 1
	    ;;
	--skip-colcon-build)
	    skip_opts="$skip_opts --skip-colcon-build"
	    shift 1
	    ;;
	--skip-gen-debian)
	    skip_opts="$skip_opts --skip-gen-debian"
	    shift 1
	    ;;
	--skip-build-deb)
	    skip_opts="$skip_opts --skip-build-deb"
	    shift 1
	    ;;
	--)
	    shift
	    break
	    ;;
	*)
	    echo "Invalid option: $1" >&2
	    print_usage >&2
	    exit 1
	    ;;
    esac
done

if [ -z "$uid" -o -z "$gid" ]; then
    echo "Invalid options" >&2
    print_usage >&2
    exit 1
fi

# Create (or reuse) a user with the host uid/gid and grant passwordless sudo.
# Newer base images (Ubuntu >= 23.04) already ship a user at uid 1000, so a
# blind useradd fails; reuse the existing account in that case.
name=builder
groupadd -g "$gid" "$name" 2>/dev/null || true
existing_user=$(getent passwd "$uid" | cut -d: -f1 || true)
if [ -n "$existing_user" ]; then
    echo "Reusing existing user '$existing_user' (uid $uid)"
    name="$existing_user"
    usermod -s /bin/bash "$name"
else
    useradd -m -u "$uid" -g "$gid" -s /bin/bash "$name" || {
        echo "error: failed to create build user with uid=$uid gid=$gid" >&2
        exit 1
    }
fi
mkdir -p /etc/sudoers.d
echo "$name ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/colcon2deb-builder

# Install required dependencies
echo "Installing build dependencies..."
apt-get update -qq
apt-get install -y -qq parallel fakeroot debhelper dh-python rsync > /dev/null

# Install pip packages. Newer images mark the system Python externally
# managed (PEP 668); the container is ephemeral, so overriding is safe.
echo "Installing rosdeb-bloom and rich..."
pip install --quiet /rosdeb-bloom rich 2>/dev/null \
    || pip install --quiet --break-system-packages /rosdeb-bloom rich

# Create a script to run as the builder user
_entry_tmpdir=$(mktemp -d)
chmod 755 "$_entry_tmpdir"
cat > "$_entry_tmpdir/build-as-user.sh" << 'USERSCRIPT'
#!/usr/bin/env bash
set -e

script_dir="$1"
output="$2"
log_dir="$3"
shift 3
skip_opts="$@"

# Source optional user-provided setup script if it exists.
# Users can create /colcon2deb-setup.sh in their Docker image to set up
# ROS environment or any other build dependencies. Example:
#   RUN echo 'source /opt/ros/humble/setup.bash' > /colcon2deb-setup.sh
if [ -f "/colcon2deb-setup.sh" ]; then
    echo "Sourcing /colcon2deb-setup.sh..."
    source /colcon2deb-setup.sh
fi

# Verify the image provides the ROS build toolchain before doing any work
missing=""
for cmd in python3 colcon rosdep cmake; do
    command -v "$cmd" >/dev/null 2>&1 || missing="$missing $cmd"
done
if [ -n "$missing" ]; then
    echo "error: builder image is missing required commands:$missing" >&2
    echo "hint: base your Dockerfile on a ROS image (e.g. FROM ros:humble)," >&2
    echo "      or install ros-dev-tools and source the ROS setup in /colcon2deb-setup.sh" >&2
    exit 1
fi

# Update rosdep as user
rosdep update

# Run the main build script
exec python3 "$script_dir/main.py" --workspace=/workspace --output="$output" --log-dir="$log_dir" $skip_opts
USERSCRIPT
chmod +x "$_entry_tmpdir/build-as-user.sh"

# Run everything as the host user so files are owned correctly
exec su "$name" -c "'$_entry_tmpdir/build-as-user.sh' '$script_dir' '$output' '$log_dir' $skip_opts"
