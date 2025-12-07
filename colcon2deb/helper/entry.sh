#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,uid:,gid:,output:,log-dir:,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,skip-gen-debian,skip-build-deb
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@")

# Check if getopt failed
if [[ $? -ne 0 ]]; then
    exit 1
fi

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

# Create a user for the specfied uid/gid and fix file permissions
name=ubuntu
groupadd -g "$gid" "$name"
useradd -m -u "$uid" -g "$gid" "$name"
usermod -aG sudo "$name"
passwd -d "$name"
# /workspace is mounted from host, just fix permissions
chown -R "$name:$name" /workspace

# Run the build script
# Both workspace and output are always required now
sudo -u ubuntu \
     bash -c "rosdep update && python3 '$script_dir/main.py' --workspace=/workspace --output='$output' --log-dir='$log_dir' $skip_opts"
