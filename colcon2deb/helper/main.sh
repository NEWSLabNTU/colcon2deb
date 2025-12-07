#!/usr/bin/env bash
set -e

# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,skip-gen-debian,skip-build-deb,workspace:,output:
PARSED=$(getopt --options "$OPTIONS" --longoptions "$LONGOPTIONS" --name "$0" -- "$@")

# Check if getopt failed
if [[ $? -ne 0 ]]; then
    exit 1
fi

# Evaluate the parsed options
eval set -- "$PARSED"

# Parse arguments
export rosdep_install=y
export gen_rosdep_list=y
export copy_src=y
export colcon_build=y
export gen_debian=y
export build_deb=y
export workspace_dir=
export output_dir=

print_usage() {
    echo "Usage: $0 [OPTION]... --workspace=WORKSPACE_DIR --output=OUTPUT_DIR"
    echo "  -h,--help                   show this help message"
    echo "  --skip-rosdep-install       do not run rosdep install"
    echo "  --skip-copy-src             do not copy source files to the build cache"
    echo "  --skip-gen-rosdep-list      do not modify the system rosdep list"
    echo "  --skip-colcon-build         do not run colcon build"
    echo "  --skip-gen-debian           do not generate Debian metadata (Phase 7)"
    echo "  --skip-build-deb            do not build .deb packages (Phase 8)"
    echo "  --workspace=WORKSPACE_DIR   source workspace directory"
    echo "  --output=OUTPUT_DIR         output directory for build artifacts and .deb files"
}

while true; do
    case "$1" in
	-h|--help)
	    print_usage
	    exit 0
	    ;;
	--skip-rosdep-install)
	    rosdep_install=n
	    shift 1
	    ;;
	--skip-copy-src)
	    copy_src=n
	    shift 1
	    ;;
	--skip-gen-rosdep-list)
	    gen_rosdep_list=n
	    shift 1
	    ;;
	--skip-colcon-build)
	    colcon_build=n
	    shift 1
	    ;;
	--skip-gen-debian)
	    gen_debian=n
	    shift 1
	    ;;
	--skip-build-deb)
	    build_deb=n
	    shift 1
	    ;;
	--workspace)
	    workspace_dir="$2"
	    shift 2
	    ;;
	--output)
	    output_dir="$2"
	    shift 2
	    ;;
	--)
	    shift
	    break
	    ;;
	*)
	    echo "error: invalid option: ${1}" >&2
	    print_usage >&2
	    exit 1
	    ;;
    esac
done

if [ -z "$workspace_dir" ] || [ -z "$output_dir" ]; then
    echo "Both --workspace and --output must be provided" >&2
    print_usage >&2
    exit 1
fi

# Locate utility scripts
export script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
export workspace_dir=$(realpath "$workspace_dir")
export output_dir=$(realpath "$output_dir")

# Set ROS distribution (default to humble for Autoware)
export ROS_DISTRO=${ROS_DISTRO:-humble}

# Set ROS installation prefix (can be overridden via environment variable)
# Default: /opt/ros/${ROS_DISTRO}
export ROS_INSTALL_PREFIX="${ROS_INSTALL_PREFIX:-/opt/ros/${ROS_DISTRO}}"

# Helper function to print timestamped messages
print_phase() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo ""
    echo "[$timestamp] $1"
}

print_phase "Build Configuration"
echo "  ROS Distribution: $ROS_DISTRO"
echo "  Install Prefix: $ROS_INSTALL_PREFIX"

# Set working directory to the parent of this script.
cd "$script_dir"

# All build artifacts go directly in the output directory
export top_work_dir="$output_dir"
export colcon_work_dir="$top_work_dir/sources"
export config_dir="/config"
export release_dir="$top_work_dir/dist"
export pkg_build_dir="$top_work_dir/build"

# File management strategy:
# - All build artifacts go directly in the output directory
# - The dist/ subdirectory contains the final .deb packages
# - We check dist/ for existing packages to avoid rebuilding
export check_dir="$release_dir"
export final_output_dir="$release_dir"

# Create timestamped log directory
export log_timestamp=$(date '+%Y-%m-%d_%H-%M-%S')
export log_base_dir="$top_work_dir/log"
export log_dir="$log_base_dir/$log_timestamp"
mkdir -p "$log_dir"

# Create/update 'latest' symlink
rm -f "$log_base_dir/latest"
ln -s "$log_timestamp" "$log_base_dir/latest"

# Log file paths with numbered prefixes
export deb_pkgs_file="$log_dir/06-deb_pkgs.txt"
export successful_pkgs_file="$log_dir/08-successful_pkgs.txt"
export failed_pkgs_file="$log_dir/08-failed_pkgs.txt"
export skipped_pkgs_file="$log_dir/08-skipped_pkgs.txt"

# export generate_debian_script="$script_dir/generate-debian.sh"
export rosdep_gen_script="$script_dir/generate-rosdep-commands.sh"
export make_deb_script="$script_dir/make-deb.sh"

make_pkg_work_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$pkg_build_dir/$pkg_name"
}
export -f make_pkg_work_dir

make_pkg_config_dir() {
    pkg_name="$1"
    shift || return 1
    echo "$config_dir/$pkg_name"
}
export -f make_pkg_config_dir

# Prepare the working directory
print_phase "Phase 1: Preparing working directories"
./prepare.sh

# Copy source files
print_phase "Phase 2: Copying source files"
./copy-src.sh

# Install dependencies
print_phase "Phase 3: Installing dependencies"
./install-deps.sh

# Compile the whole repository
print_phase "Phase 4: Compiling packages"
./build-src.sh
source "$colcon_work_dir/install/setup.bash"

# Create a rosdep list file
print_phase "Phase 5: Generating rosdep list"
./create-rosdep-list.sh

# Create a Debian package list file
print_phase "Phase 6: Creating package list"
./create-package-list.sh

# Copy or generate Debian control/rules files
if [[ "$gen_debian" == "y" ]]; then
    print_phase "Phase 7: Generating Debian metadata"
    python3 "$script_dir/generate_debian_dir.py"
else
    print_phase "Phase 7: Skipping Debian metadata generation (--skip-gen-debian)"
fi

# Build Debian packages
if [[ "$build_deb" == "y" ]]; then
    print_phase "Phase 8: Building Debian packages"
    python3 "$script_dir/build_deb.py"
else
    print_phase "Phase 8: Skipping Debian package builds (--skip-build-deb)"
fi

# Since we're working directly in the output directory, packages are already in the right place
echo "Packages are in: $release_dir"

# Print detailed summary
echo ""
echo "==============================================="
echo "           BUILD SUMMARY                      "
echo "==============================================="

# Count packages
total_pkgs=$(wc -l < "$deb_pkgs_file" 2>/dev/null || echo 0)
successful_pkgs=$(wc -l < "$successful_pkgs_file" 2>/dev/null || echo 0)
failed_pkgs=$(wc -l < "$failed_pkgs_file" 2>/dev/null || echo 0)
skipped_pkgs=$(wc -l < "$skipped_pkgs_file" 2>/dev/null || echo 0)
output_debs=$(find "$final_output_dir" -name "*.deb" 2>/dev/null | wc -l || echo 0)

echo ""
echo "📊 Package Statistics:"
echo "  Total packages found:      $total_pkgs"
echo "  ⏭️  Previously built:       $skipped_pkgs (skipped)"
echo "  ✅ Successfully built:     $successful_pkgs (new)"
echo "  ❌ Failed to build:        $failed_pkgs"
echo "  📦 Output .deb files:      $output_debs"

# Calculate build success rate (for packages that were actually built, not skipped)
attempted_pkgs=$((successful_pkgs + failed_pkgs))
if [ "$attempted_pkgs" -gt 0 ]; then
    build_success_rate=$(( successful_pkgs * 100 / attempted_pkgs ))
    echo "  Build success rate:        ${build_success_rate}% (of $attempted_pkgs attempted)"
fi

# Calculate overall completion rate
completed_pkgs=$((successful_pkgs + skipped_pkgs))
if [ "$total_pkgs" -gt 0 ]; then
    completion_rate=$(( completed_pkgs * 100 / total_pkgs ))
    echo "  Overall completion:        ${completion_rate}% ($completed_pkgs/$total_pkgs)"
fi

# Sanity check
accounted_pkgs=$((successful_pkgs + failed_pkgs + skipped_pkgs))
if [ "$accounted_pkgs" -ne "$total_pkgs" ]; then
    unaccounted=$((total_pkgs - accounted_pkgs))
    echo "  ⚠️  Unaccounted packages:   $unaccounted"
fi

echo ""
echo "📁 Directories:"
echo "  Output directory:  $top_work_dir"
echo "  Packages directory: $release_dir"
echo "  Log directory:     $log_dir"

# Show failed packages if any
if [ "$failed_pkgs" -gt 0 ]; then
    echo ""
    echo "❌ Failed Packages (first 10):"
    head -10 "$failed_pkgs_file" | while read pkg; do
        echo "  - $pkg"
        # Check if error log exists
        err_file="$pkg_build_dir/$pkg/build.err"
        if [ -f "$err_file" ] && [ -s "$err_file" ]; then
            echo "    → Error log: $err_file"
        fi
    done

    if [ "$failed_pkgs" -gt 10 ]; then
        echo "  ... and $((failed_pkgs - 10)) more"
    fi

    echo ""
    echo "💡 To investigate failures:"
    echo "  1. Check individual error logs:"
    echo "     cat $pkg_build_dir/PACKAGE_NAME/build.err"
    echo "  2. View complete failed list:"
    echo "     cat $failed_pkgs_file"
    echo "  3. Run diagnostic tool:"
    echo "     ./check-build-results.py --workspace $workspace_dir --output $top_work_dir"
fi

echo ""
echo "==============================================="

# Exit with appropriate code
if [ "$failed_pkgs" -gt 0 ]; then
    echo "⚠️  Build completed with failures"
    echo ""
    echo "Next steps:"
    echo "  1. Check build logs for failed packages"
    echo "  2. Try installing successfully built packages:"
    echo "     ./install-partial.py --config CONFIG --mode safe"
    exit 1
else
    echo "✅ Build completed successfully!"
    echo ""
    echo "Next steps:"
    echo "  1. Install packages:"
    echo "     ./install-packages.py --config CONFIG"
    echo "  2. Test installation:"
    echo "     ./test-installation.sh --config CONFIG"
    exit 0
fi
