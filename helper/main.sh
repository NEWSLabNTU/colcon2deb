#!/usr/bin/env bash
set -e

# Parse options using getopt
OPTIONS=h
LONGOPTIONS=help,skip-rosdep-install,skip-copy-src,skip-gen-rosdep-list,skip-colcon-build,workspace:,output:
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
export workspace_dir=
export output_dir=

print_usage() {
    echo "Usage: $0 [OPTION]... --workspace=WORKSPACE_DIR --output=OUTPUT_DIR"
    echo "  -h,--help                   show this help message"
    echo "  --skip-rosdep-install       do not run `rosdep install`"
    echo "  --skip-copy-src             do not copy source files to the build cache"
    echo "  --skip-gen-rosdep-list      do not modify the system rosdep list"
    echo "  --skip-colcon-build         do not run `colcon build`"
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
echo "info: ROS_DISTRO=$ROS_DISTRO"
echo "info: ROS_INSTALL_PREFIX=$ROS_INSTALL_PREFIX"

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

export log_dir="$top_work_dir/log"
export deb_pkgs_file="$log_dir/deb_pkgs.txt"
export successful_pkgs_file="$log_dir/successful_pkgs.txt"
export failed_pkgs_file="$log_dir/failed_pkgs.txt"
export skipped_pkgs_file="$log_dir/skipped_pkgs.txt"

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
./prepare.sh

# Copy source files
./copy-src.sh

# Install dependencies
./install-deps.sh

# Compile the whole repository
./build-src.sh
source "$colcon_work_dir/install/setup.bash"

# Create a rosdep list file
./create-rosdep-list.sh

# Create a Debian package list file
./create-package-list.sh

# Copy or generate Debian control/rules files
./generate-debian-dir.sh

# Build Debian packages
./build-deb.sh

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
