# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo 'info: build Debian packages'

# Export ROS_INSTALL_PREFIX for debian/rules to use
# This allows packages to be installed to custom locations
export ROS_INSTALL_PREFIX="${ROS_INSTALL_PREFIX:-/opt/ros/${ROS_DISTRO}}"
echo "info: using install prefix: $ROS_INSTALL_PREFIX"

# Use 1/4 of CPU cores for parallel package builds to avoid resource exhaustion
# Each package build itself uses parallel compilation via DEB_BUILD_OPTIONS
njobs=$(( ( $(nproc) + 3 ) / 4 ))
export DEB_BUILD_OPTIONS="parallel=$(nproc)" # Enable parallel compilation within each package

build_deb() {
    # Use set -e for proper error handling
    set -euo pipefail

    pkg_name="$1"
    shift
    pkg_dir="$1"
    shift

    # Prepare the working directory for the package
    pkg_name_dashed="${pkg_name//_/-}"
    pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
    out_file="$pkg_work_dir/build.out"
    err_file="$pkg_work_dir/build.err"

    truncate -s 0 "$out_file"
    truncate -s 0 "$err_file"

    # If the Debian package is already built, skip the build.
    # Check in the check_dir (either output_dir if specified, or release_dir)
    # Note: Don't quote the pattern in find command to allow glob expansion
    deb_path=$(find "$check_dir" -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" 2>/dev/null | head -n1 || true)
    if [ -n "$deb_path" ]; then
	echo "info: skip $pkg_name that its Debian package is already built"
	echo "$pkg_name" >> "$skipped_pkgs_file"
	return 0
    fi

    echo "info: build Debian package for $pkg_name"

    # Try to build the package, capture success/failure
    local build_result=0
    (
	set -e
	echo "$pkg_dir"
	echo "$pkg_work_dir"
	cd "$pkg_dir"

	# Copy Debian packaging scripts
	rm -rf debian .obj-* ros-$ROS_DISTRO-"${pkg_name_dashed}"_*.deb
	cp -r -t "$pkg_dir" "$pkg_work_dir/debian"

	# Build the package
	fakeroot debian/rules binary

	# Save generated .deb/.ddeb files
	deb_path=$(find .. -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" | head -n1)
	if [ -n "$deb_path" ]; then
	    mv -v "$deb_path" "$release_dir"
	fi

	ddeb_path=$(find .. -name "ros-${ROS_DISTRO}-${pkg_name_dashed}-dbgsym_*.ddeb" | head -n1)
	if [ -n "$ddeb_path" ]; then
	    mv -v "$ddeb_path" "$release_dir"
	fi
    ) > "$out_file" 2> "$err_file" || build_result=$?

    # Check if the .deb file was actually created and moved
    pkg_deb_in_release=$(find "$release_dir" -name "ros-${ROS_DISTRO}-${pkg_name_dashed}_*.deb" 2>/dev/null | head -n1 || true)

    # Check the build result and verify the package was created
    if [ $build_result -eq 0 ] && [ -n "$pkg_deb_in_release" ]; then
	echo "info: build successful for ${pkg_name}"
	echo "$pkg_name" >> "$successful_pkgs_file"
	return 0
    elif [ $build_result -eq 0 ] && [ -z "$pkg_deb_in_release" ]; then
	echo "warning: build command succeeded but no .deb file found for ${pkg_name}"
	echo "$pkg_name" >> "$failed_pkgs_file"
	return 1
    else
	echo "error: fail to build Debian package for ${pkg_name}"
	echo "$pkg_name" >> "$failed_pkgs_file"
	return 1
    fi
}
export -f build_deb

# Use unique semaphore ID for package builds to avoid blocking other scripts
sem_id="build_deb_$$_${RANDOM}"

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	pkg_dir=$(realpath "$pkg_dir")
	sem --id "$sem_id" "-j${njobs}" build_deb "$pkg_name" "$pkg_dir"
    done
sem --id "$sem_id" --wait
