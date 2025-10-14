# Build Debian files for each package in topological order.
cd "$colcon_work_dir"
echo "info: generate Debian packaging scripts"

copy_or_create_debian_dir() {
    set -Eeuo pipefail

    pkg_name="$1"
    shift

    pkg_dir="$1"
    shift

    pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
    pkg_config_dir="$(make_pkg_config_dir $pkg_name)"
    out_file="$pkg_work_dir/gen_deb.out"
    err_file="$pkg_work_dir/gen_deb.err"
    src_debian_dir="$config_dir/$pkg_name/debian"
    dst_debian_dir="$pkg_work_dir/debian"

    mkdir -p "$pkg_work_dir"
    mkdir -p "$pkg_config_dir"
    truncate -s 0 "$out_file"
    truncate -s 0 "$err_file"

    if [ -d "$src_debian_dir" ]; then
	echo "info: copy provided debian directory for $pkg_name"

	rsync -av --delete "$src_debian_dir/" "$dst_debian_dir/" >> "$out_file" 2>> "$err_file" ||
	    echo "error: fail to copy debian directory for ${pkg_name}" >&2
    else
	echo "info: run bloom-generate for $pkg_name"
	(
	    set -Eeuo pipefail
	    cd "$pkg_config_dir"
	    bloom-generate rosdebian --ros-distro "$ROS_DISTRO" "$pkg_dir"

	    # Apply custom rules.em template to support configurable install prefix
	    custom_rules_template="$script_dir/templates/debian/rules.em"
	    if [ -f "$custom_rules_template" ]; then
	        echo "info: applying custom rules.em template for $pkg_name"
	        cp "$custom_rules_template" "$pkg_config_dir/debian/rules.em"
	        # Reprocess the rules template with bloom
	        bloom-generate rosdebian --process-template-files --ros-distro "$ROS_DISTRO" "$pkg_dir"
	    fi

	    rsync -av --delete "$pkg_config_dir/debian/" "$dst_debian_dir/"
	) >> "$out_file" 2>> "$err_file" ||
	    echo "error: fail to generate Debain files for ${pkg_name}" >&2

    fi
}
export -f copy_or_create_debian_dir

# Use fewer parallel jobs for I/O-heavy operations like bloom-generate
# Too many parallel bloom processes can cause file system contention
njobs_io=$(( ( $(nproc) + 1 ) / 2 ))  # Use half the CPU cores for I/O operations

# Use unique semaphore ID for this operation to avoid blocking other scripts
sem_id="generate_debian_$$_${RANDOM}"

colcon list --base-paths src | cut -f1-2 | \
    while read -r pkg_name pkg_dir; do
	# Prepare the working directory for the package
	pkg_dir=$(realpath "$pkg_dir")
	sem --id "$sem_id" "-j${njobs_io}" copy_or_create_debian_dir "$pkg_name" "$pkg_dir"
    done
sem --id "$sem_id" --wait
