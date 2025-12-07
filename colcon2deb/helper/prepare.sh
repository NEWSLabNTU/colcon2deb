#!/usr/bin/env bash
echo 'info: prepare working directories'

# Create directories first before trying to create files in them
mkdir -p "$top_work_dir"
mkdir -p "$colcon_work_dir"
mkdir -p "$release_dir"
mkdir -p "$log_dir"
mkdir -p "$pkg_build_dir"

# Now safe to create/truncate log files
truncate -s 0 "$deb_pkgs_file" 2>/dev/null || true
truncate -s 0 "$successful_pkgs_file" 2>/dev/null || true
truncate -s 0 "$failed_pkgs_file" 2>/dev/null || true
truncate -s 0 "$skipped_pkgs_file" 2>/dev/null || true

# Prevent colcon from erroneously scan this folder
touch "$top_work_dir/COLCON_IGNORE"

# Helper function to get package work directory
make_pkg_work_dir() {
    local pkg_name="$1"
    echo "$pkg_build_dir/$pkg_name"
}
export -f make_pkg_work_dir

clean_work_dir() {
    set -e
    work_dir=$1
    shift
    mkdir -p "$work_dir"
    rm -f "$work_dir"/*.out "$work_dir"/*.err
}
export -f clean_work_dir

# Only scan for packages if src directory exists
# This will be populated by copy-src.sh later
if [ -d "$colcon_work_dir/src" ]; then
    cd "$colcon_work_dir"
    # Directory creation is I/O bound, use fewer parallel jobs
    njobs_io=$(( ( $(nproc) + 1 ) / 2 ))

    # Use unique semaphore ID for this operation
    sem_id="prepare_$$_${RANDOM}"

    colcon list --base-paths src | cut -f1-2 | \
        while read -r pkg_name pkg_dir; do
	    # Prepare the working directory for the package
	    pkg_dir=$(realpath "$pkg_dir")
	    pkg_work_dir="$(make_pkg_work_dir $pkg_name)"
	    sem --id "$sem_id" "-j${njobs_io}" clean_work_dir "$pkg_work_dir"
        done
    sem --id "$sem_id" --wait
else
    echo "info: skipping package directory cleanup (src not yet copied)"
fi
