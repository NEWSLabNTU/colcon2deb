#!/usr/bin/env bash
# Create directories first before trying to create files in them
mkdir -p "$top_work_dir"
mkdir -p "$colcon_work_dir"
mkdir -p "$release_dir"
mkdir -p "$log_dir"
mkdir -p "$log_logs_dir"
mkdir -p "$log_reports_dir"
mkdir -p "$log_scripts_dir"
mkdir -p "$pkg_build_dir"

# Now safe to create/truncate report files
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

# Only scan for packages if src directory exists
# This will be populated by copy-src.sh later
if [ -d "$colcon_work_dir/src" ]; then
    cd "$colcon_work_dir"
    # Directory creation is I/O bound, use fewer parallel jobs
    njobs_io=$(( ( $(nproc) + 1 ) / 2 ))

    # Use GNU parallel instead of sem with exported functions
    # This avoids bash function export issues with /bin/sh
    colcon list --base-paths src | cut -f1 | \
        parallel -j "$njobs_io" "mkdir -p '$pkg_build_dir'/{}; rm -f '$pkg_build_dir'/{}/*.out '$pkg_build_dir'/{}/*.err"
fi
