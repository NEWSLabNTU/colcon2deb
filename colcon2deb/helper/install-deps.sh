# Helper: run command with sudo if not root
run_privileged() {
    if [ "$(id -u)" = "0" ]; then
        "$@"
    else
        sudo "$@"
    fi
}

# Run `apt update` to refresh package caches
echo 'info: updating package lists...'
apt_update_ts=$(date '+%Y-%m-%d_%H-%M-%S')
run_privileged apt update > "$log_dir/${apt_update_ts}_apt_update.log" 2>&1 || {
    echo 'warning: apt update had errors (continuing anyway)' >&2
}
echo '  ✓ Package lists updated'

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    echo 'info: installing dependencies...'

    # Generate the install script and execute it
    install_script=$(mktemp /tmp/install_deps_XXXXXX.sh)
    ./generate-rosdep-commands.sh > "$install_script"

    # Save install script to log for reference (no datetime prefix - it's a script, not a log)
    cp "$install_script" "$log_dir/install_script.sh"

    # Execute the installation with logging
    apt_install_ts=$(date '+%Y-%m-%d_%H-%M-%S')
    apt_install_log="$log_dir/${apt_install_ts}_apt_install.log"
    bash "$install_script" > "$apt_install_log" 2>&1 || {
        echo "error: failed to install dependencies" >&2
        echo "  See log: $apt_install_log" >&2
        tail -n 30 "$apt_install_log" >&2
        rm -f "$install_script"
        exit 1
    }
    rm -f "$install_script"

    echo '  ✓ Dependencies installed'
else
    echo 'info: skip installing dependencies'
fi
