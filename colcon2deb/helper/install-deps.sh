# Helper: run command with sudo if not root
run_privileged() {
    if [ "$(id -u)" = "0" ]; then
        "$@"
    else
        sudo "$@"
    fi
}

# Run `apt update` to refresh package caches
apt_update_ts=$(date '+%Y-%m-%d_%H-%M-%S')
run_privileged apt update > "$log_dir/${apt_update_ts}_apt_update.log" 2>&1 || {
    echo 'warning: apt update had errors (continuing anyway)' >&2
}

# Generate commands that is effectively the same with `rosdep install
# ...` and execute them.
if [ "$rosdep_install" = y ]; then
    # Generate the install script and execute it
    install_script=$(mktemp /tmp/install_deps_XXXXXX.sh)
    ./generate-rosdep-commands.sh > "$install_script"

    # Save install script to log for reference
    cp "$install_script" "$log_dir/install_script.sh"

    # Execute the installation with logging
    apt_install_ts=$(date '+%Y-%m-%d_%H-%M-%S')
    apt_install_log="$log_dir/${apt_install_ts}_apt_install.log"
    bash "$install_script" > "$apt_install_log" 2>&1 || {
        echo "error: failed to install dependencies" >&2
        tail -n 30 "$apt_install_log" >&2
        rm -f "$install_script"
        exit 1
    }
    rm -f "$install_script"
fi
