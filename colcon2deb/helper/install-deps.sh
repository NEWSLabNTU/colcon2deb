# Run `apt update` to refresh package caches
echo 'info: updating package lists...'
sudo apt update > "$log_dir/03-apt_update.log" 2>&1 || {
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

    # Save install script to log for reference
    cp "$install_script" "$log_dir/03-install_script.sh"

    # Execute the installation with logging
    bash "$install_script" > "$log_dir/03-apt_install.log" 2>&1 || {
        echo "error: failed to install dependencies" >&2
        echo "  See log: $log_dir/03-apt_install.log" >&2
        tail -n 30 "$log_dir/03-apt_install.log" >&2
        rm -f "$install_script"
        exit 1
    }
    rm -f "$install_script"

    echo '  ✓ Dependencies installed'
else
    echo 'info: skip installing dependencies'
fi
