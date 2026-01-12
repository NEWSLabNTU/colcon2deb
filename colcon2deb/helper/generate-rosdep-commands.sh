#!/usr/bin/env bash
# Generate dependency installation commands using rosdep install --simulate
#
# This script uses rosdep's built-in simulation mode which:
# - Resolves all dependencies in a single call
# - Handles parallelism internally
# - Outputs commands in a structured format
#
# Output: A bash script that installs apt and pip dependencies

set -eo pipefail

cd "$colcon_work_dir"

# Source ROS setup (temporarily disable strict mode for ROS scripts)
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

echo "info: resolving dependencies with rosdep install --simulate" >&2

# Run rosdep install in simulate mode
# --reinstall: show all deps even if installed
# --ignore-src: ignore packages in src/
# -r: continue on errors (some keys may be missing)
simulate_output=$(rosdep install --simulate --reinstall \
    --from-paths src --ignore-src \
    --rosdistro "${ROS_DISTRO:-humble}" \
    -r 2>&1) || true

# Save raw output for debugging
echo "$simulate_output" > "$log_logs_dir/phase5_rosdep.log" 2>/dev/null || true

# Parse the simulate output to extract package names
# Format is:
#   #[apt] Installation commands:
#     sudo -H apt-get install package1
#     sudo -H apt-get install package2
#   #[pip] Installation commands:
#     sudo -H pip install pypkg1

apt_packages=()
pip_packages=()
current_mode=""

while IFS= read -r line; do
    # Detect section headers
    if [[ "$line" =~ ^\#\[apt\] ]]; then
        current_mode="apt"
        continue
    elif [[ "$line" =~ ^\#\[pip\] ]]; then
        current_mode="pip"
        continue
    fi

    # Extract package name from install commands
    if [[ "$current_mode" == "apt" && "$line" =~ apt-get[[:space:]]+install[[:space:]]+(.+)$ ]]; then
        pkg="${BASH_REMATCH[1]}"
        # Remove any trailing whitespace
        pkg="${pkg%% }"
        apt_packages+=("$pkg")
    elif [[ "$current_mode" == "pip" && "$line" =~ pip[[:space:]]+install[[:space:]]+(.+)$ ]]; then
        pkg="${BASH_REMATCH[1]}"
        pkg="${pkg%% }"
        pip_packages+=("$pkg")
    fi
done <<< "$simulate_output"

# Report counts
echo "info: found ${#apt_packages[@]} apt packages, ${#pip_packages[@]} pip packages" >&2

# Generate the install script
echo "#!/usr/bin/env bash"
echo "set -e  # Exit on error"
echo ""
echo "# Helper: run command with sudo if not root"
echo "run_privileged() {"
echo "    if [ \"\$(id -u)\" = \"0\" ]; then"
echo "        \"\$@\""
echo "    else"
echo "        sudo \"\$@\""
echo "    fi"
echo "}"
echo ""

if [ ${#apt_packages[@]} -gt 0 ]; then
    echo "echo \"Installing ${#apt_packages[@]} APT packages...\" >&2"
    echo -n "run_privileged apt-get install -y"
    for pkg in "${apt_packages[@]}"; do
        echo -n " $pkg"
    done
    echo ""
    echo ""
fi

if [ ${#pip_packages[@]} -gt 0 ]; then
    echo "echo \"Installing ${#pip_packages[@]} pip packages...\" >&2"
    echo -n "pip install -U"
    for pkg in "${pip_packages[@]}"; do
        echo -n " $pkg"
    done
    echo ""
    echo ""
fi

if [ ${#apt_packages[@]} -eq 0 ] && [ ${#pip_packages[@]} -eq 0 ]; then
    echo "echo \"No dependencies to install\" >&2"
fi
