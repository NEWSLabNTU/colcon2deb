"""Install ROS dependencies using rosdep."""

import subprocess
import tempfile
from pathlib import Path

from .config import BuildConfig
from .utils import logger, print_phase, run_command, add_subtask, get_display


def generate_rosdep_commands(config: BuildConfig) -> str:
    """Generate shell script to install dependencies.

    Mimics rosdep install but generates a script for better control.
    Based on helper/generate-rosdep-commands.sh logic.
    """
    # Source ROS setup to get rosdep environment
    ros_setup = f"/opt/ros/{config.ros_distro}/setup.bash"

    # First, try to detect which packages can't be resolved
    # Note: Don't use 'set -u' because ROS setup.bash has unbound variables
    detect_cmd = [
        "bash",
        "-c",
        f"""
        source {ros_setup}
        cd {config.colcon_work_dir}/src

        # Run rosdep once to see if there are unresolvable packages
        rosdep install --from-paths . --ignore-src --simulate --default-yes 2>&1 | \\
            grep "Cannot locate rosdep definition for" | \\
            sed 's/.*\\[\\(.*\\)\\].*/\\1/' || true
        """
    ]

    result = run_command(detect_cmd, capture_output=True, check=False)
    skip_keys = result.stdout.strip().split('\n') if result.stdout.strip() else []

    # Build skip-keys argument
    skip_args = ""
    if skip_keys:
        logger.debug(f"Skipping unresolvable rosdep keys: {', '.join(skip_keys)}")
        for key in skip_keys:
            if key:  # Skip empty strings
                skip_args += f" --skip-keys {key}"

    # Generate rosdep resolve commands for all packages
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        source {ros_setup}
        cd {config.colcon_work_dir}/src

        # Get all package dependencies, skipping unresolvable keys
        # Use grep with || true to avoid exit code 1 when no matches
        rosdep install --from-paths . --ignore-src --simulate --default-yes{skip_args} 2>&1 | \\
            (grep "^apt-get install" || true) | \\
            sed 's/^/sudo /'
        """
    ]

    result = run_command(cmd, capture_output=True)
    return result.stdout


def update_package_lists(config: BuildConfig):
    """Run apt update to refresh package caches."""
    add_subtask(3, "Updating package lists...")

    cmd = ["sudo", "apt", "update"]
    log_file = config.log_dir / "apt_update.log"

    try:
        run_command(cmd, log_file=log_file, check=False)
        add_subtask(3, "✓ Package lists updated")
    except subprocess.CalledProcessError:
        add_subtask(3, "⚠ apt update had errors (continuing anyway)")


def install_dependencies(config: BuildConfig):
    """Install ROS dependencies for all packages in workspace."""
    if config.skip_rosdep_install:
        add_subtask(3, "Skipped (--skip-rosdep-install)")
        return

    # Update package lists first
    update_package_lists(config)

    add_subtask(3, "Generating dependency install script...")

    # Generate install script
    try:
        install_commands = generate_rosdep_commands(config)
    except subprocess.CalledProcessError as e:
        logger.error("Failed to generate rosdep commands")
        raise

    if not install_commands.strip():
        add_subtask(3, "No dependencies to install")
        return

    # Save install script for reference
    install_script_path = config.log_dir / "install_script.sh"
    with open(install_script_path, 'w') as f:
        f.write("#!/bin/bash\n")
        f.write("set -eo pipefail\n\n")
        f.write(install_commands)

    add_subtask(3, "Installing dependencies...")

    # Execute installation
    cmd = ["bash", str(install_script_path)]
    log_file = config.log_dir / "apt_install.log"

    try:
        run_command(cmd, log_file=log_file)
        add_subtask(3, "✓ Dependencies installed")
    except subprocess.CalledProcessError as e:
        logger.error("Failed to install dependencies")
        logger.error(f"  See log: {log_file}")
        raise
