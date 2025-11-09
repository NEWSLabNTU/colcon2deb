"""Install ROS dependencies using rosdep."""

import subprocess
import tempfile
from pathlib import Path

from .config import BuildConfig
from .utils import logger, print_phase, run_command


def generate_rosdep_commands(config: BuildConfig) -> str:
    """Generate shell script to install dependencies.

    Mimics rosdep install but generates a script for better control.
    Based on helper/generate-rosdep-commands.sh logic.
    """
    # Source ROS setup to get rosdep environment
    ros_setup = f"/opt/ros/{config.ros_distro}/setup.bash"

    # Generate rosdep resolve commands for all packages
    # Note: Don't use 'set -u' because ROS setup.bash has unbound variables
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        source {ros_setup}
        cd {config.colcon_work_dir}/src

        # Get all package dependencies
        # Use grep with || true to avoid exit code 1 when no matches
        rosdep install --from-paths . --ignore-src --simulate --default-yes 2>&1 | \\
            (grep "^apt-get install" || true) | \\
            sed 's/^/sudo /'
        """
    ]

    result = run_command(cmd, capture_output=True)
    return result.stdout


def update_package_lists(config: BuildConfig):
    """Run apt update to refresh package caches."""
    logger.info("info: updating package lists...")

    cmd = ["sudo", "apt", "update"]
    log_file = config.log_dir / "apt_update.log"

    try:
        run_command(cmd, log_file=log_file, check=False)
        logger.info("  ✓ Package lists updated")
    except subprocess.CalledProcessError:
        logger.warning("warning: apt update had errors (continuing anyway)")


def install_dependencies(config: BuildConfig):
    """Install ROS dependencies for all packages in workspace."""
    if config.skip_rosdep_install:
        logger.info("info: skip installing dependencies")
        return

    print_phase("Phase 3: Installing dependencies")

    # Update package lists first
    update_package_lists(config)

    logger.info("info: installing dependencies...")

    # Generate install script
    try:
        install_commands = generate_rosdep_commands(config)
    except subprocess.CalledProcessError as e:
        logger.error("Failed to generate rosdep commands")
        raise

    if not install_commands.strip():
        logger.info("  No dependencies to install")
        return

    # Save install script for reference
    install_script_path = config.log_dir / "install_script.sh"
    with open(install_script_path, 'w') as f:
        f.write("#!/bin/bash\n")
        f.write("set -eo pipefail\n\n")
        f.write(install_commands)

    logger.debug(f"Install script saved to: {install_script_path}")

    # Execute installation
    cmd = ["bash", str(install_script_path)]
    log_file = config.log_dir / "apt_install.log"

    try:
        run_command(cmd, log_file=log_file)
        logger.info("  ✓ Dependencies installed")
    except subprocess.CalledProcessError as e:
        logger.error("Failed to install dependencies")
        logger.error(f"  See log: {log_file}")
        raise
