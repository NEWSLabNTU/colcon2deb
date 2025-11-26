"""Install ROS dependencies using rosdep."""

import subprocess
import tempfile
from pathlib import Path

from .config import BuildConfig
from .utils import logger, print_phase, run_command, add_subtask, get_display


def generate_rosdep_commands(config: BuildConfig) -> str:
    """Generate shell script to install dependencies.

    Uses rosdep --simulate to collect all packages, then installs them
    in a single apt-get command for better efficiency.
    """
    # Source ROS setup to get rosdep environment
    ros_setup = f"/opt/ros/{config.ros_distro}/setup.bash"

    logger.info("Detecting unresolvable rosdep keys...")
    # First, try to detect which packages can't be resolved
    # Note: Don't use 'set -u' because ROS setup.bash has unbound variables
    # Use r-string to preserve backslashes for sed regex
    detect_script = rf"""
        source {ros_setup}
        cd {config.colcon_work_dir}/src

        # Run rosdep once to see if there are unresolvable packages
        # Output is like: "package_name: Cannot locate rosdep definition for [key_name]"
        rosdep install --from-paths . --ignore-src --rosdistro={config.ros_distro} --simulate --default-yes 2>&1 | \
            grep "Cannot locate rosdep definition for" | \
            sed 's/.*Cannot locate rosdep definition for \[\(.*\)\].*/\1/' || true
        """
    detect_cmd = ["bash", "-c", detect_script]

    result = run_command(detect_cmd, capture_output=True, check=False)

    # Debug: Save detection script and output
    detection_log = config.log_dir / "rosdep_detection.log"
    with open(detection_log, 'w') as f:
        f.write("=== Detection script ===\n")
        f.write(detect_script)
        f.write("\n\n=== Detection command output ===\n")
        f.write(f"stdout:\n{result.stdout}\n\n")
        f.write(f"stderr:\n{result.stderr}\n\n")
        f.write(f"returncode: {result.returncode}\n")

    skip_keys = result.stdout.strip().split('\n') if result.stdout.strip() else []
    # Remove empty strings and duplicates
    skip_keys = list(set([k.strip() for k in skip_keys if k.strip()]))

    # Build skip-keys argument
    skip_args = ""
    if skip_keys:
        logger.info(f"Found {len(skip_keys)} unresolvable rosdep keys: {', '.join(skip_keys)}")
        for key in skip_keys:
            skip_args += f" --skip-keys {key}"
    else:
        logger.info("No unresolvable rosdep keys found")

    logger.info("Collecting package dependencies using rosdep --simulate...")
    # Use rosdep --simulate to collect all packages, then install in single command
    # Use r-string to preserve backslashes for sed regex
    collect_script = rf"""
        source {ros_setup}
        cd {config.colcon_work_dir}/src

        # Save full rosdep output for debugging
        rosdep install --from-paths . --ignore-src --rosdistro={config.ros_distro} --simulate --default-yes{skip_args} 2>&1 | tee /output/log/rosdep_full_output.log

        # Get all package dependencies using --simulate, skipping unresolvable keys
        # rosdep --simulate outputs lines like:  sudo -H apt-get install -y pkg1
        # Use || true to prevent grep from failing when no matches found
        packages=$(cat /output/log/rosdep_full_output.log | \
            grep -E "sudo.*apt-get install" || true)

        # Extract package names from all install lines
        if [ -n "$packages" ]; then
            packages=$(echo "$packages" | \
                sed 's/^.*apt-get install[[:space:]]*-y[[:space:]]*//' | \
                tr '\n' ' ' | \
                sed 's/[[:space:]]\+/ /g' | \
                sed 's/^[[:space:]]*//' | \
                sed 's/[[:space:]]*$//')

            # Generate install command if we have packages
            if [ -n "$packages" ]; then
                echo "sudo apt-get install -y $packages"
            fi
        fi
        """
    cmd = ["bash", "-c", collect_script]

    result = run_command(cmd, capture_output=True, check=False)

    # If command failed, log and re-raise
    if result.returncode != 0:
        logger.error(f"rosdep command failed with exit code {result.returncode}")
        if result.stderr:
            logger.error(f"stderr: {result.stderr}")
        raise subprocess.CalledProcessError(result.returncode, cmd, result.stdout, result.stderr)

    if result.stdout.strip():
        # Count packages in the install command
        pkg_count = len(result.stdout.strip().split()) - 3  # subtract "sudo apt-get install -y"
        logger.info(f"Generated install command with approximately {pkg_count} packages")
    else:
        logger.warning("rosdep --simulate generated no packages")

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


def fix_ros2_namespace_issues(config: BuildConfig):
    """Fix ROS 2 Humble doubled namespace issue by creating symlinks.

    Some ROS 2 Humble packages have doubled namespace structure that causes
    include errors. Create symlinks to allow single namespace access.
    """
    packages_to_fix = ['can_msgs', 'geographic_msgs', 'udp_msgs']
    ros_include_dir = Path(f"/opt/ros/{config.ros_distro}/include")

    for pkg in packages_to_fix:
        pkg_dir = ros_include_dir / pkg
        if pkg_dir.exists():
            msg_symlink = pkg_dir / "msg"
            msg_target = pkg_dir / pkg / "msg"

            if msg_target.exists() and not msg_symlink.exists():
                try:
                    msg_symlink.symlink_to(f"{pkg}/msg")
                    logger.debug(f"Created symlink for {pkg}")
                except Exception as e:
                    logger.warning(f"Failed to create symlink for {pkg}: {e}")


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

    # Always save the install script, even if empty, for debugging
    install_script_path = config.log_dir / "install_script.sh"
    logger.info(f"Saving install script to: {install_script_path}")

    with open(install_script_path, 'w') as f:
        f.write("#!/bin/bash\n")
        f.write("set -eo pipefail\n\n")
        if install_commands.strip():
            f.write(install_commands)
            logger.info(f"Install script contains {len(install_commands.strip().split())} items")
        else:
            f.write("# No dependencies to install\n")
            logger.info("Install script is empty - no dependencies found")

    if not install_commands.strip():
        add_subtask(3, "No dependencies to install")
        logger.warning("rosdep generated no install commands - this may indicate an issue")
        return

    add_subtask(3, "Installing dependencies...")
    logger.info(f"Executing install script: {install_script_path}")

    # Execute installation
    cmd = ["bash", str(install_script_path)]
    log_file = config.log_dir / "apt_install.log"

    try:
        run_command(cmd, log_file=log_file)
        add_subtask(3, "✓ Dependencies installed")
        logger.info(f"Dependencies installed successfully - see log: {log_file}")
    except subprocess.CalledProcessError as e:
        logger.error("Failed to install dependencies")
        logger.error(f"  See log: {log_file}")
        raise

    # Fix ROS 2 Humble namespace issues after installing dependencies
    add_subtask(3, "Fixing ROS 2 namespace issues...")
    fix_ros2_namespace_issues(config)
    add_subtask(3, "✓ Namespace fixes applied")
