"""Compile ROS workspace using colcon."""

import subprocess
from pathlib import Path

from .config import BuildConfig
from .utils import logger, print_phase, run_command


def build_workspace(config: BuildConfig):
    """Compile all packages in the workspace using colcon build.

    Uses colcon's event handlers for progress-only output, with
    detailed compilation logs saved to file.
    """
    if config.skip_colcon_build:
        logger.info("info: skip compiling packages")
        return

    print_phase("Phase 4: Compiling packages")
    logger.info("info: compiling packages (this may take a while...)")

    # Source ROS environment
    ros_setup = f"/opt/ros/{config.ros_distro}/setup.bash"

    # Colcon build command with progress-only event handlers
    # Note: Don't use 'set -u' because ROS setup.bash has unbound variables
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        source {ros_setup}
        cd {config.colcon_work_dir}

        colcon build \\
            --base-paths src \\
            --cmake-args -DCMAKE_BUILD_TYPE=Release \\
            --event-handlers console_direct+
        """
    ]

    log_file = config.log_dir / "colcon_build.log"

    try:
        run_command(cmd, log_file=log_file)
        logger.info("  ✓ Compilation complete")
    except subprocess.CalledProcessError as e:
        logger.error("Colcon build failed")
        logger.error(f"  See log: {log_file}")
        raise


def source_install_setup(config: BuildConfig):
    """Source the install/setup.bash for subsequent commands.

    Note: This doesn't actually source in the current Python process,
    but subsequent commands can source it.

    Returns the path to source in shell commands.
    """
    setup_bash = config.colcon_work_dir / "install" / "setup.bash"

    if not setup_bash.exists():
        raise FileNotFoundError(
            f"Colcon install setup not found: {setup_bash}\n"
            "Did colcon build complete successfully?"
        )

    logger.debug(f"Install setup available at: {setup_bash}")
    return setup_bash
