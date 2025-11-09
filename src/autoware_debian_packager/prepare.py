"""Prepare working directories and copy source files."""

import subprocess
import shutil
from pathlib import Path

from .config import BuildConfig
from .utils import logger, print_phase, ensure_dir, run_command, add_subtask


def setup_directories(config: BuildConfig):
    """Prepare working directories for build.

    Creates:
    - sources/ - Colcon workspace
    - dist/ - Final .deb packages
    - build/ - Per-package build artifacts
    - log/ - Build logs
    """
    add_subtask(1, "Creating directory structure")

    ensure_dir(config.colcon_work_dir)
    ensure_dir(config.release_dir)
    ensure_dir(config.pkg_build_dir)
    ensure_dir(config.log_dir)

    # Initialize log files
    config.deb_pkgs_file.touch()
    config.successful_pkgs_file.touch()
    config.failed_pkgs_file.touch()
    config.skipped_pkgs_file.touch()

    add_subtask(1, "✓ Directories prepared")


def copy_workspace(config: BuildConfig):
    """Copy source files from workspace to build area.

    Uses rsync for efficient copying with progress display.
    """
    if config.skip_copy_src:
        add_subtask(2, "Skipped (--skip-copy-src)")
        return

    src_dir = config.workspace_dir / "src"
    dst_dir = config.colcon_work_dir / "src"

    if not src_dir.exists():
        raise FileNotFoundError(f"Source directory not found: {src_dir}")

    add_subtask(2, f"Copying from {src_dir.name}/ to {dst_dir}")

    # Use rsync for efficient copying
    cmd = [
        "rsync",
        "-av",
        "--delete",
        f"{src_dir}/",
        str(dst_dir),
    ]

    log_file = config.log_dir / "rsync_copy.log"
    try:
        run_command(cmd, log_file=log_file)
        add_subtask(2, "✓ Source files copied")
    except subprocess.CalledProcessError as e:
        logger.error("Failed to copy source files")
        raise
