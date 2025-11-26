"""Build configuration and state management."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class BuildConfig:
    """Configuration for building Debian packages from ROS workspace."""

    workspace_dir: Path
    output_dir: Path
    ros_distro: str = "humble"
    ros_install_prefix: Optional[str] = None

    # Optional flags
    skip_rosdep_install: bool = False
    skip_copy_src: bool = False
    skip_gen_rosdep_list: bool = False
    skip_colcon_build: bool = False

    def __post_init__(self):
        """Initialize paths and defaults."""
        self.workspace_dir = Path(self.workspace_dir).resolve()
        self.output_dir = Path(self.output_dir).resolve()

        # Set default install prefix if not provided
        if self.ros_install_prefix is None:
            self.ros_install_prefix = f"/opt/ros/{self.ros_distro}"

        # Define directory structure
        self.colcon_work_dir = self.output_dir / "sources"
        self.config_dir = Path("/config")  # Docker mount point
        self.release_dir = self.output_dir / "dist"
        self.pkg_build_dir = self.output_dir / "build"
        self.log_dir = self.output_dir / "log"

        # Log files
        self.deb_pkgs_file = self.log_dir / "deb_pkgs.txt"
        self.successful_pkgs_file = self.log_dir / "successful_pkgs.txt"
        self.failed_pkgs_file = self.log_dir / "failed_pkgs.txt"
        self.skipped_pkgs_file = self.log_dir / "skipped_pkgs.txt"

    def get_pkg_work_dir(self, pkg_name: str) -> Path:
        """Get working directory for a package."""
        return self.pkg_build_dir / pkg_name

    def get_pkg_config_dir(self, pkg_name: str) -> Path:
        """Get config directory for a package."""
        return self.config_dir / pkg_name
