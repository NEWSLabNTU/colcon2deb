"""colcon2deb - Build Debian packages from colcon workspaces in Docker containers."""

__version__ = "0.2.0"

from .main import main

__all__ = ["main", "__version__"]
