"""colcon2deb - Build Debian packages from colcon workspaces in Docker containers."""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("colcon2deb")
except PackageNotFoundError:
    # Package not installed, fall back to reading pyproject.toml
    __version__ = "0.0.0-dev"

from .main import main

__all__ = ["main", "__version__"]
