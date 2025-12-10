"""Test that Python modules import correctly."""

import pytest
import re


def test_import_colcon2deb():
    """Test that the main package imports and has a version."""
    import colcon2deb
    # Check version exists and is a valid semver-like format or dev version
    assert colcon2deb.__version__ is not None
    assert re.match(r'^\d+\.\d+\.\d+(-\w+)?$', colcon2deb.__version__)


def test_import_main():
    """Test that main module imports."""
    from colcon2deb.main import main
    assert callable(main)


def test_version_matches_pyproject():
    """Test that runtime version matches pyproject.toml."""
    import colcon2deb
    from pathlib import Path

    pyproject_path = Path(__file__).parent.parent / "pyproject.toml"
    pyproject_content = pyproject_path.read_text()

    # Extract version from pyproject.toml
    for line in pyproject_content.split('\n'):
        if line.startswith('version = '):
            pyproject_version = line.split('"')[1]
            break
    else:
        pytest.fail("Could not find version in pyproject.toml")

    assert colcon2deb.__version__ == pyproject_version
