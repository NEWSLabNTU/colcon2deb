"""Test that all Python modules import correctly."""

import pytest
from pathlib import Path


def test_import_autoware_debian_packager():
    """Test that the main package imports."""
    import autoware_debian_packager
    assert autoware_debian_packager.__version__ == "0.2.0"


def test_import_config():
    """Test that config module imports and BuildConfig is available."""
    from autoware_debian_packager.config import BuildConfig

    # Test creating a BuildConfig
    config = BuildConfig(
        workspace_dir=Path("/tmp/workspace"),
        output_dir=Path("/tmp/output"),
        ros_distro="humble",
    )

    assert config.workspace_dir == Path("/tmp/workspace")
    assert config.output_dir == Path("/tmp/output")
    assert config.ros_distro == "humble"
    assert config.ros_install_prefix == "/opt/ros/humble"
    assert config.colcon_work_dir == Path("/tmp/output/sources")
    assert config.release_dir == Path("/tmp/output/dist")


def test_import_utils():
    """Test that utils module imports."""
    from autoware_debian_packager.utils import logger, print_phase, run_command
    assert logger is not None
    assert callable(print_phase)
    assert callable(run_command)


def test_import_prepare():
    """Test that prepare module imports."""
    from autoware_debian_packager.prepare import setup_directories, copy_workspace
    assert callable(setup_directories)
    assert callable(copy_workspace)


def test_import_dependencies():
    """Test that dependencies module imports."""
    from autoware_debian_packager.dependencies import (
        generate_rosdep_commands,
        install_dependencies,
    )
    assert callable(generate_rosdep_commands)
    assert callable(install_dependencies)


def test_import_compiler():
    """Test that compiler module imports."""
    from autoware_debian_packager.compiler import build_workspace, source_install_setup
    assert callable(build_workspace)
    assert callable(source_install_setup)


def test_import_debian():
    """Test that debian module imports."""
    from autoware_debian_packager.debian import (
        create_rosdep_list,
        get_package_list,
        generate_debian_metadata,
    )
    assert callable(create_rosdep_list)
    assert callable(get_package_list)
    assert callable(generate_debian_metadata)


def test_import_packager():
    """Test that packager module imports."""
    from autoware_debian_packager.packager import (
        build_single_package,
        build_packages_parallel,
        print_build_summary,
    )
    assert callable(build_single_package)
    assert callable(build_packages_parallel)
    assert callable(print_build_summary)


def test_import_main():
    """Test that main module imports."""
    from autoware_debian_packager.main import main
    assert callable(main)


def test_config_with_custom_install_prefix():
    """Test BuildConfig with custom installation prefix."""
    from autoware_debian_packager.config import BuildConfig

    config = BuildConfig(
        workspace_dir=Path("/tmp/workspace"),
        output_dir=Path("/tmp/output"),
        ros_distro="humble",
        ros_install_prefix="/opt/autoware/2025.02",
    )

    assert config.ros_install_prefix == "/opt/autoware/2025.02"


def test_config_skip_flags():
    """Test BuildConfig with skip flags."""
    from autoware_debian_packager.config import BuildConfig

    config = BuildConfig(
        workspace_dir=Path("/tmp/workspace"),
        output_dir=Path("/tmp/output"),
        skip_rosdep_install=True,
        skip_copy_src=True,
        skip_gen_rosdep_list=True,
        skip_colcon_build=True,
    )

    assert config.skip_rosdep_install is True
    assert config.skip_copy_src is True
    assert config.skip_gen_rosdep_list is True
    assert config.skip_colcon_build is True
