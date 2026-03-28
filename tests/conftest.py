"""Shared pytest fixtures for colcon2deb tests."""

from __future__ import annotations

from pathlib import Path

import pytest

FIXTURES_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def fixtures_dir() -> Path:
    return FIXTURES_DIR


@pytest.fixture
def package_xmls_dir() -> Path:
    return FIXTURES_DIR / "package_xmls"


@pytest.fixture
def configs_dir() -> Path:
    return FIXTURES_DIR / "configs"


@pytest.fixture
def events_dir() -> Path:
    return FIXTURES_DIR / "events"


@pytest.fixture
def debian_controls_dir() -> Path:
    return FIXTURES_DIR / "debian_controls"


@pytest.fixture
def tmp_output_dir(tmp_path: Path) -> Path:
    """Create a temporary output directory mimicking the colcon2deb build structure."""
    output = tmp_path / "build"
    output.mkdir()
    (output / "debs").mkdir()
    (output / "packaging").mkdir()
    (output / "workspace" / "src").mkdir(parents=True)
    logs = output / "logs" / "2026-03-28_02-09-16"
    (logs / "logs").mkdir(parents=True)
    (logs / "reports").mkdir()
    (logs / "scripts").mkdir()
    # Create 'latest' symlink
    (output / "logs" / "latest").symlink_to("2026-03-28_02-09-16")
    return output


@pytest.fixture
def successful_build_output(tmp_output_dir: Path) -> Path:
    """Create a temporary output directory with a successful 453-package build."""
    reports = tmp_output_dir / "logs" / "latest" / "reports"

    # Realistic subset of Autoware package names (from real 1.5.0 build)
    packages = [
        "agnocast_ioctl_wrapper",
        "agnocast_sample_interfaces",
        "agnocast_sample_application",
        "agnocast_e2e_test",
        "autoware_lint_common",
        "autoware_adapi_specs",
        "autoware_adapi_v1_msgs",
        "autoware_ndt_scan_matcher",
        "autoware_carla_interface",
        "autoware_mission_planner",
    ]

    # packages.txt: deb name=version format (no suffix in this field)
    packages_lines = [
        "ros-humble-agnocast-ioctl-wrapper=2.1.2-0jammy",
        "ros-humble-agnocast-sample-interfaces=2.1.2-0jammy",
        "ros-humble-agnocast-sample-application=2.1.2-0jammy",
        "ros-humble-agnocast-e2e-test=2.1.2-0jammy",
        "ros-humble-autoware-lint-common=1.1.0-0jammy",
        "ros-humble-autoware-adapi-specs=1.5.0-0jammy",
        "ros-humble-autoware-adapi-v1-msgs=1.9.1-0jammy",
        "ros-humble-autoware-ndt-scan-matcher=1.5.0-0jammy",
        "ros-humble-autoware-carla-interface=0.48.0-0jammy",
        "ros-humble-autoware-mission-planner=0.48.0-0jammy",
    ]

    (reports / "successful.txt").write_text("\n".join(packages) + "\n")
    (reports / "failed.txt").write_text("")
    (reports / "skipped.txt").write_text("")
    (reports / "packages.txt").write_text("\n".join(packages_lines) + "\n")

    # Create fake .deb files in debs/ directory
    debs_dir = tmp_output_dir / "debs"
    deb_specs = [
        ("agnocast-ioctl-wrapper-1-5-0", "2.1.2"),
        ("agnocast-sample-interfaces-1-5-0", "2.1.2"),
        ("agnocast-sample-application-1-5-0", "2.1.2"),
        ("agnocast-e2e-test-1-5-0", "2.1.2"),
        ("autoware-lint-common-1-5-0", "1.1.0"),
        ("autoware-adapi-specs-1-5-0", "1.5.0"),
        ("autoware-adapi-v1-msgs-1-5-0", "1.9.1"),
        ("autoware-ndt-scan-matcher-1-5-0", "1.5.0"),
        ("autoware-carla-interface-1-5-0", "0.48.0"),
        ("autoware-mission-planner-1-5-0", "0.48.0"),
    ]
    for pkg_name, ver in deb_specs:
        (debs_dir / f"ros-humble-{pkg_name}_{ver}-0jammy_amd64.deb").write_bytes(b"fake")
        (debs_dir / f"ros-humble-{pkg_name}-dbgsym_{ver}-0jammy_amd64.ddeb").write_bytes(b"fake")

    return tmp_output_dir


@pytest.fixture
def partial_failure_build_output(tmp_output_dir: Path) -> Path:
    """Create a temporary output directory with some package failures."""
    reports = tmp_output_dir / "logs" / "latest" / "reports"

    successful = [
        "autoware_lint_common",
        "autoware_adapi_v1_msgs",
        "autoware_carla_interface",
    ]
    failed = [
        "autoware_ndt_scan_matcher",
        "autoware_mission_planner",
    ]
    skipped = [
        "agnocast_e2e_test",
    ]

    all_pkgs_lines = [
        "ros-humble-autoware-lint-common=1.1.0-0jammy",
        "ros-humble-autoware-adapi-v1-msgs=1.9.1-0jammy",
        "ros-humble-autoware-carla-interface=0.48.0-0jammy",
        "ros-humble-autoware-ndt-scan-matcher=1.5.0-0jammy",
        "ros-humble-autoware-mission-planner=0.48.0-0jammy",
        "ros-humble-agnocast-e2e-test=2.1.2-0jammy",
    ]

    (reports / "successful.txt").write_text("\n".join(successful) + "\n")
    (reports / "failed.txt").write_text("\n".join(failed) + "\n")
    (reports / "skipped.txt").write_text("\n".join(skipped) + "\n")
    (reports / "packages.txt").write_text("\n".join(all_pkgs_lines) + "\n")

    # Only create debs for successful + skipped packages
    debs_dir = tmp_output_dir / "debs"
    for pkg_name, ver in [
        ("autoware-lint-common-1-5-0", "1.1.0"),
        ("autoware-adapi-v1-msgs-1-5-0", "1.9.1"),
        ("autoware-carla-interface-1-5-0", "0.48.0"),
        ("agnocast-e2e-test-1-5-0", "2.1.2"),
    ]:
        (debs_dir / f"ros-humble-{pkg_name}_{ver}-0jammy_amd64.deb").write_bytes(b"fake")

    return tmp_output_dir
