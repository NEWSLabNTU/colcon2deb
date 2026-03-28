"""Tests for debian control file structure and conventions.

Validates the generated debian/control files against real Autoware 1.5.0
output patterns for all package types:
  - ament_cmake simple (autoware_lint_common)
  - ament_cmake complex with many deps (autoware_ndt_scan_matcher)
  - rosidl message package (autoware_adapi_v1_msgs)
  - ament_python package (autoware_carla_interface)
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


def parse_control(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    """Parse a debian/control file into source and binary package paragraphs.

    Returns (source_paragraph, [binary_paragraphs]).
    """
    all_paragraphs: list[dict[str, str]] = []
    current: dict[str, str] = {}
    last_key = ""

    for line in path.read_text().splitlines():
        if not line.strip():
            if current:
                all_paragraphs.append(current)
                current = {}
                last_key = ""
            continue
        if line.startswith(" ") or line.startswith("\t"):
            if last_key:
                current[last_key] += " " + line.strip()
        elif ":" in line:
            field, _, value = line.partition(":")
            field = field.strip()
            value = value.strip()
            current[field] = value
            last_key = field

    if current:
        all_paragraphs.append(current)

    source = all_paragraphs[0] if all_paragraphs else {}
    binaries = all_paragraphs[1:]
    return source, binaries


class TestControlFileStructure:
    """Test debian/control file format conventions."""

    def test_ament_cmake_simple_source_fields(
        self, debian_controls_dir: Path
    ) -> None:
        src, _ = parse_control(
            debian_controls_dir / "ament_cmake_simple_with_suffix.control"
        )
        assert src["Source"] == "ros-humble-autoware-lint-common-1-5-0"
        assert src["Section"] == "misc"
        assert src["Priority"] == "optional"
        assert "debhelper" in src["Build-Depends"]
        assert src["Standards-Version"] == "3.9.2"
        assert "Ryohsuke Mitsudome" in src["Maintainer"]

    def test_package_suffix_in_source_and_binary(
        self, debian_controls_dir: Path
    ) -> None:
        """When package_suffix is set, both Source and Package include it."""
        src, binaries = parse_control(
            debian_controls_dir / "ament_cmake_simple_with_suffix.control"
        )
        assert src["Source"] == "ros-humble-autoware-lint-common-1-5-0"
        assert len(binaries) == 1
        assert binaries[0]["Package"] == "ros-humble-autoware-lint-common-1-5-0"

    def test_workspace_deps_get_suffix(self, debian_controls_dir: Path) -> None:
        """Workspace-internal dependencies get the suffix appended."""
        src, _ = parse_control(
            debian_controls_dir / "ament_cmake_complex_with_suffix.control"
        )
        build_deps = src["Build-Depends"]
        # Workspace packages should have -1-5-0 suffix
        assert "ros-humble-autoware-cmake-1-5-0" in build_deps
        assert "ros-humble-autoware-internal-debug-msgs-1-5-0" in build_deps
        assert "ros-humble-autoware-localization-util-1-5-0" in build_deps

    def test_system_deps_no_suffix(self, debian_controls_dir: Path) -> None:
        """System/ROS dependencies do NOT get the suffix."""
        src, _ = parse_control(
            debian_controls_dir / "ament_cmake_complex_with_suffix.control"
        )
        build_deps = src["Build-Depends"]
        # System ROS packages should NOT have -1-5-0 suffix
        assert "ros-humble-rclcpp," in build_deps or "ros-humble-rclcpp" in build_deps
        assert "ros-humble-rclcpp-1-5-0" not in build_deps

    def test_nocheck_on_test_depends(self, debian_controls_dir: Path) -> None:
        """Test dependencies should have <!nocheck> annotation."""
        src, _ = parse_control(
            debian_controls_dir / "ament_cmake_complex_with_suffix.control"
        )
        build_deps = src["Build-Depends"]
        assert "ros-humble-ament-cmake-cppcheck <!nocheck>" in build_deps
        assert "ros-humble-ament-lint-auto <!nocheck>" in build_deps

    def test_complex_package_many_dependencies(
        self, debian_controls_dir: Path
    ) -> None:
        """autoware_ndt_scan_matcher has 40+ dependencies (PCL, TF, etc.)."""
        src, _ = parse_control(
            debian_controls_dir / "ament_cmake_complex_with_suffix.control"
        )
        build_deps = [d.strip() for d in src["Build-Depends"].split(",")]
        assert len(build_deps) > 30
        pcl_deps = [d for d in build_deps if "libpcl" in d]
        assert len(pcl_deps) > 10

    def test_rosidl_msgs_package(self, debian_controls_dir: Path) -> None:
        """Message packages depend on rosidl_default_generators (build) and _runtime (exec)."""
        src, binaries = parse_control(
            debian_controls_dir / "rosidl_msgs_with_suffix.control"
        )
        assert "ros-humble-rosidl-default-generators" in src["Build-Depends"]
        assert len(binaries) == 1
        assert "ros-humble-rosidl-default-runtime" in binaries[0]["Depends"]

    def test_ament_python_package(self, debian_controls_dir: Path) -> None:
        """Python packages have python3-all, dh-python, python3-setuptools in Build-Depends."""
        src, _ = parse_control(
            debian_controls_dir / "ament_python_with_suffix.control"
        )
        build_deps = src["Build-Depends"]
        assert "python3-all" in build_deps
        assert "python3-setuptools" in build_deps
        assert "dh-python" in build_deps

    def test_ament_python_depends_on_python3(
        self, debian_controls_dir: Path
    ) -> None:
        """Python binary packages use ${python3:Depends} instead of ${shlibs:Depends}."""
        _, binaries = parse_control(
            debian_controls_dir / "ament_python_with_suffix.control"
        )
        assert len(binaries) == 1
        assert "${python3:Depends}" in binaries[0]["Depends"]


class TestPackageNameConversion:
    """Test the underscore-to-dash conversion for debian package names."""

    @pytest.mark.parametrize(
        "ros_name,expected_deb_name",
        [
            ("autoware_lint_common", "ros-humble-autoware-lint-common"),
            ("autoware_ndt_scan_matcher", "ros-humble-autoware-ndt-scan-matcher"),
            ("autoware_adapi_v1_msgs", "ros-humble-autoware-adapi-v1-msgs"),
            ("agnocastlib", "ros-humble-agnocastlib"),
            ("tf2_ros", "ros-humble-tf2-ros"),
            # Edge case: single-word package name
            ("rclcpp", "ros-humble-rclcpp"),
        ],
    )
    def test_name_conversion(self, ros_name: str, expected_deb_name: str) -> None:
        """ROS package names (underscores) convert to deb names (dashes)."""
        deb_name = "ros-humble-" + ros_name.replace("_", "-")
        assert deb_name == expected_deb_name

    @pytest.mark.parametrize(
        "ros_name,suffix,expected",
        [
            ("autoware_lint_common", "1-5-0", "ros-humble-autoware-lint-common-1-5-0"),
            ("autoware_lint_common", None, "ros-humble-autoware-lint-common"),
            ("agnocastlib", "1-5-0", "ros-humble-agnocastlib-1-5-0"),
        ],
    )
    def test_name_with_suffix(
        self, ros_name: str, suffix: str | None, expected: str
    ) -> None:
        deb_name = "ros-humble-" + ros_name.replace("_", "-")
        if suffix:
            deb_name += f"-{suffix}"
        assert deb_name == expected
