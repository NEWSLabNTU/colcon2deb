"""The ament_cmake debian/rules must point at the build's own ROS distro.

rules.em exports AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH from ROS_BASE_PATH,
replacing what the build environment had. With /opt/ros/humble fixed in the
template, a Jazzy build found no ROS package at all in debian/rules, and every
message package failed to configure with "No 'rosidl_typesupport_c' found".
"""

from __future__ import annotations

import re
from pathlib import Path

RULES_EM = (
    Path(__file__).parent.parent
    / "colcon2deb/rosdeb-bloom/rosdeb_bloom/generators/debian/templates/ament_cmake/rules.em"
)
API = Path(__file__).parent.parent / "colcon2deb/rosdeb-bloom/rosdeb_bloom/api.py"


def _code_lines(text: str) -> list[str]:
    return [line for line in text.splitlines() if not line.lstrip().startswith("#")]


def test_ros_base_path_comes_from_the_distro() -> None:
    lines = _code_lines(RULES_EM.read_text())
    assert "ROS_BASE_PATH ?= /opt/ros/@(Rosdistro)" in lines


def test_no_distro_name_is_hardcoded() -> None:
    for line in _code_lines(RULES_EM.read_text()):
        assert not re.search(r"/opt/ros/(humble|iron|jazzy|kilted|rolling)\b", line), line


def test_api_provides_rosdistro_to_templates() -> None:
    assert "subs['Rosdistro'] = ros_distro" in API.read_text()
