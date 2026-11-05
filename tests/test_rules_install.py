"""The ament_cmake debian/rules install step must handle a package that installs nothing.

Autoware's CUDA packages (cuda_utils, tensorrt_common, ...) call return() before
any install() when CUDA is absent, the normal case in a GPU-free build. CMake
then generates no `install` make target, and the template's
`make -C .obj-* install` failed every such package with "No rule to make target
'install'".

These tests run the template's real override_dh_auto_install recipe, lifted out
of rules.em, against two configured CMake projects.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

RULES_EM = (
    Path(__file__).parent.parent
    / "colcon2deb/rosdeb-bloom/rosdeb_bloom/generators/debian/templates/ament_cmake/rules.em"
)
HOST_TYPE = "x86_64-linux-gnu"
PACKAGE = "ros-humble-demo"

pytestmark = pytest.mark.skipif(
    shutil.which("cmake") is None or shutil.which("make") is None,
    reason="needs cmake and make",
)


def _install_recipe() -> str:
    """The override_dh_auto_install target from rules.em, template fields filled in."""
    text = RULES_EM.read_text()
    m = re.search(r"^override_dh_auto_install:\n((?:\t.*\n?)+)", text, re.M)
    assert m, "override_dh_auto_install not found in rules.em"
    return "override_dh_auto_install:\n" + m.group(1).replace("@(Package)", PACKAGE)


def _run_install(project: Path, cmakelists: str) -> subprocess.CompletedProcess[str]:
    project.mkdir()
    (project / "CMakeLists.txt").write_text(cmakelists)
    (project / "payload.txt").write_text("hello\n")
    obj = project / f".obj-{HOST_TYPE}"
    subprocess.run(
        ["cmake", "-S", str(project), "-B", str(obj), "-DCMAKE_INSTALL_PREFIX=/opt/demo"],
        check=True,
        capture_output=True,
    )
    rules = project / "rules.mk"
    rules.write_text(
        "SHELL := /bin/bash\n"
        f"DEB_HOST_GNU_TYPE := {HOST_TYPE}\n"
        "SETUP_SCRIPT := /nonexistent/setup.bash\n"
        "PATH_INSTALL := /nonexistent\n"
        "PATH_ROS := /nonexistent\n" + _install_recipe()
    )
    return subprocess.run(
        ["make", "-f", "rules.mk", "override_dh_auto_install"],
        cwd=project,
        capture_output=True,
        text=True,
    )


def test_package_without_install_rules_installs_nothing(tmp_path: Path) -> None:
    """CUDA-less shape: return() before any install(); must succeed, empty package dir."""
    result = _run_install(
        tmp_path / "skipped",
        "cmake_minimum_required(VERSION 3.14)\nproject(demo NONE)\nreturn()\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    staged = tmp_path / "skipped" / "debian" / PACKAGE
    assert not staged.exists() or not any(p.is_file() for p in staged.rglob("*"))


def test_package_with_install_rules_still_installs(tmp_path: Path) -> None:
    """The normal case must keep installing into DESTDIR."""
    result = _run_install(
        tmp_path / "normal",
        "cmake_minimum_required(VERSION 3.14)\nproject(demo NONE)\n"
        "install(FILES payload.txt DESTINATION share/demo)\n",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    installed = tmp_path / "normal" / "debian" / PACKAGE / "opt/demo/share/demo/payload.txt"
    assert installed.read_text() == "hello\n"
