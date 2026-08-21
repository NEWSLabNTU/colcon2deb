"""Behavior tests for debian-dir generation and deb-build skip logic.

Runs copy_or_create_debian_dir / build_single_package directly with a fake
rosdeb_bloom module, covering the cache-invalidation bugs:
- bloom output must not be cached into the user's overrides dir
- fingerprint mismatch must re-run bloom, never reuse stale metadata
- phase 7's fingerprint must not make phase 8 skip the .deb build
"""

from __future__ import annotations

import os
import shutil
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

HELPER_DIR = Path(__file__).parent.parent / "colcon2deb" / "helper"
if str(HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(HELPER_DIR))

# Install a fake rosdeb_bloom before importing the module under test.
# The real one is only pip-installed inside the build container.
_bloom_calls: list[str] = []


def _fake_generate_debian(
    package_path: Path, ros_distro: str, install_prefix: str, **_: Any
) -> SimpleNamespace:
    _bloom_calls.append(str(package_path))
    debian = Path(package_path) / "debian"
    debian.mkdir(parents=True, exist_ok=True)
    (debian / "control").write_text(f"generated for prefix {install_prefix}\n")
    return SimpleNamespace(success=True, error=None, debian_dir=str(debian))


_fake_pkg = types.ModuleType("rosdeb_bloom")
_fake_api = types.ModuleType("rosdeb_bloom.api")
_fake_api.generate_debian = _fake_generate_debian  # type: ignore[attr-defined]
_fake_logging = types.ModuleType("rosdeb_bloom.logging")
_fake_logging.disable_ANSI_colors = lambda: None  # type: ignore[attr-defined]
sys.modules.setdefault("rosdeb_bloom", _fake_pkg)
sys.modules.setdefault("rosdeb_bloom.api", _fake_api)
sys.modules.setdefault("rosdeb_bloom.logging", _fake_logging)

from colcon2deb.helper.build_deb import BuildStatus, build_single_package  # noqa: E402
from colcon2deb.helper.fingerprint import compute_fingerprint, write_fingerprint  # noqa: E402
from colcon2deb.helper.generate_debian_dir import (  # noqa: E402
    DebianDirStatus,
    copy_or_create_debian_dir,
)

FP_INPUTS = {
    "install_prefix": "/opt/ros/humble",
    "package_suffix": "",
    "ros_distro": "humble",
    "colcon2deb_version": "0.4.1",
    "docker_image_id": "sha256:abc123",
}


@pytest.fixture(autouse=True)
def reset_bloom_calls() -> None:
    _bloom_calls.clear()


@pytest.fixture
def env(tmp_path: Path) -> SimpleNamespace:
    pkg_dir = tmp_path / "ws" / "src" / "mypkg"
    pkg_dir.mkdir(parents=True)
    (pkg_dir / "package.xml").write_text("<package><name>my_pkg</name></package>")
    (pkg_dir / "main.cpp").write_text("int main() {}")
    config_dir = tmp_path / "overrides"
    config_dir.mkdir()
    pkg_build_dir = tmp_path / "packaging"
    pkg_build_dir.mkdir()
    logs = tmp_path / "logs"
    logs.mkdir()
    return SimpleNamespace(
        pkg_dir=pkg_dir,
        config_dir=config_dir,
        pkg_build_dir=pkg_build_dir,
        logs=logs,
        tmp=tmp_path,
    )


def _generate(env: SimpleNamespace, fp_inputs: dict[str, str] | None = None) -> Any:
    inputs = dict(fp_inputs if fp_inputs is not None else FP_INPUTS)
    return copy_or_create_debian_dir(
        pkg_name="my_pkg",
        pkg_dir=env.pkg_dir,
        config_dir=env.config_dir,
        pkg_build_dir=env.pkg_build_dir,
        script_dir=HELPER_DIR,
        ros_distro="humble",
        ros_install_prefix=inputs["install_prefix"],
        peer_packages=["my_pkg"],
        package_suffix=None,
        log_packages_dir=env.logs,
        fingerprint_inputs=inputs,
    )


class TestGenerateDebianDir:
    def test_bloom_generates_into_work_dir(self, env: SimpleNamespace) -> None:
        result = _generate(env)
        assert result.status == DebianDirStatus.SUCCESS
        assert (env.pkg_build_dir / "my_pkg" / "debian" / "control").is_file()
        assert len(_bloom_calls) == 1

    def test_bloom_does_not_write_into_overrides_dir(self, env: SimpleNamespace) -> None:
        """The user's debian-overrides dir is configuration input, not a cache."""
        _generate(env)
        assert list(env.config_dir.iterdir()) == []

    def test_fingerprint_match_skips_regeneration(self, env: SimpleNamespace) -> None:
        _generate(env)
        result = _generate(env)
        assert result.status == DebianDirStatus.SUCCESS
        assert len(_bloom_calls) == 1  # second call skipped

    def test_install_prefix_change_regenerates(self, env: SimpleNamespace) -> None:
        """Changed build config must re-run bloom, not reuse stale metadata."""
        _generate(env)
        changed = {**FP_INPUTS, "install_prefix": "/opt/autoware"}
        result = _generate(env, changed)
        assert result.status == DebianDirStatus.SUCCESS
        assert len(_bloom_calls) == 2
        control = (env.pkg_build_dir / "my_pkg" / "debian" / "control").read_text()
        assert "/opt/autoware" in control

    def test_source_change_regenerates(self, env: SimpleNamespace) -> None:
        _generate(env)
        (env.pkg_dir / "main.cpp").write_text("int main() { return 2; }")
        _generate(env)
        assert len(_bloom_calls) == 2

    @pytest.mark.skipif(shutil.which("rsync") is None, reason="rsync not available")
    def test_user_override_is_copied(self, env: SimpleNamespace) -> None:
        override_debian = env.config_dir / "my_pkg" / "debian"
        override_debian.mkdir(parents=True)
        (override_debian / "control").write_text("user override\n")
        result = _generate(env)
        assert result.status == DebianDirStatus.SUCCESS
        assert result.method == "copy"
        dst = env.pkg_build_dir / "my_pkg" / "debian" / "control"
        assert dst.read_text() == "user override\n"
        assert len(_bloom_calls) == 0


class TestBuildDebSkipLogic:
    def _fp_for(self, env: SimpleNamespace) -> dict[str, str]:
        return compute_fingerprint(
            pkg_name="my_pkg",
            pkg_dir=env.pkg_dir,
            overrides_dir=env.config_dir,
            **FP_INPUTS,
        )

    def _build(self, env: SimpleNamespace, check_dir: Path) -> Any:
        release_dir = env.tmp / "debs"
        release_dir.mkdir(exist_ok=True)
        return build_single_package(
            pkg_name="my_pkg",
            pkg_dir=env.pkg_dir,
            pkg_build_dir=env.pkg_build_dir,
            release_dir=release_dir,
            check_dir=check_dir,
            ros_distro="humble",
            ros_install_prefix="/opt/ros/humble",
            colcon_install_path=str(env.tmp / "install"),
            package_suffix=None,
            log_packages_dir=env.logs,
            fingerprint_inputs=dict(FP_INPUTS),
        )

    def test_phase7_fingerprint_does_not_skip_deb_build(
        self, env: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Phase 7 writing its fingerprint must not convince phase 8 that the
        .deb is up to date — this shipped stale .debs on every source change."""
        monkeypatch.setenv("config_dir", str(env.config_dir))
        check_dir = env.tmp / "debs-existing"
        check_dir.mkdir()
        (check_dir / "ros-humble-my-pkg_1.0.0-0jammy_amd64.deb").write_bytes(b"old deb")

        pkg_work_dir = env.pkg_build_dir / "my_pkg"
        pkg_work_dir.mkdir(parents=True)
        # Phase 7 stored its fingerprint (as after regeneration)
        write_fingerprint(pkg_work_dir / ".fingerprint.debian.json", self._fp_for(env))

        result = self._build(env, check_dir)
        assert result.status != BuildStatus.SKIPPED

    def test_deb_skip_when_own_fingerprint_matches(
        self, env: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setenv("config_dir", str(env.config_dir))
        check_dir = env.tmp / "debs-existing"
        check_dir.mkdir()
        (check_dir / "ros-humble-my-pkg_1.0.0-0jammy_amd64.deb").write_bytes(b"deb")

        pkg_work_dir = env.pkg_build_dir / "my_pkg"
        pkg_work_dir.mkdir(parents=True)
        write_fingerprint(pkg_work_dir / ".fingerprint.deb.json", self._fp_for(env))

        result = self._build(env, check_dir)
        assert result.status == BuildStatus.SKIPPED


class TestGetPackageList:
    def _fake_colcon(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
        """Create a workspace and a fake `colcon list` on PATH."""
        ws = tmp_path / "ws"
        ros_pkg = ws / "src" / "ros_pkg"
        ros_pkg.mkdir(parents=True)
        (ros_pkg / "package.xml").write_text("<package><name>ros_pkg</name></package>")
        plain_pkg = ws / "src" / "plain_pkg"
        plain_pkg.mkdir(parents=True)  # colcon package without package.xml

        bin_dir = tmp_path / "bin"
        bin_dir.mkdir()
        fake = bin_dir / "colcon"
        fake.write_text(
            "#!/bin/sh\n"
            'printf "ros_pkg\\tsrc/ros_pkg\\t(ros.ament_cmake)\\n"\n'
            'printf "plain_pkg\\tsrc/plain_pkg\\t(cmake)\\n"\n'
        )
        fake.chmod(0o755)
        monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ['PATH']}")
        return ws

    def test_build_deb_skips_non_ros_packages(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Packages without package.xml are skipped by phase 7 on purpose, so
        phase 8 must not try (and fail) to build them."""
        from colcon2deb.helper.build_deb import get_package_list as deb_list

        ws = self._fake_colcon(tmp_path, monkeypatch)
        names = [name for name, _ in deb_list(ws)]
        assert names == ["ros_pkg"]

    def test_generate_debian_skips_non_ros_packages(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        from colcon2deb.helper.generate_debian_dir import get_package_list as gen_list

        ws = self._fake_colcon(tmp_path, monkeypatch)
        names = [name for name, _ in gen_list(ws)]
        assert names == ["ros_pkg"]


class TestStaleDebianArtifacts:
    def test_bloom_precleans_stale_debian_dir(self, env: SimpleNamespace) -> None:
        """A previous phase-8 run leaves debian/.debhelper (with dangling
        symlinks) inside the package source copy; generation must clean it
        instead of crashing while copying the generated debian dir."""
        stale = env.pkg_dir / "debian" / ".debhelper" / "pkgroot" / "usr" / "share" / "doc"
        stale.mkdir(parents=True)
        (stale / "dangling").symlink_to("/nonexistent/target")
        (env.pkg_dir / "debian" / "control").write_text("stale control\n")

        result = _generate(env)
        assert result.status == DebianDirStatus.SUCCESS
        control = (env.pkg_build_dir / "my_pkg" / "debian" / "control").read_text()
        assert "stale" not in control
        assert not (env.pkg_build_dir / "my_pkg" / "debian" / ".debhelper").exists()
