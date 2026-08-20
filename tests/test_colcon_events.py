"""Tests for the colcon events.log parser and per-package build gate.

The pipelined build starts a package's .deb build as soon as colcon
reports that package finished (JobEnded rc=0 in log/build_<ts>/events.log),
while colcon keeps building the rest of the workspace.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

HELPER_DIR = Path(__file__).parent.parent / "colcon2deb" / "helper"
if str(HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(HELPER_DIR))

from colcon2deb.helper.colcon_events import (  # noqa: E402
    ColconGate,
    find_events_log,
    parse_job_ended,
    snapshot_build_dirs,
)


class TestParseJobEnded:
    def test_parses_success_line(self) -> None:
        line = "[0.101057] (test_cpp_pkg) JobEnded: {'identifier': 'test_cpp_pkg', 'rc': 0}"
        assert parse_job_ended(line) == ("test_cpp_pkg", 0)

    def test_parses_failure_line(self) -> None:
        line = "[5.2] (bad_pkg) JobEnded: {'identifier': 'bad_pkg', 'rc': 2}"
        assert parse_job_ended(line) == ("bad_pkg", 2)

    def test_ignores_other_events(self) -> None:
        assert parse_job_ended("[0.0] (pkg) JobStarted: {'identifier': 'pkg'}") is None
        assert parse_job_ended("[0.0] (-) TimerEvent: {}") is None
        assert parse_job_ended("") is None

    def test_ignores_malformed_job_ended(self) -> None:
        assert parse_job_ended("[0.0] (pkg) JobEnded: garbage") is None


class TestFindEventsLog:
    def test_ignores_dirs_from_previous_runs(self, tmp_path: Path) -> None:
        """Snapshot-based exclusion: a previous run's build dir must never
        be mistaken for the current run's (mtimes are too coarse)."""
        old = tmp_path / "build_2026-01-01_00-00-00"
        old.mkdir()
        (old / "events.log").write_text("")
        before = snapshot_build_dirs(tmp_path)

        assert find_events_log(tmp_path, exclude=before) is None

        new = tmp_path / "build_2026-01-02_00-00-00"
        new.mkdir()
        (new / "events.log").write_text("")
        found = find_events_log(tmp_path, exclude=before)
        assert found == new / "events.log"

    def test_missing_log_base(self, tmp_path: Path) -> None:
        assert find_events_log(tmp_path / "nope", exclude=set()) is None


class TestColconGate:
    def _gate(self, tmp_path: Path) -> tuple[ColconGate, Path, Path]:
        log_base = tmp_path / "log"
        log_base.mkdir()
        status = tmp_path / ".colcon.status"
        gate = ColconGate(log_base=log_base, status_file=status, poll_interval=0.02)
        return gate, log_base, status

    def test_wait_releases_on_job_ended(self, tmp_path: Path) -> None:
        gate, log_base, _ = self._gate(tmp_path)
        build_dir = log_base / "build_x"
        build_dir.mkdir()
        events = build_dir / "events.log"
        events.write_text("")
        gate.start()
        try:
            result: list[bool] = []
            t = threading.Thread(target=lambda: result.append(gate.wait("pkg_a", timeout=5)))
            t.start()
            time.sleep(0.1)
            assert not result  # still blocked
            with open(events, "a") as f:
                f.write("[1.0] (pkg_a) JobEnded: {'identifier': 'pkg_a', 'rc': 0}\n")
            t.join(timeout=5)
            assert result == [True]
        finally:
            gate.stop()

    def test_wait_fails_for_failed_package(self, tmp_path: Path) -> None:
        gate, log_base, _ = self._gate(tmp_path)
        build_dir = log_base / "build_x"
        build_dir.mkdir()
        (build_dir / "events.log").write_text(
            "[1.0] (pkg_a) JobEnded: {'identifier': 'pkg_a', 'rc': 1}\n"
        )
        gate.start()
        try:
            assert gate.wait("pkg_a", timeout=5) is False
        finally:
            gate.stop()

    def test_colcon_success_releases_everything(self, tmp_path: Path) -> None:
        """Status file with 0 releases packages with no JobEnded line
        (e.g. cached packages colcon logged in an unexpected format)."""
        gate, _, status = self._gate(tmp_path)
        gate.start()
        try:
            status.write_text("0")
            assert gate.wait("never_logged_pkg", timeout=5) is True
        finally:
            gate.stop()

    def test_colcon_failure_blocks_unfinished_packages(self, tmp_path: Path) -> None:
        gate, log_base, status = self._gate(tmp_path)
        build_dir = log_base / "build_x"
        build_dir.mkdir()
        (build_dir / "events.log").write_text(
            "[1.0] (done_pkg) JobEnded: {'identifier': 'done_pkg', 'rc': 0}\n"
        )
        gate.start()
        try:
            status.write_text("1")
            # Finished before the failure: still buildable
            assert gate.wait("done_pkg", timeout=5) is True
            # Never finished: not buildable
            assert gate.wait("unfinished_pkg", timeout=5) is False
        finally:
            gate.stop()

    def test_disabled_gate_releases_immediately(self, tmp_path: Path) -> None:
        gate = ColconGate.disabled()
        assert gate.wait("anything", timeout=1) is True


class TestBuildDebGating:
    """build_single_package must consult the colcon gate before building."""

    def _run(self, tmp_path: Path, gate: ColconGate) -> object:
        from colcon2deb.helper.build_deb import build_single_package

        pkg_dir = tmp_path / "ws" / "src" / "mypkg"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "package.xml").write_text("<package><name>my_pkg</name></package>")
        for d in ["packaging", "debs", "logs", "overrides"]:
            (tmp_path / d).mkdir(exist_ok=True)
        return build_single_package(
            pkg_name="my_pkg",
            pkg_dir=pkg_dir,
            pkg_build_dir=tmp_path / "packaging",
            release_dir=tmp_path / "debs",
            check_dir=tmp_path / "debs",
            ros_distro="humble",
            ros_install_prefix="/opt/ros/humble",
            colcon_install_path=str(tmp_path / "install"),
            package_suffix=None,
            log_packages_dir=tmp_path / "logs",
            fingerprint_inputs=None,
            colcon_gate=gate,
        )

    def test_refused_package_fails_with_colcon_error(self, tmp_path: Path) -> None:
        from colcon2deb.helper.build_deb import BuildStatus

        gate = ColconGate(
            log_base=tmp_path / "log", status_file=tmp_path / "st", poll_interval=0.02
        )
        gate.start()
        try:
            (tmp_path / "st").write_text("1")  # colcon failed; pkg never ended
            result = self._run(tmp_path, gate)
            assert result.status == BuildStatus.FAILED
            assert "colcon" in (result.error or "").lower()
        finally:
            gate.stop()

    def test_released_package_proceeds_past_gate(self, tmp_path: Path) -> None:
        from colcon2deb.helper.build_deb import BuildStatus

        gate = ColconGate.disabled()
        result = self._run(tmp_path, gate)
        # Proceeds to the debian-dir check (which fails — no debian/ staged),
        # proving the gate released rather than refused.
        assert result.status == BuildStatus.FAILED
        assert "debian directory not found" in (result.error or "")
