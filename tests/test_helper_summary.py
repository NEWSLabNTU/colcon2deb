"""Tests for the container-side build summary and exit-code logic."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

HELPER_DIR = Path(__file__).parent.parent / "colcon2deb" / "helper"
if str(HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(HELPER_DIR))

from colcon2deb.helper.main import print_summary, write_summary_file  # noqa: E402


@pytest.fixture
def reports_dir(tmp_path: Path) -> Path:
    d = tmp_path / "reports"
    d.mkdir()
    return d


def _summary_args(reports_dir: Path, tmp_path: Path) -> dict[str, Path]:
    release_dir = tmp_path / "debs"
    release_dir.mkdir(exist_ok=True)
    return {
        "deb_pkgs_file": reports_dir / "packages.txt",
        "successful_pkgs_file": reports_dir / "successful.txt",
        "failed_pkgs_file": reports_dir / "failed.txt",
        "skipped_pkgs_file": reports_dir / "skipped.txt",
        "release_dir": release_dir,
        "log_reports_dir": reports_dir,
    }


class TestPrintSummaryExitCode:
    def test_phase_failure_returns_nonzero(self, reports_dir: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        """A failed phase must produce a non-zero exit code even when no
        per-package failures were recorded (e.g. colcon build failed in
        phase 4, so phases 5-8 never ran and failed.txt is empty)."""
        rc = print_summary(
            **_summary_args(reports_dir, tmp_path),
            last_failing_phase="Phase 4: Compiling packages",
        )
        assert rc != 0

    def test_phase_failure_does_not_claim_completion(self, reports_dir: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        print_summary(
            **_summary_args(reports_dir, tmp_path),
            last_failing_phase="Phase 4: Compiling packages",
        )
        out = capsys.readouterr().out
        assert "Build completed" not in out
        assert "Phase 4" in out

    def test_package_failures_return_nonzero(self, reports_dir: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        args = _summary_args(reports_dir, tmp_path)
        args["failed_pkgs_file"].write_text("pkg_a\npkg_b\n")
        rc = print_summary(**args, last_failing_phase=None)
        assert rc != 0

    def test_success_returns_zero(self, reports_dir: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]):
        args = _summary_args(reports_dir, tmp_path)
        args["successful_pkgs_file"].write_text("pkg_a\n")
        args["deb_pkgs_file"].write_text("pkg_a\n")
        rc = print_summary(**args, last_failing_phase=None)
        assert rc == 0


class TestSummaryFileStatus:
    def test_status_failed_when_packages_failed(self, reports_dir: Path):
        """summary.txt must not say SUCCESS when packages failed."""
        write_summary_file(
            log_reports_dir=reports_dir,
            last_failing_phase=None,
            successful_pkgs=1,
            failed_pkgs=3,
            skipped_pkgs=0,
            total_pkgs=4,
            output_debs=1,
        )
        content = (reports_dir / "summary.txt").read_text()
        assert "Status: FAILED" in content

    def test_status_failed_on_phase_failure(self, reports_dir: Path):
        write_summary_file(
            log_reports_dir=reports_dir,
            last_failing_phase="Phase 4: Compiling packages",
            successful_pkgs=0,
            failed_pkgs=0,
            skipped_pkgs=0,
            total_pkgs=0,
            output_debs=0,
        )
        content = (reports_dir / "summary.txt").read_text()
        assert "Status: FAILED" in content

    def test_status_success_when_clean(self, reports_dir: Path):
        write_summary_file(
            log_reports_dir=reports_dir,
            last_failing_phase=None,
            successful_pkgs=2,
            failed_pkgs=0,
            skipped_pkgs=0,
            total_pkgs=2,
            output_debs=2,
        )
        content = (reports_dir / "summary.txt").read_text()
        assert "Status: SUCCESS" in content
