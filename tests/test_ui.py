"""Tests for the Rich-based build UI.

Tests phase tracking, package sub-items, log tailing, and rendering
using patterns from real Autoware 1.5.0 builds.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon2deb.ui import BuildUI, PhaseStatus, SimpleBuildUI


class TestBuildUIPhases:
    """Test BuildUI phase lifecycle."""

    def test_add_phases(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase1", "Preparing working directories")
        ui.add_phase("phase2", "Copying source files")
        assert len(ui.phases) == 2
        assert len(ui.phase_order) == 2
        assert ui.phase_order == ["phase1", "phase2"]

    def test_phase_starts_running(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase1", "Preparing")
        ui.start_phase("phase1")
        assert ui.phases["phase1"].status == PhaseStatus.RUNNING
        assert ui.phases["phase1"].start_time is not None
        assert ui.current_phase == "phase1"

    def test_phase_completes_successfully(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase1", "Preparing")
        ui.start_phase("phase1")
        ui.complete_phase("phase1", success=True)
        assert ui.phases["phase1"].status == PhaseStatus.COMPLETED
        assert ui.phases["phase1"].end_time is not None
        assert ui.current_phase is None

    def test_phase_fails(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase4", "Compiling packages")
        ui.start_phase("phase4")
        ui.complete_phase("phase4", success=False)
        assert ui.phases["phase4"].status == PhaseStatus.FAILED

    def test_phase_skipped(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase7", "Generating Debian metadata")
        ui.skip_phase("phase7")
        assert ui.phases["phase7"].status == PhaseStatus.SKIPPED

    def test_eight_phase_autoware_lifecycle(self) -> None:
        """Simulate the full 8-phase Autoware build lifecycle."""
        ui = BuildUI()
        phase_names = [
            ("phase1", "Phase 1: Preparing working directories"),
            ("phase2", "Phase 2: Copying source files"),
            ("phase3", "Phase 3: Installing dependencies"),
            ("phase4", "Phase 4: Compiling packages"),
            ("phase5", "Phase 5: Generating rosdep list"),
            ("phase6", "Phase 6: Creating package list"),
            ("phase7", "Phase 7: Generating Debian metadata"),
            ("phase8", "Phase 8: Building Debian packages"),
        ]
        for pid, desc in phase_names:
            ui.add_phase(pid, desc)

        # Run all phases successfully
        for pid, _ in phase_names:
            ui.start_phase(pid)
            ui.complete_phase(pid, success=True)

        for pid, _ in phase_names:
            assert ui.phases[pid].status == PhaseStatus.COMPLETED

    def test_elapsed_time_formatting(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase4", "Compiling")
        ui.start_phase("phase4")
        # Manually set times for deterministic test
        ui.phases["phase4"].start_time = 1000.0
        ui.phases["phase4"].end_time = 1045.5
        assert ui.phases["phase4"].elapsed_str() == "45.5s"

    def test_elapsed_time_minutes(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase4", "Compiling")
        ui.start_phase("phase4")
        ui.phases["phase4"].start_time = 1000.0
        ui.phases["phase4"].end_time = 1000.0 + 223.0  # 3m 43s
        assert ui.phases["phase4"].elapsed_str() == "3m 43s"


class TestBuildUIPackages:
    """Test package sub-item tracking within phases."""

    def test_add_package(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase8", "Building Debian packages")
        ui.add_package("phase8", "autoware_lint_common")
        assert len(ui.phases["phase8"].packages) == 1
        assert ui.phases["phase8"].packages[0].name == "autoware_lint_common"
        assert ui.phases["phase8"].packages[0].status == PhaseStatus.RUNNING

    def test_complete_package(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase8", "Building")
        ui.add_package("phase8", "autoware_ndt_scan_matcher")
        ui.complete_package("phase8", "autoware_ndt_scan_matcher", success=True)
        assert ui.phases["phase8"].packages[0].status == PhaseStatus.COMPLETED

    def test_failed_package(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase8", "Building")
        ui.add_package("phase8", "autoware_ndt_scan_matcher")
        ui.complete_package("phase8", "autoware_ndt_scan_matcher", success=False)
        assert ui.phases["phase8"].packages[0].status == PhaseStatus.FAILED

    def test_multiple_packages_mixed_results(self) -> None:
        """Simulate building multiple packages with mixed success/failure."""
        ui = BuildUI()
        ui.add_phase("phase8", "Building")
        packages = [
            ("autoware_lint_common", True),
            ("autoware_ndt_scan_matcher", False),
            ("autoware_carla_interface", True),
            ("autoware_adapi_v1_msgs", True),
        ]
        for name, _ in packages:
            ui.add_package("phase8", name)
        for name, success in packages:
            ui.complete_package("phase8", name, success=success)

        results = {p.name: p.status for p in ui.phases["phase8"].packages}
        assert results["autoware_lint_common"] == PhaseStatus.COMPLETED
        assert results["autoware_ndt_scan_matcher"] == PhaseStatus.FAILED
        assert results["autoware_carla_interface"] == PhaseStatus.COMPLETED

    def test_add_duplicate_package_restarts_it(self) -> None:
        """Re-adding an existing package sets it back to RUNNING."""
        ui = BuildUI()
        ui.add_phase("phase8", "Building")
        ui.add_package("phase8", "autoware_lint_common")
        ui.complete_package("phase8", "autoware_lint_common", success=True)
        assert ui.phases["phase8"].packages[0].status == PhaseStatus.COMPLETED
        # Re-add restarts
        ui.add_package("phase8", "autoware_lint_common")
        assert ui.phases["phase8"].packages[0].status == PhaseStatus.RUNNING


class TestBuildUIRendering:
    """Test rendering produces output without errors."""

    def test_render_empty_ui(self) -> None:
        ui = BuildUI()
        group = ui._render()
        assert group is not None

    def test_render_with_phases(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase1", "Preparing")
        ui.start_phase("phase1")
        ui.complete_phase("phase1")
        ui.add_phase("phase2", "Building")
        ui.start_phase("phase2")
        group = ui._render()
        assert group is not None

    def test_render_with_log_lines(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase4", "Compiling")
        ui.start_phase("phase4")
        ui.update_log("-- Building CXX object CMakeFiles/ndt_scan_matcher.dir/src/ndt_scan_matcher.cpp.o")
        ui.update_log("[100%] Built target ndt_scan_matcher")
        group = ui._render()
        assert group is not None

    def test_refresh_increments_spinner(self) -> None:
        ui = BuildUI()
        ui.add_phase("phase1", "Running")
        ui.start_phase("phase1")
        old_frame = ui._spinner_frame
        ui.refresh()
        assert ui._spinner_frame == old_frame + 1


class TestSimpleBuildUI:
    """Test the SimpleBuildUI (non-Live, for Docker containers)."""

    def test_lifecycle(self, capsys: pytest.CaptureFixture[str]) -> None:
        ui = SimpleBuildUI()
        ui.add_phase("phase1", "Preparing")
        ui.start_phase("phase1")
        ui.complete_phase("phase1", success=True)
        ui.skip_phase("phase2")
        captured = capsys.readouterr()
        assert "Preparing" in captured.out
        assert "Done" in captured.out
