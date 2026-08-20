"""Tests for the host-side event file reader.

The old tailer advanced its file position past partially-written lines,
permanently dropping those events, and never drained the file after the
container exited — the final phase_complete/build_complete events were lost.
"""

from __future__ import annotations

from pathlib import Path

from colcon2deb.main import read_new_events
from colcon2deb.ui import BuildUI, PhaseStatus


class TestReadNewEvents:
    def test_reads_complete_lines(self, tmp_path: Path) -> None:
        f = tmp_path / "events.jsonl"
        f.write_text('{"type": "phase_start", "phase": 1}\n{"type": "phase_complete", "phase": 1}\n')
        events, pos = read_new_events(f, 0)
        assert [e["type"] for e in events] == ["phase_start", "phase_complete"]
        assert pos == f.stat().st_size

    def test_partial_line_not_consumed(self, tmp_path: Path) -> None:
        """A line still being written must be left for the next poll."""
        f = tmp_path / "events.jsonl"
        f.write_text('{"type": "phase_start", "phase": 1}\n{"type": "phase_co')
        events, pos = read_new_events(f, 0)
        assert len(events) == 1

        # Writer finishes the line; the next read must pick it up whole
        with open(f, "a") as fh:
            fh.write('mplete", "phase": 1}\n')
        events2, pos2 = read_new_events(f, pos)
        assert [e["type"] for e in events2] == ["phase_complete"]
        assert pos2 == f.stat().st_size

    def test_missing_file(self, tmp_path: Path) -> None:
        events, pos = read_new_events(tmp_path / "nope.jsonl", 0)
        assert events == []
        assert pos == 0

    def test_invalid_json_line_skipped(self, tmp_path: Path) -> None:
        f = tmp_path / "events.jsonl"
        f.write_text('not json\n{"type": "phase_start", "phase": 2}\n')
        events, pos = read_new_events(f, 0)
        assert [e["type"] for e in events] == ["phase_start"]
        assert pos == f.stat().st_size


class TestCompletePackageAddsMissing:
    def test_complete_without_start_still_displays(self) -> None:
        """Container scripts emit only package_complete (no start); the UI
        must add the package on completion instead of dropping the event."""
        ui = BuildUI()
        ui.add_phase("phase8", "Building Debian packages")
        ui.start_phase("phase8")
        ui.complete_package("phase8", "my_pkg", success=False)
        pkgs = ui.phases["phase8"].packages
        assert len(pkgs) == 1
        assert pkgs[0].name == "my_pkg"
        assert pkgs[0].status == PhaseStatus.FAILED
