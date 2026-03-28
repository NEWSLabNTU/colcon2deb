"""Tests for the event protocol used for container-to-host TUI communication.

Events are JSONL (one JSON object per line) written to a shared file.
Based on real event sequences from Autoware 1.5.0/amd64 builds.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest


def parse_events(path: Path) -> list[dict]:
    """Parse a JSONL events file into a list of event dicts."""
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


class TestEventParsing:
    """Test parsing of event JSONL files."""

    def test_successful_build_events(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "successful_build.jsonl")
        assert len(events) == 18
        assert events[0]["type"] == "build_start"
        assert events[0]["phases"] == 8
        assert events[-1]["type"] == "build_complete"
        assert events[-1]["success"] is True

    def test_all_phases_present_in_successful_build(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "successful_build.jsonl")
        phase_starts = [e for e in events if e["type"] == "phase_start"]
        phase_completes = [e for e in events if e["type"] == "phase_complete"]
        assert len(phase_starts) == 8
        assert len(phase_completes) == 8
        # All phases succeeded
        for pc in phase_completes:
            assert pc["success"] is True

    def test_phase_ordering(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "successful_build.jsonl")
        phase_starts = [e for e in events if e["type"] == "phase_start"]
        phases_in_order = [e["phase"] for e in phase_starts]
        assert phases_in_order == [1, 2, 3, 4, 5, 6, 7, 8]

    def test_failed_build_stops_at_phase(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "failed_at_phase4.jsonl")
        assert events[-1]["type"] == "build_complete"
        assert events[-1]["success"] is False
        # Phase 4 failed
        phase4_complete = [
            e
            for e in events
            if e["type"] == "phase_complete" and e["phase"] == 4
        ]
        assert len(phase4_complete) == 1
        assert phase4_complete[0]["success"] is False
        # Phases 5-8 should not appear
        later_phases = [
            e for e in events if e.get("phase", 0) > 4 and e["type"] == "phase_start"
        ]
        assert len(later_phases) == 0

    def test_partial_package_failures(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "partial_package_failures.jsonl")
        pkg_completes = [e for e in events if e["type"] == "package_complete"]
        succeeded = [e for e in pkg_completes if e["success"] is True]
        failed = [e for e in pkg_completes if e["success"] is False]
        assert len(succeeded) == 3  # lint_common, ndt in phase 7; lint_common in phase 8
        assert len(failed) == 1  # ndt failed in phase 8
        # Build still completes (partial success)
        assert events[-1]["type"] == "build_complete"
        assert events[-1]["success"] is True

    def test_event_timestamps_are_iso_format(self, events_dir: Path) -> None:
        events = parse_events(events_dir / "successful_build.jsonl")
        for event in events:
            ts = event["timestamp"]
            # Should be ISO format: YYYY-MM-DDTHH:MM:SS.ffffff
            assert "T" in ts
            assert len(ts) >= 19

    def test_phase_names_present(self, events_dir: Path) -> None:
        """Phase start events include a human-readable name."""
        events = parse_events(events_dir / "successful_build.jsonl")
        phase_starts = [e for e in events if e["type"] == "phase_start"]
        for ps in phase_starts:
            assert "name" in ps
            assert isinstance(ps["name"], str)
            assert len(ps["name"]) > 0
