"""Round-trip tests for the container→host event protocol.

The container emits events via colcon2deb.helper.events; the host tails the
file and dispatches on the type constants in colcon2deb.events. These tests
run the real emitter and check the host-side contract, so writer/reader
format drift fails a test instead of silently breaking the TUI.
"""

from __future__ import annotations

import json
from pathlib import Path

from colcon2deb import events as host_events
from colcon2deb.helper import events as emitter


def _read_events(output_dir: Path) -> list[dict[str, object]]:
    event_file = output_dir / host_events.EVENT_FILE
    return [json.loads(line) for line in event_file.read_text().splitlines() if line.strip()]


class TestEmitterRoundTrip:
    def test_emitter_writes_to_host_event_file(self, tmp_path: Path) -> None:
        """The emitter and the host must agree on the event file name."""
        emitter.init(tmp_path)
        emitter.build_start(total_phases=8)
        assert (tmp_path / host_events.EVENT_FILE).exists()

    def test_full_build_sequence(self, tmp_path: Path) -> None:
        emitter.init(tmp_path)
        emitter.build_start(total_phases=8)
        emitter.phase_start(1, "Preparing")
        emitter.phase_complete(1, success=True)
        emitter.phase_skip(7)
        emitter.package_start(8, "my_pkg")
        emitter.package_complete(8, "my_pkg", success=False)
        emitter.build_complete(success=False)

        events = _read_events(tmp_path)
        types = [e["type"] for e in events]
        assert types == [
            host_events.BUILD_START,
            host_events.PHASE_START,
            host_events.PHASE_COMPLETE,
            host_events.PHASE_SKIP,
            host_events.PACKAGE_START,
            host_events.PACKAGE_COMPLETE,
            host_events.BUILD_COMPLETE,
        ]

    def test_events_are_one_json_object_per_line(self, tmp_path: Path) -> None:
        emitter.init(tmp_path)
        emitter.phase_start(1, "Preparing")
        emitter.phase_complete(1, success=True)
        raw = (tmp_path / host_events.EVENT_FILE).read_text()
        lines = [ln for ln in raw.splitlines() if ln.strip()]
        assert len(lines) == 2
        for line in lines:
            json.loads(line)  # must each parse standalone

    def test_event_payload_fields(self, tmp_path: Path) -> None:
        emitter.init(tmp_path)
        emitter.phase_complete(4, success=False)
        emitter.package_complete(8, "pkg_a", success=True)
        events = _read_events(tmp_path)
        phase_evt, pkg_evt = events
        assert phase_evt["phase"] == 4
        assert phase_evt["success"] is False
        assert "timestamp" in phase_evt
        assert pkg_evt["package"] == "pkg_a"
        assert pkg_evt["success"] is True

    def test_init_clears_previous_events(self, tmp_path: Path) -> None:
        emitter.init(tmp_path)
        emitter.build_start()
        emitter.init(tmp_path)
        assert _read_events(tmp_path) == []

    def test_emit_without_init_is_noop(self) -> None:
        emitter._event_file = None  # type: ignore[attr-defined]
        emitter.build_start()  # must not raise
