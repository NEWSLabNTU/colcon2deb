"""Container-side event emitter for TUI communication.

This module writes structured events to a shared file that the host
process tails to update the TUI. Regular print() statements are
unaffected and will appear in the log panel.
"""

import json
from datetime import datetime
from pathlib import Path

# Event file path (set by init())
_event_file: Path | None = None


def init(output_dir: Path):
    """Initialize the event emitter with the output directory.

    Args:
        output_dir: Path to the output directory (e.g., /output)
    """
    global _event_file
    _event_file = output_dir / ".events.jsonl"
    # Clear previous events
    _event_file.write_text("")


def emit(event_type: str, **kwargs):
    """Emit an event to the shared event file.

    Args:
        event_type: Type of event (e.g., "phase_start", "phase_complete")
        **kwargs: Additional event data
    """
    if _event_file is None:
        return

    event = {"type": event_type, "timestamp": datetime.now().isoformat(), **kwargs}

    with open(_event_file, "a") as f:
        f.write(json.dumps(event) + "\n")
        f.flush()


# Convenience functions for common events
def build_start(total_phases: int = 8):
    """Emit build start event."""
    emit("build_start", phases=total_phases)


def build_complete(success: bool):
    """Emit build complete event."""
    emit("build_complete", success=success)


def phase_start(phase: int, name: str):
    """Emit phase start event."""
    emit("phase_start", phase=phase, name=name)


def phase_complete(phase: int, success: bool):
    """Emit phase complete event."""
    emit("phase_complete", phase=phase, success=success)


def phase_skip(phase: int):
    """Emit phase skip event."""
    emit("phase_skip", phase=phase)


def package_start(phase: int, package: str):
    """Emit package start event (for phases with sub-items)."""
    emit("package_start", phase=phase, package=package)


def package_complete(phase: int, package: str, success: bool):
    """Emit package complete event."""
    emit("package_complete", phase=phase, package=package, success=success)
