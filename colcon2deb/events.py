"""Event protocol for container-to-host TUI communication.

Events are written to a shared file (.events.jsonl) in the output directory.
The host process tails this file and updates the TUI accordingly.
"""

# Event file name (relative to output directory)
EVENT_FILE = ".events.jsonl"

# Event types
BUILD_START = "build_start"
BUILD_COMPLETE = "build_complete"
PHASE_START = "phase_start"
PHASE_COMPLETE = "phase_complete"
PHASE_SKIP = "phase_skip"
PACKAGE_START = "package_start"
PACKAGE_COMPLETE = "package_complete"
LOG_LINE = "log"
