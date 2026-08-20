"""Per-package colcon build tracking for the pipelined build.

Colcon writes one structured event stream per run under
log/build_<timestamp>/events.log, with lines such as:

    [0.101057] (test_cpp_pkg) JobEnded: {'identifier': 'test_cpp_pkg', 'rc': 0}

The pipelined build starts a package's .deb build as soon as its JobEnded
event lands, while colcon keeps building the rest of the workspace. The
event log is per-run, so stale install-tree state from previous builds
cannot release a package early.
"""

from __future__ import annotations

import re
import threading
import time
from pathlib import Path

# [ts] (pkg) JobEnded: {'identifier': 'pkg', 'rc': N}
_JOB_ENDED_RE = re.compile(r"^\[[0-9.]+\]\s+\(([^)]+)\)\s+JobEnded:\s+\{[^}]*'rc':\s*(-?\d+)")


def parse_job_ended(line: str) -> tuple[str, int] | None:
    """Parse a JobEnded event line. Returns (package, rc) or None."""
    m = _JOB_ENDED_RE.match(line.strip())
    if not m:
        return None
    return m.group(1), int(m.group(2))


def snapshot_build_dirs(log_base: Path) -> set[str]:
    """Names of colcon build_* log dirs that already exist.

    Taken before launching colcon; find_events_log then looks only at
    dirs not in this set, so a previous run's log can never be mistaken
    for the current run's (filesystem mtimes are too coarse to compare
    against wall-clock time reliably).
    """
    try:
        return {d.name for d in log_base.iterdir() if d.name.startswith("build_")}
    except OSError:
        return set()


def find_events_log(log_base: Path, exclude: set[str]) -> Path | None:
    """Find the events.log of the colcon run whose dir is not in `exclude`."""
    try:
        candidates = [
            d
            for d in log_base.iterdir()
            if d.is_dir() and d.name.startswith("build_") and d.name not in exclude
        ]
    except OSError:
        return None
    for d in sorted(candidates, key=lambda d: d.name, reverse=True):
        events = d / "events.log"
        if events.exists():
            return events
    return None


class ColconGate:
    """Blocks .deb builds until colcon has finished the package.

    A package is released when any of:
    - its JobEnded event with rc == 0 has been seen,
    - the colcon status file reports overall success (releases everything),
    - the gate is disabled (--skip-colcon-build: the install tree is
      trusted as-is).

    A package is refused (wait returns False) when its JobEnded has
    rc != 0, or when colcon finished with a failure before the package's
    JobEnded was seen.
    """

    def __init__(
        self,
        log_base: Path,
        status_file: Path,
        exclude_dirs: set[str] | None = None,
        poll_interval: float = 0.2,
    ) -> None:
        self._log_base = log_base
        self._status_file = status_file
        self._exclude_dirs: set[str] = exclude_dirs if exclude_dirs is not None else set()
        self._poll_interval = poll_interval
        self._cond = threading.Condition()
        self._ended: dict[str, int] = {}  # package -> rc
        self._colcon_rc: int | None = None
        self._disabled = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._events_log: Path | None = None
        self._position = 0

    @classmethod
    def disabled(cls) -> ColconGate:
        """A gate that releases every package immediately."""
        gate = cls(Path("/nonexistent"), Path("/nonexistent"))
        gate._disabled = True
        return gate

    def start(self) -> None:
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def wait(self, package: str, timeout: float | None = None) -> bool:
        """Block until colcon's verdict on `package` is known.

        Returns True if the package's dependencies are ready for a .deb
        build, False if colcon failed before finishing this package.
        """
        if self._disabled:
            return True
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cond:
            while True:
                rc = self._ended.get(package)
                if rc is not None:
                    return rc == 0
                if self._colcon_rc is not None:
                    # Colcon is done; no JobEnded for this package. Overall
                    # success releases it (colcon considers the workspace
                    # built); failure refuses it.
                    return self._colcon_rc == 0
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return False
                self._cond.wait(
                    timeout=self._poll_interval
                    if remaining is None
                    else min(self._poll_interval, remaining)
                )

    def _watch(self) -> None:
        while not self._stop.is_set():
            self._poll_once()
            with self._cond:
                if self._colcon_rc is not None:
                    return
            self._stop.wait(self._poll_interval)

    def _poll_once(self) -> None:
        if self._events_log is None:
            self._events_log = find_events_log(self._log_base, self._exclude_dirs)

        new_ended: dict[str, int] = {}
        if self._events_log is not None:
            try:
                with open(self._events_log, "rb") as f:
                    f.seek(self._position)
                    chunk = f.read()
            except OSError:
                chunk = b""
            consumed = 0
            while True:
                newline = chunk.find(b"\n", consumed)
                if newline == -1:
                    break
                line = chunk[consumed:newline].decode("utf-8", errors="replace")
                consumed = newline + 1
                parsed = parse_job_ended(line)
                if parsed:
                    new_ended[parsed[0]] = parsed[1]
            self._position += consumed

        colcon_rc: int | None = None
        try:
            text = self._status_file.read_text().strip()
            if text:
                colcon_rc = int(text)
        except (OSError, ValueError):
            pass

        if new_ended or colcon_rc is not None:
            with self._cond:
                self._ended.update(new_ended)
                if colcon_rc is not None:
                    self._colcon_rc = colcon_rc
                self._cond.notify_all()
