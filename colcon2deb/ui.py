"""Rich-based UI for colcon2deb build progress.

Provides a Claude Code-like interface showing:
- Build phases with status indicators (pending, running, completed)
- Live log tail panel showing the last N lines of the current log
- Minimal, non-flooding terminal output
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from rich.console import Console, Group, RenderableType
from rich.live import Live
from rich.panel import Panel
from rich.text import Text

# Spinner frames for running state
SPINNER_FRAMES = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]


class PhaseStatus(Enum):
    """Status of a build phase."""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class Package:
    """Represents a package being processed within a phase."""

    name: str
    status: PhaseStatus = PhaseStatus.RUNNING


@dataclass
class Phase:
    """Represents a build phase."""

    name: str
    description: str
    status: PhaseStatus = PhaseStatus.PENDING
    packages: list["Package"] = field(default_factory=lambda: list["Package"]())
    start_time: float | None = None
    end_time: float | None = None

    def elapsed_str(self) -> str | None:
        """Get elapsed time as a formatted string."""
        if self.start_time is None:
            return None
        end = self.end_time if self.end_time else time.time()
        elapsed = end - self.start_time
        if elapsed < 60:
            return f"{elapsed:.1f}s"
        minutes = int(elapsed // 60)
        seconds = elapsed % 60
        return f"{minutes}m {seconds:.0f}s"


@dataclass
class BuildUI:
    """Rich-based UI for build progress display.

    Example usage:
        ui = BuildUI()
        ui.add_phase("phase1", "Preparing directories")
        ui.add_phase("phase2", "Compiling packages")

        with ui.live_context():
            ui.start_phase("phase1")
            # do work...
            ui.complete_phase("phase1")

            ui.start_phase("phase2", log_file=Path("/path/to/log"))
            # do work...
            ui.complete_phase("phase2")
    """

    console: Console = field(default_factory=Console)
    phases: dict[str, Phase] = field(default_factory=lambda: dict[str, Phase]())
    phase_order: list[str] = field(default_factory=lambda: list[str]())
    current_phase: str | None = None
    log_lines: deque[str] = field(default_factory=lambda: deque(maxlen=15))
    _live: Live | None = None
    _spinner_frame: int = 0
    # State is mutated by the event tailer thread and rendered by the
    # refresher and main threads; one coarse lock keeps frames consistent.
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def add_phase(self, phase_id: str, description: str) -> None:
        """Add a new phase to track."""
        with self._lock:
            self.phases[phase_id] = Phase(name=phase_id, description=description)
            self.phase_order.append(phase_id)

    def start_phase(self, phase_id: str) -> None:
        """Mark a phase as running."""
        with self._lock:
            if phase_id in self.phases:
                self.phases[phase_id].status = PhaseStatus.RUNNING
                self.phases[phase_id].start_time = time.time()
                self.current_phase = phase_id

    def complete_phase(self, phase_id: str, success: bool = True) -> None:
        """Mark a phase as completed or failed."""
        with self._lock:
            if phase_id in self.phases:
                self.phases[phase_id].status = (
                    PhaseStatus.COMPLETED if success else PhaseStatus.FAILED
                )
                self.phases[phase_id].end_time = time.time()
                if self.current_phase == phase_id:
                    self.current_phase = None

    def skip_phase(self, phase_id: str) -> None:
        """Mark a phase as skipped."""
        with self._lock:
            if phase_id in self.phases:
                self.phases[phase_id].status = PhaseStatus.SKIPPED
                self.phases[phase_id].end_time = time.time()

    def add_package(self, phase_id: str, package: str) -> None:
        """Add a package sub-item to a phase."""
        with self._lock:
            self._add_package_locked(phase_id, package, PhaseStatus.RUNNING)

    def _add_package_locked(self, phase_id: str, package: str, status: PhaseStatus) -> None:
        if phase_id in self.phases:
            phase = self.phases[phase_id]
            for pkg in phase.packages:
                if pkg.name == package:
                    pkg.status = status
                    return
            phase.packages.append(Package(name=package, status=status))

    def complete_package(self, phase_id: str, package: str, success: bool = True) -> None:
        """Mark a package as complete, adding it if it was never started.

        Container scripts may emit only package_complete events, so an
        unknown package is added with its final status instead of being
        silently dropped.
        """
        with self._lock:
            status = PhaseStatus.COMPLETED if success else PhaseStatus.FAILED
            self._add_package_locked(phase_id, package, status)

    def update_log(self, line: str) -> None:
        """Add a line to the log display."""
        self.log_lines.append(line.rstrip())

    def _render_phases(self) -> Text:
        """Render the phase list with status indicators."""
        text = Text()

        for phase_id in self.phase_order:
            phase = self.phases[phase_id]

            # Status indicator
            if phase.status == PhaseStatus.COMPLETED:
                indicator = Text("✓ ", style="green bold")
            elif phase.status == PhaseStatus.FAILED:
                indicator = Text("✗ ", style="red bold")
            elif phase.status == PhaseStatus.RUNNING:
                # Use animated spinner for running phase
                spinner_char = SPINNER_FRAMES[self._spinner_frame % len(SPINNER_FRAMES)]
                indicator = Text(f"{spinner_char} ", style="blue bold")
            elif phase.status == PhaseStatus.SKIPPED:
                indicator = Text("○ ", style="dim")
            else:  # PENDING
                indicator = Text("○ ", style="dim")

            text.append(indicator)
            text.append(phase.description)

            # Add elapsed time and status suffix
            elapsed = phase.elapsed_str()
            if phase.status == PhaseStatus.RUNNING:
                if elapsed:
                    text.append(f" [{elapsed}]", style="blue dim")
            elif phase.status == PhaseStatus.COMPLETED:
                if elapsed:
                    text.append(f" [{elapsed}]", style="dim")
            elif phase.status == PhaseStatus.SKIPPED:
                text.append(" (skipped)", style="dim")
            elif phase.status == PhaseStatus.FAILED:
                if elapsed:
                    text.append(f" [{elapsed}]", style="red dim")
                text.append(" (failed)", style="red dim")

            text.append("\n")

            # Render packages under this phase. Large workspaces have
            # hundreds of packages; show failures and active work
            # individually, summarize the completed bulk.
            completed = [p for p in phase.packages if p.status == PhaseStatus.COMPLETED]
            visible = [p for p in phase.packages if p.status != PhaseStatus.COMPLETED]
            display = visible[-8:] + completed[-3:]
            for pkg in display:
                text.append("    ")  # Indent
                if pkg.status == PhaseStatus.COMPLETED:
                    text.append("✓ ", style="green")
                elif pkg.status == PhaseStatus.FAILED:
                    text.append("✗ ", style="red")
                elif pkg.status == PhaseStatus.RUNNING:
                    spinner_char = SPINNER_FRAMES[self._spinner_frame % len(SPINNER_FRAMES)]
                    text.append(f"{spinner_char} ", style="blue")
                else:
                    text.append("○ ", style="dim")
                text.append(pkg.name, style="dim" if pkg.status == PhaseStatus.COMPLETED else None)
                text.append("\n")
            hidden = len(phase.packages) - len(display)
            if hidden > 0:
                failed_count = sum(1 for p in phase.packages if p.status == PhaseStatus.FAILED)
                summary = f"    … {hidden} more ({len(completed)} done"
                if failed_count:
                    summary += f", {failed_count} failed"
                summary += ")\n"
                text.append(summary, style="dim")

        return text

    def _render_log_panel(self) -> Panel | None:
        """Render the log tail panel."""
        if not self.log_lines:
            return None

        log_text = Text()
        for i, line in enumerate(self.log_lines):
            # Truncate long lines
            display_line = line[:120] + "..." if len(line) > 120 else line
            log_text.append(display_line, style="dim")
            if i < len(self.log_lines) - 1:
                log_text.append("\n")

        title = "Log"
        return Panel(
            log_text,
            title=title,
            title_align="left",
            border_style="dim",
            padding=(0, 1),
        )

    def _render(self) -> Group:
        """Render the complete UI."""
        renderables: list[RenderableType] = []

        # Phase list
        phases_text = self._render_phases()
        renderables.append(phases_text)

        # Log panel (if we have log lines)
        log_panel = self._render_log_panel()
        if log_panel:
            renderables.append(log_panel)

        return Group(*renderables)

    def live_context(self) -> Live:
        """Get a Live context for rendering updates."""
        self._live = Live(
            self._render(),
            console=self.console,
            refresh_per_second=4,
            transient=False,
        )
        return self._live

    def refresh(self) -> None:
        """Manually refresh the display."""
        with self._lock:
            self._spinner_frame += 1
            if self._live:
                self._live.update(self._render())


class SimpleBuildUI:
    """Simpler UI that doesn't use Live display.

    Useful for environments where Live display may not work well
    (e.g., inside Docker with limited TTY support).
    """

    def __init__(self) -> None:
        self.console = Console()
        self.phases: dict[str, Phase] = {}
        self.phase_order: list[str] = []
        self.current_phase: str | None = None

    def add_phase(self, phase_id: str, description: str) -> None:
        """Add a new phase to track."""
        self.phases[phase_id] = Phase(name=phase_id, description=description)
        self.phase_order.append(phase_id)

    def start_phase(self, phase_id: str, log_file: Path | None = None) -> None:
        """Mark a phase as running."""
        if phase_id in self.phases:
            self.phases[phase_id].status = PhaseStatus.RUNNING
            self.current_phase = phase_id
            phase = self.phases[phase_id]
            self.console.print(f"[blue]●[/blue] {phase.description}...", highlight=False)

    def complete_phase(self, phase_id: str, success: bool = True) -> None:
        """Mark a phase as completed or failed."""
        if phase_id in self.phases:
            phase = self.phases[phase_id]
            if success:
                phase.status = PhaseStatus.COMPLETED
                self.console.print("  [green]✓[/green] Done", highlight=False)
            else:
                phase.status = PhaseStatus.FAILED
                self.console.print("  [red]✗[/red] Failed", highlight=False)

            if self.current_phase == phase_id:
                self.current_phase = None

    def skip_phase(self, phase_id: str) -> None:
        """Mark a phase as skipped."""
        if phase_id in self.phases:
            phase = self.phases[phase_id]
            phase.status = PhaseStatus.SKIPPED
            self.console.print(f"[dim]○ {phase.description} (skipped)[/dim]", highlight=False)

    def print_summary(self) -> None:
        """Print final summary of all phases."""
        self.console.print()
        for phase_id in self.phase_order:
            phase = self.phases[phase_id]
            if phase.status == PhaseStatus.COMPLETED:
                self.console.print(f"[green]✓[/green] {phase.description}", highlight=False)
            elif phase.status == PhaseStatus.FAILED:
                self.console.print(f"[red]✗[/red] {phase.description}", highlight=False)
            elif phase.status == PhaseStatus.SKIPPED:
                self.console.print(f"[dim]○ {phase.description} (skipped)[/dim]", highlight=False)
            else:
                self.console.print(f"[dim]○ {phase.description}[/dim]", highlight=False)
