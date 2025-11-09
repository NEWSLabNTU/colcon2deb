"""Docker-style terminal display for build progress using Rich."""

import time
from enum import Enum
from typing import Optional, Dict, Any, List
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TimeElapsedColumn
from rich.text import Text


class PhaseStatus(Enum):
    """Status indicators for build phases."""
    PENDING = ("⏸", "dim")
    RUNNING = ("↻", "cyan")
    DONE = ("✓", "green")
    FAILED = ("✗", "red")
    CACHED = ("⚡", "yellow")


class BuildDisplay:
    """Docker-style build display with rich progress bars.

    Provides a live-updating terminal interface showing:
    - Numbered stages [N/total]
    - Status indicators (pending, running, done, failed)
    - Subtask messages
    - Progress bars for parallel operations
    - Elapsed time per phase

    Example output:
        [1/8] Preparing working directories        ✓ 0.1s
        [2/8] Copying source files                 ✓ 2.3s
        [3/8] Installing dependencies              ↻ Running...
              └─ apt update                        ✓
              └─ apt install (234 packages)        ... 45%
        [4/8] Compiling packages                   ⏸ Pending
    """

    def __init__(self, total_phases: int = 8, enable_live: bool = True, verbose: bool = False):
        """Initialize the build display.

        Args:
            total_phases: Total number of build phases
            enable_live: Whether to use live display (set False for non-TTY)
            verbose: Whether to show verbose debug output
        """
        self.console = Console()
        self.total_phases = total_phases
        self.phases: Dict[int, Dict[str, Any]] = {}
        self.current_phase: Optional[int] = None
        self.live: Optional[Live] = None
        self.progress: Optional[Progress] = None
        self.enable_live = enable_live and self.console.is_terminal
        self.verbose = verbose

        # Initialize phases
        for i in range(1, total_phases + 1):
            self.phases[i] = {
                "name": "...",
                "status": PhaseStatus.PENDING,
                "start_time": None,
                "elapsed": None,
                "subtasks": []
            }

    def __enter__(self):
        """Enter context manager."""
        if self.enable_live:
            # Create progress display for parallel operations
            self.progress = Progress(
                SpinnerColumn(),
                TextColumn("[bold]{task.description}"),
                BarColumn(),
                TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
                TimeElapsedColumn(),
                console=self.console,
                transient=True,  # Progress bar disappears when done
            )

            # Start live display
            self.live = Live(
                self._render(),
                console=self.console,
                refresh_per_second=4,
                transient=False
            )
            self.live.__enter__()

        return self

    def __exit__(self, *args):
        """Exit context manager."""
        if self.live:
            # Final render
            self.live.update(self._render(), refresh=True)
            self.live.__exit__(*args)

        if self.progress:
            self.progress.stop()

    def _render(self) -> Table:
        """Render the current display state as a Rich table."""
        table = Table.grid(padding=(0, 1))
        table.add_column(style="bold cyan", justify="left", width=8)
        table.add_column()
        table.add_column(justify="right", width=8)

        for phase_num in range(1, self.total_phases + 1):
            phase = self.phases[phase_num]
            status = phase["status"]

            # Phase line
            prefix = f"[{phase_num}/{self.total_phases}]"
            status_icon, status_style = status.value
            name = phase["name"]

            # Elapsed time
            elapsed_str = ""
            if phase["elapsed"] is not None:
                elapsed_str = f"{phase['elapsed']:.1f}s"
            elif phase["start_time"] is not None and status == PhaseStatus.RUNNING:
                current_elapsed = time.time() - phase["start_time"]
                elapsed_str = f"{current_elapsed:.1f}s"

            # Build phase line with color
            status_text = Text(status_icon, style=status_style)
            name_text = Text(f" {name}")

            table.add_row(prefix, status_text + name_text, elapsed_str)

            # Subtasks (indented)
            for subtask in phase.get("subtasks", [])[-5:]:  # Show last 5 subtasks
                table.add_row("", f"  └─ {subtask}", "")

        # Add progress bars if active
        if self.progress and self.progress.tasks:
            table.add_row("", "", "")
            for task in self.progress.tasks:
                if not task.finished:
                    progress_line = self.progress.make_tasks_table([task])
                    table.add_row("", progress_line, "")

        return table

    def start_phase(self, phase_num: int, name: str):
        """Start a new phase.

        Args:
            phase_num: Phase number (1-indexed)
            name: Phase name/description
        """
        self.current_phase = phase_num
        self.phases[phase_num]["name"] = name
        self.phases[phase_num]["status"] = PhaseStatus.RUNNING
        self.phases[phase_num]["start_time"] = time.time()
        self.phases[phase_num]["subtasks"] = []  # Clear previous subtasks

        if self.enable_live and self.live:
            self.live.update(self._render(), refresh=True)
        else:
            # Fallback to simple print
            print(f"[{phase_num}/{self.total_phases}] {name}")

    def complete_phase(self, phase_num: int, success: bool = True):
        """Mark phase as complete.

        Args:
            phase_num: Phase number (1-indexed)
            success: Whether phase completed successfully
        """
        phase = self.phases[phase_num]
        phase["status"] = PhaseStatus.DONE if success else PhaseStatus.FAILED

        if phase["start_time"] is not None:
            phase["elapsed"] = time.time() - phase["start_time"]

        if self.enable_live and self.live:
            self.live.update(self._render(), refresh=True)
        else:
            # Fallback to simple print
            status = "✓" if success else "✗"
            elapsed = phase.get("elapsed", 0)
            print(f"[{phase_num}/{self.total_phases}] {status} {phase['name']} ({elapsed:.1f}s)")

    def add_subtask(self, phase_num: int, message: str):
        """Add a subtask message to a phase.

        Args:
            phase_num: Phase number (1-indexed)
            message: Subtask message
        """
        if phase_num not in self.phases:
            return

        self.phases[phase_num]["subtasks"].append(message)

        if self.enable_live and self.live:
            self.live.update(self._render(), refresh=True)

    def log(self, message: str, level: str = "info"):
        """Log a message (appears above the live display).

        Args:
            message: Message to log
            level: Log level (info, warning, error, debug)
        """
        # Skip debug messages unless verbose mode is enabled
        if level == "debug" and not self.verbose:
            return

        if self.enable_live and self.live:
            # Temporarily stop live display to print message
            self.live.console.print(message)
        else:
            print(message)

    def create_progress_task(self, description: str, total: int):
        """Create a progress bar task for parallel operations.

        Args:
            description: Task description
            total: Total number of items

        Returns:
            Task ID for updating progress
        """
        if self.progress:
            return self.progress.add_task(description, total=total)
        return None

    def update_progress(self, task_id, advance: int = 1):
        """Update progress bar.

        Args:
            task_id: Task ID from create_progress_task
            advance: Number of items to advance
        """
        if self.progress and task_id is not None:
            self.progress.update(task_id, advance=advance)

            if self.enable_live and self.live:
                self.live.update(self._render(), refresh=True)

    def remove_progress_task(self, task_id):
        """Remove a progress bar task.

        Args:
            task_id: Task ID to remove
        """
        if self.progress and task_id is not None:
            self.progress.remove_task(task_id)

            if self.enable_live and self.live:
                self.live.update(self._render(), refresh=True)
