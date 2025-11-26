"""Utility functions for logging and command execution."""

import logging
import subprocess
from pathlib import Path
from typing import Optional, Union
from datetime import datetime


# ANSI color codes
class Colors:
    """ANSI color codes for terminal output."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'

    # Foreground colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # Bright foreground colors
    BRIGHT_BLACK = '\033[90m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'


class ColoredFormatter(logging.Formatter):
    """Custom formatter that adds colors to log levels."""

    COLORS = {
        'DEBUG': Colors.DIM + Colors.WHITE,
        'INFO': Colors.CYAN,
        'WARNING': Colors.YELLOW,
        'ERROR': Colors.RED,
        'CRITICAL': Colors.BOLD + Colors.RED,
    }

    def format(self, record):
        # Add color to levelname
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = f"{self.COLORS[levelname]}{levelname}{Colors.RESET}"

        # Format the message
        result = super().format(record)

        # Reset levelname for other handlers
        record.levelname = levelname

        return result


# Configure logging with colors
handler = logging.StreamHandler()
handler.setFormatter(ColoredFormatter(
    fmt='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
))

logging.basicConfig(
    level=logging.INFO,
    handlers=[handler]
)

logger = logging.getLogger(__name__)

# Global display instance (set by main)
_display = None


def set_display(display):
    """Set the global display instance for Docker-style output.

    Args:
        display: BuildDisplay instance or None
    """
    global _display
    _display = display


def get_display():
    """Get the global display instance.

    Returns:
        BuildDisplay instance or None
    """
    return _display


def print_phase(message: str):
    """Print a timestamped phase message.

    Deprecated: Use start_phase() with phase number instead.

    Args:
        message: Phase message
    """
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"\n[{timestamp}] {message}")
    logger.info(message)


def start_phase(phase_num: int, name: str):
    """Start a build phase with Docker-style display.

    Args:
        phase_num: Phase number (1-indexed)
        name: Phase name
    """
    if _display:
        _display.start_phase(phase_num, name)
    else:
        # Fallback to simple logging
        print_phase(f"Phase {phase_num}: {name}")


def complete_phase(phase_num: int, success: bool = True):
    """Mark a phase as complete.

    Args:
        phase_num: Phase number (1-indexed)
        success: Whether phase succeeded
    """
    if _display:
        _display.complete_phase(phase_num, success)


def add_subtask(phase_num: int, message: str):
    """Add a subtask message to current phase.

    Args:
        phase_num: Phase number (1-indexed)
        message: Subtask message
    """
    if _display:
        _display.add_subtask(phase_num, message)


def run_command(
    cmd: list[str],
    cwd: Optional[Path] = None,
    log_file: Optional[Path] = None,
    check: bool = True,
    capture_output: bool = False,
) -> subprocess.CompletedProcess:
    """Run a command with optional logging.

    Args:
        cmd: Command and arguments as list
        cwd: Working directory
        log_file: Optional log file to write output
        check: Raise exception on non-zero exit code
        capture_output: Capture stdout/stderr (if log_file not set)

    Returns:
        CompletedProcess instance

    Raises:
        subprocess.CalledProcessError: If check=True and command fails
    """
    logger.debug(f"Running: {' '.join(cmd)}")

    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        with open(log_file, 'w') as f:
            f.write(f"Command: {' '.join(cmd)}\n")
            f.write("=" * 80 + "\n\n")

            result = subprocess.run(
                cmd,
                cwd=cwd,
                stdout=f,
                stderr=subprocess.STDOUT,
                check=False,
            )

        if result.returncode != 0:
            logger.error(f"Command failed with exit code {result.returncode}")
            logger.error(f"  See log: {log_file}")
            if check:
                # Show last 20 lines of log
                with open(log_file) as f:
                    lines = f.readlines()
                    for line in lines[-20:]:
                        print(line, end='')
                raise subprocess.CalledProcessError(result.returncode, cmd)
    else:
        result = subprocess.run(
            cmd,
            cwd=cwd,
            capture_output=capture_output,
            text=True,
            check=check,
        )

    return result


def ensure_dir(path: Union[str, Path]) -> Path:
    """Ensure directory exists, create if needed."""
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path
