"""Utility functions for logging and command execution."""

import logging
import subprocess
from pathlib import Path
from typing import Optional, Union
from datetime import datetime


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

logger = logging.getLogger(__name__)


def print_phase(message: str):
    """Print a timestamped phase message."""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"\n[{timestamp}] {message}")
    logger.info(message)


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
