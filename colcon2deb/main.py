#!/usr/bin/env python3
"""Docker run script replacement for make run with enhanced functionality."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import uuid
from importlib.metadata import version
from pathlib import Path
from types import FrameType
from typing import Any

__version__ = version("colcon2deb")

from .config import ConfigError, load_config_file, validate_config
from .events import EVENT_FILE
from .ui import BuildUI

# Regex to strip ANSI escape codes
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")

# Global state for signal handling
_container_id = None
_interrupt_count = 0
_interrupt_lock = threading.Lock()


def stop_container(container_id: str, force: bool = False) -> None:
    """Stop a Docker container."""
    if not container_id:
        return

    try:
        if force:
            print(f"\nForce killing container {container_id[:12]}...", file=sys.stderr)
            subprocess.run(["docker", "kill", container_id], capture_output=True, timeout=10)
        else:
            print(
                f"\nStopping container {container_id[:12]} (press Ctrl+C again to force)...",
                file=sys.stderr,
            )
            subprocess.run(
                ["docker", "stop", "-t", "10", container_id], capture_output=True, timeout=20
            )
    except subprocess.TimeoutExpired:
        # If stop times out, try kill
        try:
            subprocess.run(["docker", "kill", container_id], capture_output=True, timeout=5)
        except Exception as e:
            print(f"Warning: Failed to kill container {container_id[:12]}: {e}", file=sys.stderr)
    except Exception as e:
        print(f"Warning: Failed to stop container: {e}", file=sys.stderr)


def signal_handler(signum: int, frame: FrameType | None) -> None:
    """Handle interrupt signals with escalating force.

    docker stop can block for many seconds; running it in a background
    thread keeps the terminal responsive so a second Ctrl+C escalates
    immediately.
    """
    global _interrupt_count

    with _interrupt_lock:
        _interrupt_count += 1
        count = _interrupt_count
        container = _container_id

    if count == 1:
        print("\n\nReceived interrupt signal. Stopping container gracefully...", file=sys.stderr)
        print(
            "Press Ctrl+C again to force stop, or 3 times to force kill immediately.",
            file=sys.stderr,
        )
        if container:
            threading.Thread(target=stop_container, args=(container, False), daemon=True).start()
    elif count == 2:
        print("\n\nReceived second interrupt. Force stopping...", file=sys.stderr)
        if container:
            threading.Thread(target=stop_container, args=(container, True), daemon=True).start()
    else:
        print("\n\nReceived third interrupt. Forcing immediate exit...", file=sys.stderr)
        if container:
            stop_container(container, force=True)
        sys.exit(130)  # 128 + SIGINT


def check_docker_prereqs() -> str | None:
    """Verify docker exists and its daemon is reachable.

    Returns an error message, or None when everything is usable. Runs
    before any expensive work so a stopped daemon or missing install
    fails in seconds, not after minutes of setup.
    """
    if shutil.which("docker") is None:
        return "'docker' command not found. Install Docker and ensure it is on PATH."
    try:
        result = subprocess.run(["docker", "info"], capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return "Docker daemon did not respond within 30s (is it running?)"
    if result.returncode != 0:
        detail = (result.stderr or "").strip().splitlines()
        hint = f": {detail[-1]}" if detail else ""
        return (
            "Docker daemon is not reachable (is it running? do you have permission "
            f"to use it?){hint}"
        )
    return None


def read_new_events(event_file: Path, position: int) -> tuple[list[dict[str, Any]], int]:
    """Read newline-terminated events from event_file starting at position.

    Returns the parsed events and the new byte position. A partially
    written trailing line is left in place for the next call, so no
    event is ever half-read and dropped.
    """
    try:
        with open(event_file, "rb") as f:
            f.seek(position)
            chunk = f.read()
    except OSError:
        return [], position

    events: list[dict[str, Any]] = []
    consumed = 0
    while True:
        newline = chunk.find(b"\n", consumed)
        if newline == -1:
            break
        line = chunk[consumed:newline].decode("utf-8", errors="replace").strip()
        consumed = newline + 1
        if line:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return events, position + consumed


def _run_container_plain(docker_cmd: list[str], container_name: str) -> int:
    """Run the container streaming raw output — for non-TTY environments.

    The container's own line-oriented progress output narrates the build;
    no Rich Live display, no event tailing, no 4 Hz frame dumps in CI logs.
    """
    process = subprocess.Popen(
        docker_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        bufsize=1,
    )
    try:
        if process.stdout:
            for line in process.stdout:
                print(_ANSI_ESCAPE_RE.sub("", line.rstrip()), flush=True)
        process.wait()
        return process.returncode
    finally:
        if process.poll() is None:
            stop_container(container_name, force=True)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()


def run_container_with_tui(docker_cmd: list[str], container_name: str, output_dir: Path) -> int:
    """Run a Docker container with TUI display (plain streaming when not a TTY).

    Args:
        docker_cmd: Docker command to run
        container_name: Unique container name (used for signal-based cleanup)
        output_dir: Output directory where events file will be written

    Returns:
        Container exit code
    """
    global _container_id, _interrupt_count

    # Reset interrupt count
    with _interrupt_lock:
        _interrupt_count = 0
        _container_id = container_name

    # Set up signal handler
    original_handler = signal.signal(signal.SIGINT, signal_handler)
    process: subprocess.Popen[str] | None = None

    try:
        if not sys.stdout.isatty():
            return _run_container_plain(docker_cmd, container_name)

        # Initialize UI
        ui = BuildUI()
        ui.add_phase("phase1", "Phase 1: Preparing working directories")
        ui.add_phase("phase2", "Phase 2: Copying source files")
        ui.add_phase("phase3", "Phase 3: Installing dependencies")
        ui.add_phase("phase4", "Phase 4: Compiling packages")
        ui.add_phase("phase5", "Phase 5: Generating rosdep list")
        ui.add_phase("phase6", "Phase 6: Creating package list")
        ui.add_phase("phase7", "Phase 7: Generating Debian metadata")
        ui.add_phase("phase8", "Phase 8: Building Debian packages")

        event_file = output_dir / EVENT_FILE
        stop_event = threading.Event()

        # Clear events file from previous run
        if event_file.exists():
            event_file.unlink()

        def handle_event(event: dict[str, Any]) -> None:
            """Process an event and update UI."""
            etype = event.get("type", "")
            phase_id = f"phase{event.get('phase', 0)}"
            if etype == "phase_start":
                ui.start_phase(phase_id)
            elif etype == "phase_complete":
                ui.complete_phase(phase_id, event.get("success", True))
            elif etype == "phase_skip":
                ui.skip_phase(phase_id)
            elif etype == "package_start":
                ui.add_package(phase_id, event.get("package", ""))
            elif etype == "package_complete":
                ui.complete_package(phase_id, event.get("package", ""), event.get("success", True))
            # build_start / build_complete / log carry no extra UI state:
            # phase events already cover the display.

        event_position = 0

        def drain_events() -> None:
            """Consume all complete event lines currently in the file."""
            nonlocal event_position
            events, event_position = read_new_events(event_file, event_position)
            for event in events:
                handle_event(event)
            if events:
                ui.refresh()

        def tail_events() -> None:
            """Background thread to tail event file and update UI."""
            while not stop_event.is_set():
                drain_events()
                time.sleep(0.1)

        def periodic_refresh() -> None:
            """Background thread to periodically refresh UI for elapsed time updates."""
            while not stop_event.is_set():
                ui.refresh()
                time.sleep(0.25)  # Refresh 4 times per second

        # Start event tailer thread
        tailer = threading.Thread(target=tail_events, daemon=True)
        tailer.start()

        # Start periodic refresh thread for elapsed time updates
        refresher = threading.Thread(target=periodic_refresh, daemon=True)
        refresher.start()

        # Start the container with stdout/stderr piped
        process = subprocess.Popen(
            docker_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,  # Line buffered
        )

        # Read container output and add to log panel
        with ui.live_context():
            if process.stdout:
                for line in process.stdout:
                    # Strip ANSI escape codes before displaying
                    clean_line = _ANSI_ESCAPE_RE.sub("", line.rstrip())
                    ui.update_log(clean_line)
                    ui.refresh()

            process.wait()

            stop_event.set()
            tailer.join(timeout=1)
            refresher.join(timeout=1)

            # Final drain: pick up events written after the tailer's last
            # poll (typically phase 8 completion and build_complete).
            drain_events()
            ui.refresh()

        return process.returncode

    except Exception as e:
        print(f"Error running container: {e}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, original_handler)
        with _interrupt_lock:
            _container_id = None
        # Never leave the container building in the background if the
        # display loop died (e.g. a Rich rendering error).
        if process is not None and process.poll() is None:
            stop_container(container_name, force=True)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()


class _RunCommandResult:
    """Result-like object for streamed command output."""

    def __init__(self, returncode: int, stdout: str, stderr: str) -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def run_command(
    cmd: list[str],
    check: bool = True,
    log_file: Path | None = None,
    stream_output: bool = False,
) -> subprocess.CompletedProcess[str] | _RunCommandResult:
    """Run a shell command and return the result.

    Args:
        cmd: Command to run as a list
        check: Raise exception on non-zero exit code
        log_file: Path to log file for output
        stream_output: If True, stream output to terminal in real-time
    """
    if stream_output:
        # Stream output in real-time while also logging
        import contextlib

        with contextlib.ExitStack() as stack:
            log_handle = stack.enter_context(open(log_file, "w")) if log_file else None
            if log_handle:
                log_handle.write(f"Command: {' '.join(cmd)}\n")
                log_handle.write("=" * 80 + "\n\n")

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            stdout_lines: list[str] = []
            if process.stdout:
                for line in process.stdout:
                    line = line.rstrip()
                    stdout_lines.append(line)
                    if line:
                        print(f"  {line}")
                    if log_handle:
                        log_handle.write(line + "\n")
                        log_handle.flush()

            process.wait()

            if process.returncode != 0:
                print("  ✗ Failed", file=sys.stderr)
                if check:
                    raise subprocess.CalledProcessError(
                        process.returncode, cmd, "\n".join(stdout_lines), ""
                    )
            else:
                print("  ✓ Done")

            return _RunCommandResult(process.returncode, "\n".join(stdout_lines), "")
    elif log_file:
        # Run without check so we can log output before raising exception
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)

        with open(log_file, "w") as log:
            log.write(f"Command: {' '.join(cmd)}\n")
            log.write("=" * 80 + "\n\n")
            if result.stdout:
                log.write(result.stdout)
            if result.stderr:
                log.write("\n" + "=" * 80 + "\n")
                log.write("STDERR:\n")
                log.write(result.stderr)

        # Show summary on error
        if result.returncode != 0:
            print(f"  ✗ Failed (see {log_file})", file=sys.stderr)
            if check:
                raise subprocess.CalledProcessError(
                    result.returncode, cmd, result.stdout, result.stderr
                )
        else:
            print("  ✓ Done")

        return result
    else:
        # No log file - show everything as before
        result = subprocess.run(cmd, check=check, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        return result


def download_dockerfile(
    url: str, cache_dir: str | Path | None = None, refresh: bool = False
) -> Path:
    """Download Dockerfile from HTTP/HTTPS URL.

    Downloads are cached by URL; pass refresh=True to bypass the cache
    and re-download (the CLI exposes this as --refresh-dockerfile).
    """
    cached_file: Path | None = None

    # Create cache directory if specified
    if cache_dir:
        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)

        # Generate cache filename based on URL hash
        url_hash = hashlib.sha256(url.encode()).hexdigest()[:12]
        cached_file = cache_dir / f"Dockerfile.{url_hash}"

        # Check if cached file exists
        if cached_file.exists() and not refresh:
            print(
                f"Using cached Dockerfile for {url}\n"
                f"  (cached at {cached_file}; pass --refresh-dockerfile to re-download)"
            )
            return cached_file

    try:
        # Download the file
        request = urllib.request.Request(url, headers={"User-Agent": "colcon2deb/1.0"})
        with urllib.request.urlopen(request, timeout=30) as response:
            content = response.read()

        # Validate it looks like a Dockerfile
        content_str = content.decode("utf-8", errors="ignore")
        if not ("FROM" in content_str or "ARG" in content_str):
            print(
                "  Warning: Downloaded content may not be a valid Dockerfile",
                file=sys.stderr,
            )

        # Save to temporary file or cache
        if cache_dir and cached_file:
            cached_file.write_bytes(content)
            return cached_file
        else:
            # Create temporary file
            with tempfile.NamedTemporaryFile(
                mode="wb", suffix=".Dockerfile", delete=False
            ) as tmp_file:
                tmp_file.write(content)
                temp_path = Path(tmp_file.name)
                print("  ✓ Saved to temporary location")
                return temp_path

    except urllib.error.HTTPError as e:
        print(f"\n  ✗ HTTP Error {e.code}: {e.reason}", file=sys.stderr)
        print(f"    URL: {url}", file=sys.stderr)
        sys.exit(1)
    except urllib.error.URLError as e:
        print(f"\n  ✗ Network Error: {e.reason}", file=sys.stderr)
        print("    Please check your internet connection and the URL", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\n  ✗ Download failed: {e}", file=sys.stderr)
        sys.exit(1)


def build_image_from_dockerfile(
    dockerfile_path: str | Path,
    image_name: str,
    build_context: str | Path | None = None,
    log_dir: str | Path | None = None,
    platform: str | None = None,
) -> str:
    """Build Docker image from Dockerfile."""
    dockerfile_path = Path(dockerfile_path).resolve()
    if not dockerfile_path.exists():
        print(f"Error: Dockerfile not found at {dockerfile_path}", file=sys.stderr)
        sys.exit(1)

    # Use provided build context or default to parent directory of Dockerfile
    if build_context:
        build_context = Path(build_context).resolve()
    else:
        build_context = dockerfile_path.parent

    cmd = [
        "docker",
        "build",
    ]
    # Add platform flag for cross-compilation (e.g., building arm64 on amd64 host)
    if platform:
        cmd.extend(["--platform", platform])
    cmd.extend(
        [
            str(build_context),
            "-f",
            str(dockerfile_path),
            "-t",
            image_name,
        ]
    )

    print(f"Building Docker image '{image_name}'...")

    # Log docker build output to file if log_dir provided
    log_file = None
    if log_dir:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / "docker_build.log"

    try:
        run_command(cmd, log_file=log_file, stream_output=True)
    except FileNotFoundError:
        print(
            "Error: 'docker' command not found. Install Docker and ensure it is on PATH.",
            file=sys.stderr,
        )
        sys.exit(1)
    except subprocess.CalledProcessError:
        where = f" (see {log_file})" if log_file else ""
        print(f"Error: Docker image build failed{where}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted during Docker image build", file=sys.stderr)
        sys.exit(130)
    return image_name


def main():
    parser = argparse.ArgumentParser(description="Build Debian packages from colcon workspace")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")

    # Workspace directory
    parser.add_argument(
        "--workspace",
        required=True,
        help="Path to colcon workspace directory",
    )

    # Config file
    parser.add_argument(
        "--config",
        required=True,
        help="Path to configuration YAML file",
    )

    parser.add_argument(
        "--refresh-dockerfile",
        action="store_true",
        help="Re-download a remote Dockerfile instead of using the cached copy",
    )

    # Skip options for incremental builds
    parser.add_argument(
        "--skip-rosdep-install",
        action="store_true",
        help="Skip rosdep install step (Phase 3)",
    )
    parser.add_argument(
        "--skip-copy-src",
        action="store_true",
        help="Skip copying source files (Phase 2)",
    )
    parser.add_argument(
        "--skip-gen-rosdep-list",
        action="store_true",
        help="Skip generating rosdep list (Phase 5)",
    )
    parser.add_argument(
        "--skip-colcon-build",
        action="store_true",
        help="Skip colcon build step (Phase 4)",
    )
    parser.add_argument(
        "--skip-gen-debian",
        action="store_true",
        help="Skip generating Debian metadata (Phase 7)",
    )
    parser.add_argument(
        "--skip-build-deb",
        action="store_true",
        help="Skip building .deb packages (Phase 8)",
    )

    # Parse arguments
    args = parser.parse_args()

    # Load and validate configuration
    try:
        raw_config = load_config_file(args.config)
        cfg = validate_config(raw_config, Path(args.config).resolve().parent)
    except ConfigError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    for warning in cfg.warnings:
        print(f"Warning: {warning}", file=sys.stderr)

    # Validate all paths before any expensive work (Docker builds can take
    # a long time; a typo'd path must fail immediately).
    docker_error = check_docker_prereqs()
    if docker_error is not None:
        print(f"Error: {docker_error}", file=sys.stderr)
        sys.exit(1)

    workspace_dir = Path(args.workspace).resolve()
    if not workspace_dir.is_dir():
        print(f"Error: Workspace directory not found at {workspace_dir}", file=sys.stderr)
        sys.exit(1)

    packages_dir = cfg.packages_dir
    if not packages_dir.is_dir():
        print(
            f"Error: Packages config directory not found at {packages_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get the script directory (where this script is located)
    script_dir = Path(__file__).resolve().parent

    # Helper directory is inside the colcon2deb package
    helper_dir = script_dir / "helper"
    if not helper_dir.exists():
        print("Error: Helper scripts directory not found", file=sys.stderr)
        print(f"Expected at: {helper_dir}", file=sys.stderr)
        print(
            "Helper directory must be inside the colcon2deb package",
            file=sys.stderr,
        )
        sys.exit(1)

    # rosdeb-bloom is the vendored debian generator library inside colcon2deb
    rosdeb_bloom_dir = script_dir / "rosdeb-bloom"
    if not rosdeb_bloom_dir.exists():
        print("Error: rosdeb-bloom directory not found", file=sys.stderr)
        print(f"Expected at: {rosdeb_bloom_dir}", file=sys.stderr)
        sys.exit(1)

    output_dir = cfg.output_dir

    # Create output directory and timestamped log directory
    output_dir.mkdir(parents=True, exist_ok=True)
    log_base_dir = output_dir / "logs"
    log_base_dir.mkdir(parents=True, exist_ok=True)

    # Create timestamped log directory (with microseconds to avoid collisions)
    from datetime import datetime

    now = datetime.now()
    log_timestamp = now.strftime("%Y-%m-%d_%H-%M-%S.%f")
    log_dir = log_base_dir / log_timestamp
    log_dir.mkdir(parents=True, exist_ok=True)

    # Create log subdirectories (new structure: phases/, packages/, reports/, scripts/)
    log_phases_dir = log_dir / "phases"
    log_packages_dir = log_dir / "packages"
    log_reports_dir = log_dir / "reports"
    log_scripts_dir = log_dir / "scripts"
    log_phases_dir.mkdir(parents=True, exist_ok=True)
    log_packages_dir.mkdir(parents=True, exist_ok=True)
    log_reports_dir.mkdir(parents=True, exist_ok=True)
    log_scripts_dir.mkdir(parents=True, exist_ok=True)

    # Create/update 'latest' symlink
    latest_link = log_base_dir / "latest"
    if latest_link.is_symlink():
        latest_link.unlink()
    if latest_link.exists():
        print(
            f"Warning: {latest_link} exists and is not a symlink; not updating it",
            file=sys.stderr,
        )
    else:
        latest_link.symlink_to(log_timestamp)

    # Determine the image to use
    config_dir_path = Path(args.config).resolve().parent
    if cfg.dockerfile is not None:
        dockerfile_value = cfg.dockerfile

        # Check if it's a URL
        if dockerfile_value.startswith(("http://", "https://")):
            # Download Dockerfile from URL
            # Use cache directory in user's home or temp
            cache_dir = Path.home() / ".cache" / "colcon2deb" / "dockerfiles"
            dockerfile_path = download_dockerfile(
                dockerfile_value, cache_dir, refresh=args.refresh_dockerfile
            )

            # For remote Dockerfiles, use a minimal build context
            with tempfile.TemporaryDirectory(prefix="colcon2deb_context_") as temp_context:
                print("\nPreparing build context...")

                temp_dockerfile = Path(temp_context) / "Dockerfile"
                temp_dockerfile.write_bytes(dockerfile_path.read_bytes())

                image_name = build_image_from_dockerfile(
                    temp_dockerfile,
                    cfg.image_name,
                    build_context=temp_context,
                    log_dir=log_dir,
                    platform=cfg.platform,
                )
        else:
            # Local Dockerfile path, relative to the config file
            dockerfile_path = Path(dockerfile_value)
            if not dockerfile_path.is_absolute():
                dockerfile_path = config_dir_path / dockerfile_path
            dockerfile_path = dockerfile_path.resolve()

            # Check if a build context is specified
            build_context: str | Path | None = cfg.build_context
            if build_context and not Path(build_context).is_absolute():
                build_context = config_dir_path / build_context

            image_name = build_image_from_dockerfile(
                dockerfile_path,
                cfg.image_name,
                build_context=build_context,
                log_dir=log_dir,
                platform=cfg.platform,
            )
    else:
        assert cfg.image is not None
        image_name = cfg.image

    # Capture Docker image ID for fingerprinting
    try:
        img_result = subprocess.run(
            ["docker", "inspect", "--format", "{{.Id}}", image_name],
            capture_output=True,
            text=True,
            timeout=10,
        )
        docker_image_id = img_result.stdout.strip() if img_result.returncode == 0 else "unknown"
    except Exception:
        docker_image_id = "unknown"

    # Get current user/group IDs
    uid = os.getuid()
    gid = os.getgid()

    # Build settings (validated earlier)
    ros_distro = cfg.ros_distro
    install_prefix = cfg.install_prefix
    package_suffix = cfg.package_suffix
    parallel_jobs = cfg.parallel_jobs

    # Generate unique container name for this build run
    run_id = uuid.uuid4().hex[:8]
    container_name = f"colcon2deb-{run_id}"

    # Prepare Docker run command
    docker_cmd = [
        "docker",
        "run",
        "--rm",
        f"--name={container_name}",
        "--net=host",
        "-e",
        f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-e",
        f"ROS_DISTRO={ros_distro}",
        "-e",
        f"ROS_INSTALL_PREFIX={install_prefix}",
        "-e",
        f"ROS_PACKAGE_SUFFIX={package_suffix or ''}",
        "-e",
        f"COLCON2DEB_PARALLEL_JOBS={parallel_jobs}",
        "-e",
        f"COLCON2DEB_PIPELINE={'1' if cfg.pipeline else '0'}",
        "-e",
        f"COLCON2DEB_SKIP_TESTS={'1' if cfg.skip_tests else '0'}",
        "-e",
        f"COLCON2DEB_VERSION={__version__}",
        "-e",
        f"COLCON2DEB_IMAGE_ID={docker_image_id}",
        "-v",
        "/tmp/.X11-unix/:/tmp/.X11-unix",
        "-v",
        f"{workspace_dir}:/workspace",
        "-v",
        f"{packages_dir}:/config",
        "-v",
        f"{helper_dir}:/helper",
        "-v",
        f"{output_dir}:/output",
        "-v",
        f"{rosdeb_bloom_dir}:/rosdeb-bloom:ro",
    ]

    docker_cmd.extend(
        [
            image_name,
            "/helper/entry.sh",
            f"--uid={uid}",
            f"--gid={gid}",
            "--output=/output",
            f"--log-dir=/output/logs/{log_timestamp}",
        ]
    )

    # Add skip options if specified
    if args.skip_rosdep_install:
        docker_cmd.append("--skip-rosdep-install")
    if args.skip_copy_src:
        docker_cmd.append("--skip-copy-src")
    if args.skip_gen_rosdep_list:
        docker_cmd.append("--skip-gen-rosdep-list")
    if args.skip_colcon_build:
        docker_cmd.append("--skip-colcon-build")
    if args.skip_gen_debian:
        docker_cmd.append("--skip-gen-debian")
    if args.skip_build_deb:
        docker_cmd.append("--skip-build-deb")

    # Add nvidia runtime if available and requested in config
    if cfg.use_nvidia_runtime:
        docker_cmd.insert(4, "--runtime")
        docker_cmd.insert(5, "nvidia")

    # Run the container with TUI display
    print()  # Blank line before TUI
    return_code = run_container_with_tui(docker_cmd, container_name, output_dir)

    if return_code != 0:
        print(f"\nContainer exited with code {return_code}", file=sys.stderr)
        sys.exit(return_code)


if __name__ == "__main__":
    main()
