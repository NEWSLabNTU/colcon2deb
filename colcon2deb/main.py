#!/usr/bin/env python3
"""Docker run script replacement for make run with enhanced functionality."""

import argparse
import atexit
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
from pathlib import Path

import yaml

from .events import EVENT_FILE
from .ui import BuildUI

# Regex to strip ANSI escape codes
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")

# Global state for signal handling
_container_id = None
_interrupt_count = 0
_interrupt_lock = threading.Lock()


def stop_container(container_id, force=False):
    """Stop a Docker container."""
    if not container_id:
        return

    try:
        if force:
            print(f"\nForce killing container {container_id[:12]}...")
            subprocess.run(["docker", "kill", container_id], capture_output=True, timeout=10)
        else:
            print(f"\nStopping container {container_id[:12]} (press Ctrl+C again to force)...")
            subprocess.run(
                ["docker", "stop", "-t", "10", container_id], capture_output=True, timeout=20
            )
    except subprocess.TimeoutExpired:
        # If stop times out, try kill
        try:
            subprocess.run(["docker", "kill", container_id], capture_output=True, timeout=5)
        except Exception:
            pass
    except Exception as e:
        print(f"Warning: Failed to stop container: {e}", file=sys.stderr)


def signal_handler(signum, frame):
    """Handle interrupt signals with escalating force."""
    global _interrupt_count, _container_id

    with _interrupt_lock:
        _interrupt_count += 1
        count = _interrupt_count

    if count == 1:
        print("\n\nReceived interrupt signal. Stopping container gracefully...")
        print("Press Ctrl+C again to force stop, or 3 times to force kill immediately.")
        # Try to find and stop the container
        if _container_id:
            stop_container(_container_id, force=False)
    elif count == 2:
        print("\n\nReceived second interrupt. Force stopping...")
        if _container_id:
            stop_container(_container_id, force=True)
    else:
        print("\n\nReceived third interrupt. Forcing immediate exit...")
        if _container_id:
            stop_container(_container_id, force=True)
        sys.exit(130)  # 128 + SIGINT


def run_container_with_signal_handling(docker_cmd, image_name):
    """Run a Docker container with proper signal handling."""
    global _container_id, _interrupt_count

    # Reset interrupt count
    _interrupt_count = 0

    # Set up signal handler
    original_handler = signal.signal(signal.SIGINT, signal_handler)

    try:
        # Start the container
        process = subprocess.Popen(docker_cmd)

        # Try to get container ID (with retries)
        for _ in range(10):
            time.sleep(0.5)
            # Try to find container by image name
            try:
                result = subprocess.run(
                    [
                        "docker",
                        "ps",
                        "-q",
                        "--filter",
                        f"ancestor={image_name}",
                        "--filter",
                        "status=running",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if result.returncode == 0 and result.stdout.strip():
                    _container_id = result.stdout.strip().split("\n")[0]
                    break
            except Exception:
                pass

            # Check if process already finished
            if process.poll() is not None:
                break

        # Wait for the process to complete
        return_code = process.wait()
        return return_code

    except Exception as e:
        print(f"Error running container: {e}", file=sys.stderr)
        return 1
    finally:
        # Restore original signal handler
        signal.signal(signal.SIGINT, original_handler)
        _container_id = None


def run_container_with_tui(docker_cmd, image_name, output_dir):
    """Run a Docker container with TUI display.

    Args:
        docker_cmd: Docker command to run
        image_name: Name of the Docker image (for finding container ID)
        output_dir: Output directory where events file will be written

    Returns:
        Container exit code
    """
    global _container_id, _interrupt_count

    # Reset interrupt count
    _interrupt_count = 0

    # Set up signal handler
    original_handler = signal.signal(signal.SIGINT, signal_handler)

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

    def handle_event(event: dict) -> None:
        """Process an event and update UI."""
        etype = event.get("type", "")
        if etype == "phase_start":
            phase_id = f"phase{event.get('phase', 0)}"
            ui.start_phase(phase_id)
        elif etype == "phase_complete":
            phase_id = f"phase{event.get('phase', 0)}"
            ui.complete_phase(phase_id, event.get("success", True))
        elif etype == "phase_skip":
            phase_id = f"phase{event.get('phase', 0)}"
            ui.skip_phase(phase_id)
        elif etype == "package_start":
            phase_id = f"phase{event.get('phase', 0)}"
            ui.add_package(phase_id, event.get("package", ""))
        elif etype == "package_complete":
            phase_id = f"phase{event.get('phase', 0)}"
            ui.complete_package(phase_id, event.get("package", ""), event.get("success", True))

    def tail_events() -> None:
        """Background thread to tail event file and update UI."""
        position = 0
        while not stop_event.is_set():
            if event_file.exists():
                try:
                    with open(event_file) as f:
                        f.seek(position)
                        for line in f:
                            line = line.strip()
                            if line:
                                try:
                                    event = json.loads(line)
                                    handle_event(event)
                                    ui.refresh()
                                except json.JSONDecodeError:
                                    pass
                        position = f.tell()
                except OSError:
                    pass
            time.sleep(0.1)

    def periodic_refresh() -> None:
        """Background thread to periodically refresh UI for elapsed time updates."""
        while not stop_event.is_set():
            ui.refresh()
            time.sleep(0.25)  # Refresh 4 times per second

    try:
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
            bufsize=1,  # Line buffered
        )

        # Try to get container ID (with retries)
        for _ in range(10):
            time.sleep(0.5)
            try:
                result = subprocess.run(
                    [
                        "docker",
                        "ps",
                        "-q",
                        "--filter",
                        f"ancestor={image_name}",
                        "--filter",
                        "status=running",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if result.returncode == 0 and result.stdout.strip():
                    _container_id = result.stdout.strip().split("\n")[0]
                    break
            except Exception:
                pass

            if process.poll() is not None:
                break

        # Read container output and add to log panel
        with ui.live_context():
            for line in process.stdout:
                # Strip ANSI escape codes before displaying
                clean_line = _ANSI_ESCAPE_RE.sub("", line.rstrip())
                ui.update_log(clean_line)
                ui.refresh()

            process.wait()

        stop_event.set()
        tailer.join(timeout=1)
        refresher.join(timeout=1)

        return process.returncode

    except Exception as e:
        print(f"Error running container: {e}", file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        signal.signal(signal.SIGINT, original_handler)
        _container_id = None


def run_command(cmd, check=True, log_file=None, stream_output=False):
    """Run a shell command and return the result.

    Args:
        cmd: Command to run as a list
        check: Raise exception on non-zero exit code
        log_file: Path to log file for output
        stream_output: If True, stream output to terminal in real-time
    """
    if stream_output:
        # Stream output in real-time while also logging
        log_handle = open(log_file, "w") if log_file else None
        try:
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

            stdout_lines = []
            for line in process.stdout:
                line = line.rstrip()
                stdout_lines.append(line)
                # Filter and display Docker build progress
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

            # Return a result-like object
            class Result:
                def __init__(self, returncode, stdout, stderr):
                    self.returncode = returncode
                    self.stdout = stdout
                    self.stderr = stderr

            return Result(process.returncode, "\n".join(stdout_lines), "")
        finally:
            if log_handle:
                log_handle.close()
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


def download_dockerfile(url, cache_dir=None):
    """Download Dockerfile from HTTP/HTTPS URL."""
    # Create cache directory if specified
    if cache_dir:
        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)

        # Generate cache filename based on URL hash
        url_hash = hashlib.md5(url.encode()).hexdigest()[:8]
        cached_file = cache_dir / f"Dockerfile.{url_hash}"

        # Check if cached file exists
        if cached_file.exists():
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
                "  ⚠️  Warning: Downloaded content may not be a valid Dockerfile",
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
        print(f"\n❌ HTTP Error {e.code}: {e.reason}", file=sys.stderr)
        print(f"   URL: {url}", file=sys.stderr)
        sys.exit(1)
    except urllib.error.URLError as e:
        print(f"\n❌ Network Error: {e.reason}", file=sys.stderr)
        print("   Please check your internet connection and the URL", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)


def build_image_from_dockerfile(
    dockerfile_path, image_name, build_context=None, log_dir=None, platform=None
):
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

    run_command(cmd, log_file=log_file, stream_output=True)
    return image_name


def load_config(config_path):
    """Load configuration from YAML file."""
    config_path = Path(config_path).resolve()
    if not config_path.exists():
        print(f"Error: Config file not found at {config_path}", file=sys.stderr)
        sys.exit(1)

    with open(config_path) as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing config file: {e}", file=sys.stderr)
            sys.exit(1)

    return config


def main():
    parser = argparse.ArgumentParser(description="Build Debian packages from colcon workspace")

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

    # Load configuration
    config = load_config(args.config)

    # Validate config version
    if config.get("version") != 1:
        print(
            f"Error: Unsupported config version: {config.get('version')}",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get docker configuration
    docker_config = config.get("docker", {})
    if "image" in docker_config and "dockerfile" in docker_config:
        print(
            "Error: Cannot specify both 'image' and 'dockerfile' in config",
            file=sys.stderr,
        )
        sys.exit(1)

    if "image" not in docker_config and "dockerfile" not in docker_config:
        print(
            "Error: Must specify either 'image' or 'dockerfile' in config",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get output directory early for logging
    output_config = config.get("output", {})
    if "directory" not in output_config:
        print("Error: 'output.directory' not specified in config", file=sys.stderr)
        sys.exit(1)

    output_dir = Path(output_config["directory"])
    if not output_dir.is_absolute():
        # Relative to config file
        output_dir = Path(args.config).parent / output_dir

    # Ensure absolute path for Docker
    output_dir = output_dir.resolve()

    # Create output directory and timestamped log directory
    output_dir.mkdir(parents=True, exist_ok=True)
    log_base_dir = output_dir / "logs"
    log_base_dir.mkdir(parents=True, exist_ok=True)

    # Create timestamped log directory
    from datetime import datetime

    log_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = log_base_dir / log_timestamp
    log_dir.mkdir(parents=True, exist_ok=True)

    # Create log subdirectories
    log_logs_dir = log_dir / "logs"
    log_reports_dir = log_dir / "reports"
    log_scripts_dir = log_dir / "scripts"
    log_logs_dir.mkdir(parents=True, exist_ok=True)
    log_reports_dir.mkdir(parents=True, exist_ok=True)
    log_scripts_dir.mkdir(parents=True, exist_ok=True)

    # Create/update 'latest' symlink
    latest_link = log_base_dir / "latest"
    if latest_link.is_symlink() or latest_link.exists():
        latest_link.unlink()
    latest_link.symlink_to(log_timestamp)

    # Determine the image to use
    if "dockerfile" in docker_config:
        dockerfile_value = docker_config["dockerfile"]

        # Check if it's a URL
        if dockerfile_value.startswith(("http://", "https://")):
            # Download Dockerfile from URL
            # Use cache directory in user's home or temp
            cache_dir = Path.home() / ".cache" / "colcon2deb" / "dockerfiles"
            dockerfile_path = download_dockerfile(dockerfile_value, cache_dir)

            # For remote Dockerfiles, use a minimal build context
            # Create a temporary directory with just the Dockerfile
            temp_context = tempfile.mkdtemp(prefix="colcon2deb_context_")

            print("\nPreparing build context...")
            print(f"  Temporary context: {temp_context}")

            # Register cleanup function to remove temp directory
            def cleanup_temp_context():
                if Path(temp_context).exists():
                    shutil.rmtree(temp_context, ignore_errors=True)

            atexit.register(cleanup_temp_context)

            temp_dockerfile = Path(temp_context) / "Dockerfile"
            temp_dockerfile.write_bytes(dockerfile_path.read_bytes())

            image_name = build_image_from_dockerfile(
                temp_dockerfile,
                docker_config.get("image_name", "colcon2deb_builder"),
                build_context=temp_context,
                log_dir=log_logs_dir,
                platform=docker_config.get("platform"),
            )
        else:
            # Local Dockerfile path
            dockerfile_path = Path(dockerfile_value)
            if not dockerfile_path.is_absolute():
                # Relative to config file
                dockerfile_path = Path(args.config).parent / dockerfile_path
            # Ensure absolute path
            dockerfile_path = dockerfile_path.resolve()

            # Check if a build context is specified
            build_context = docker_config.get("build_context")
            if build_context:
                if not Path(build_context).is_absolute():
                    build_context = Path(args.config).parent / build_context

            image_name = build_image_from_dockerfile(
                dockerfile_path,
                docker_config.get("image_name", "colcon2deb_builder"),
                build_context=build_context,
                log_dir=log_logs_dir,
                platform=docker_config.get("platform"),
            )
    else:
        image_name = docker_config["image"]

    # Verify workspace directory exists
    workspace_dir = Path(args.workspace).resolve()
    if not workspace_dir.exists():
        print(f"Error: Workspace directory not found at {workspace_dir}", file=sys.stderr)
        sys.exit(1)

    # Get current user/group IDs
    uid = os.getuid()
    gid = os.getgid()

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

    # Get packages configuration directory
    packages_config = config.get("packages", {})
    if "directory" not in packages_config:
        print("Error: 'packages.directory' not specified in config", file=sys.stderr)
        sys.exit(1)

    packages_dir = Path(packages_config["directory"])
    if not packages_dir.is_absolute():
        # Relative to config file
        packages_dir = Path(args.config).parent / packages_dir

    # Ensure absolute path for Docker
    packages_dir = packages_dir.resolve()

    if not packages_dir.exists():
        print(
            f"Error: Packages config directory not found at {packages_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    # output_dir was already set up earlier for logging
    # The build_deb directory will be created inside the output directory
    # Users will find packages in output_dir/dist/

    # Get build configuration
    build_config = config.get("build", {})
    ros_distro = build_config.get("ros_distro", "humble")

    # Get install prefix configuration (optional)
    # Default: /opt/ros/{ros_distro}
    install_prefix = build_config.get("install_prefix", f"/opt/ros/{ros_distro}")

    # Get package suffix configuration (optional)
    # Example: "1.5.0" results in packages like ros-humble-autoware-utils-1.5.0
    package_suffix = build_config.get("package_suffix")

    # rosdeb-bloom is the vendored debian generator library inside colcon2deb
    rosdeb_bloom_dir = script_dir / "rosdeb-bloom"
    if not rosdeb_bloom_dir.exists():
        print("Error: rosdeb-bloom directory not found", file=sys.stderr)
        print(f"Expected at: {rosdeb_bloom_dir}", file=sys.stderr)
        sys.exit(1)

    # Prepare Docker run command
    docker_cmd = [
        "docker",
        "run",
        "--rm",
        "--net=host",
        "-e",
        f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-e",
        f"ROS_DISTRO={ros_distro}",
        "-e",
        f"ROS_INSTALL_PREFIX={install_prefix}",
        "-e",
        f"ROS_PACKAGE_SUFFIX={package_suffix or ''}",
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
    use_nvidia = build_config.get("use_nvidia_runtime", False)
    if use_nvidia:
        docker_cmd.insert(4, "--runtime")
        docker_cmd.insert(5, "nvidia")

    # Run the container with TUI display
    print()  # Blank line before TUI
    return_code = run_container_with_tui(docker_cmd, image_name, output_dir)

    if return_code != 0:
        print(f"\nContainer exited with code {return_code}", file=sys.stderr)
        sys.exit(return_code)


if __name__ == "__main__":
    main()
