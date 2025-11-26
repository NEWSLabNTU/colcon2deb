#!/usr/bin/env python3
"""Docker run script replacement for make run with enhanced functionality."""

import argparse
import os
import subprocess
import sys
import yaml
import tempfile
import urllib.request
import urllib.error
import hashlib
import shutil
import atexit
from pathlib import Path


# ANSI color codes
class Colors:
    """ANSI color codes for terminal output."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    RED = '\033[31m'
    CYAN = '\033[36m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_YELLOW = '\033[93m'


def run_command(cmd, check=True, log_file=None, show_output=False):
    """Run a shell command and return the result.

    Args:
        cmd: Command to run as a list
        check: Raise exception on non-zero exit code
        log_file: Path to log file for output (if None, output goes to terminal)
        show_output: If True, show output even when log_file is set
    """
    if not log_file:
        # No log file - show everything as before
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, check=check, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        return result
    else:
        # Log to file, optionally show progress
        if show_output:
            print(f"Running: {' '.join(cmd)}")
            print(f"  Output logged to: {log_file}")

        with open(log_file, 'w') as log:
            log.write(f"Command: {' '.join(cmd)}\n")
            log.write("=" * 80 + "\n\n")
            result = subprocess.run(cmd, check=check, capture_output=True, text=True)
            if result.stdout:
                log.write(result.stdout)
            if result.stderr:
                log.write("\n" + "=" * 80 + "\n")
                log.write("STDERR:\n")
                log.write(result.stderr)

        # Show summary on error
        if result.returncode != 0 and not show_output:
            print(f"✗ Command failed with exit code {result.returncode}", file=sys.stderr)
            print(f"  See log: {log_file}", file=sys.stderr)
        elif show_output:
            print(f"  ✓ Complete")

        return result


def download_dockerfile(url, cache_dir=None, verbose=False):
    """Download Dockerfile from HTTP/HTTPS URL."""
    if verbose:
        print(f"\nDownloading Dockerfile from URL...")
        print(f"  URL: {url}")

    # Create cache directory if specified
    if cache_dir:
        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)

        # Generate cache filename based on URL hash
        url_hash = hashlib.md5(url.encode()).hexdigest()[:8]
        cached_file = cache_dir / f"Dockerfile.{url_hash}"

        # Check if cached file exists
        if cached_file.exists():
            if verbose:
                print(f"  ✓ Using cached Dockerfile")
                print(f"  Cache location: {cached_file}")
            return cached_file

    try:
        if verbose:
            print(f"  Fetching from remote...")
        # Download the file
        request = urllib.request.Request(url, headers={"User-Agent": "colcon2deb/1.0"})
        with urllib.request.urlopen(request, timeout=30) as response:
            content = response.read()
            content_length = len(content)

        # Validate it looks like a Dockerfile
        content_str = content.decode("utf-8", errors="ignore")
        if not ("FROM" in content_str or "ARG" in content_str):
            print(
                f"  ⚠️  Warning: Downloaded content may not be a valid Dockerfile",
                file=sys.stderr,
            )

        if verbose:
            print(f"  ✓ Downloaded {content_length} bytes")

        # Save to temporary file or cache
        if cache_dir and cached_file:
            cached_file.write_bytes(content)
            if verbose:
                print(f"  ✓ Cached for future use")
                print(f"  Cache location: {cached_file}")
            return cached_file
        else:
            # Create temporary file
            with tempfile.NamedTemporaryFile(
                mode="wb", suffix=".Dockerfile", delete=False
            ) as tmp_file:
                tmp_file.write(content)
                temp_path = Path(tmp_file.name)
                if verbose:
                    print(f"  ✓ Saved to temporary location")
                return temp_path

    except urllib.error.HTTPError as e:
        print(f"\n❌ HTTP Error {e.code}: {e.reason}", file=sys.stderr)
        print(f"   URL: {url}", file=sys.stderr)
        sys.exit(1)
    except urllib.error.URLError as e:
        print(f"\n❌ Network Error: {e.reason}", file=sys.stderr)
        print(f"   Please check your internet connection and the URL", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)


def build_image_from_dockerfile(dockerfile_path, image_name, build_context=None, log_dir=None, verbose=False):
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
        str(build_context),
        "-f",
        str(dockerfile_path),
        "-t",
        image_name,
    ]

    if verbose:
        print(f"Building Docker image '{image_name}'...")
        print(f"  Dockerfile: {dockerfile_path}")
        print(f"  Build context: {build_context}")

    # Log docker build output to file if log_dir provided
    log_file = None
    if log_dir:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / "docker_build.log"

    run_command(cmd, log_file=log_file, show_output=verbose)
    return image_name


def load_config(config_path):
    """Load configuration from YAML file."""
    config_path = Path(config_path).resolve()
    if not config_path.exists():
        print(f"Error: Config file not found at {config_path}", file=sys.stderr)
        sys.exit(1)

    with open(config_path, "r") as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing config file: {e}", file=sys.stderr)
            sys.exit(1)

    return config


def main():
    parser = argparse.ArgumentParser(
        description="Build Debian packages from colcon workspace"
    )

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

    # Verbose flag
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show verbose output including Docker build details",
    )

    # Parse arguments
    args = parser.parse_args()
    verbose = args.verbose

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

    # Create output directory and log directory if they don't exist
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir = output_dir / "log"
    log_dir.mkdir(parents=True, exist_ok=True)

    # Determine the image to use
    if "dockerfile" in docker_config:
        dockerfile_value = docker_config["dockerfile"]

        # Check if it's a URL
        if dockerfile_value.startswith(("http://", "https://")):
            if verbose:
                print("\n" + "=" * 60)
                print("Remote Dockerfile Configuration Detected")
                print("=" * 60)

            # Download Dockerfile from URL
            # Use cache directory in user's home or temp
            cache_dir = Path.home() / ".cache" / "colcon2deb" / "dockerfiles"
            dockerfile_path = download_dockerfile(dockerfile_value, cache_dir, verbose=verbose)

            # For remote Dockerfiles, use a minimal build context
            # Create a temporary directory with just the Dockerfile
            temp_context = tempfile.mkdtemp(prefix="colcon2deb_context_")

            if verbose:
                print(f"\nPreparing build context...")
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
                log_dir=log_dir,
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
                log_dir=log_dir,
                verbose=verbose,
            )
    else:
        image_name = docker_config["image"]

    # Verify workspace directory exists
    workspace_dir = Path(args.workspace).resolve()
    if not workspace_dir.exists():
        print(
            f"Error: Workspace directory not found at {workspace_dir}", file=sys.stderr
        )
        sys.exit(1)

    # Get current user/group IDs
    uid = os.getuid()
    gid = os.getgid()

    # Get the script directory (where this script is located)
    script_dir = Path(__file__).resolve().parent

    # Determine the colcon2deb source directory and installation mode
    # Try development mode first: Check if we're in the repo (colcon2deb/ dir exists at same level as this script)
    # The script is in colcon2deb/cli.py, so script_dir is colcon2deb/
    # Parent of script_dir is the repo root
    repo_root = script_dir.parent
    packager_src = repo_root / "colcon2deb"
    is_dev_mode = False

    if packager_src.exists() and (repo_root / "pyproject.toml").exists():
        # Development mode - use the repository root which has pyproject.toml
        colcon2deb_src = repo_root
        is_dev_mode = True
    else:
        # Wheel installation - locate the installed package
        try:
            import importlib.util
            spec = importlib.util.find_spec("colcon2deb")
            if spec is None or spec.origin is None:
                raise ImportError("Package not found")

            # Get package location in site-packages
            pkg_init = Path(spec.origin).resolve()
            packager_src = pkg_init.parent

            # For wheel mode, mount the package directory itself
            # PYTHONPATH will need to point to the parent of the mounted package
            colcon2deb_src = packager_src
            is_dev_mode = False

        except (ImportError, AttributeError) as e:
            print("Error: colcon2deb package not found", file=sys.stderr)
            print(f"Tried development mode at: {repo_root / 'colcon2deb'}", file=sys.stderr)
            print("Tried importlib but package is not installed", file=sys.stderr)
            print(f"Details: {e}", file=sys.stderr)
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

    if verbose:
        print(f"\n  ROS Distribution: {ros_distro}")
        print(f"  Install Prefix: {install_prefix}")

    # Prepare Docker run command
    # Development mode: mount repo and pip install
    # Wheel mode: mount site-packages and use PYTHONPATH
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
        "-v",
        "/tmp/.X11-unix/:/tmp/.X11-unix",
        "-v",
        f"{workspace_dir}:/workspace",
        "-v",
        f"{packages_dir}:/config",
        "-v",
        f"{colcon2deb_src}:/opt/colcon2deb_pkg/colcon2deb:ro",
        "-v",
        f"{output_dir}:/output",
        image_name,
        "bash",
        "-c",
    ]

    if is_dev_mode:
        # Development mode: install from source
        bash_script = f"""
        set -e
        # Create user for specified uid/gid
        groupadd -g {gid} ubuntu 2>/dev/null || true
        useradd -m -u {uid} -g {gid} ubuntu 2>/dev/null || true
        usermod -aG sudo ubuntu 2>/dev/null || true
        passwd -d ubuntu 2>/dev/null || true

        # Fix permissions
        chown -R ubuntu:ubuntu /workspace

        # Install the Python package from source in editable mode
        cd /opt/colcon2deb_pkg
        pip3 install -e . || {{
            echo "Error: Failed to install colcon2deb" >&2
            exit 1
        }}

        # Initialize rosdep (must be done as root)
        sudo rosdep init || true  # Ignore if already initialized

        # Run the builder as ubuntu user
        sudo -u ubuntu bash -c "
            set -e
            rosdep update
            python3 -m colcon2deb.main \\
                --workspace=/workspace \\
                --output=/output \\
                --ros-distro={ros_distro} \\
                --install-prefix={install_prefix}
        "
        """
    else:
        # Wheel mode: use PYTHONPATH to access mounted package
        bash_script = f"""
        set -e
        # Create user for specified uid/gid
        groupadd -g {gid} ubuntu 2>/dev/null || true
        useradd -m -u {uid} -g {gid} ubuntu 2>/dev/null || true
        usermod -aG sudo ubuntu 2>/dev/null || true
        passwd -d ubuntu 2>/dev/null || true

        # Fix permissions
        chown -R ubuntu:ubuntu /workspace

        # Install dependencies
        pip3 install pyyaml rich docker requests

        # Initialize rosdep (must be done as root)
        sudo rosdep init || true  # Ignore if already initialized

        # Run the builder as ubuntu user with PYTHONPATH
        sudo -u ubuntu bash -c "
            set -e
            export PYTHONPATH=/opt/colcon2deb_pkg:$PYTHONPATH
            rosdep update
            python3 -m colcon2deb.main \\
                --workspace=/workspace \\
                --output=/output \\
                --ros-distro={ros_distro} \\
                --install-prefix={install_prefix}
        "
        """

    docker_cmd.append(bash_script)

    # Add nvidia runtime if available and requested in config
    use_nvidia = build_config.get("use_nvidia_runtime", False)
    if use_nvidia:
        docker_cmd.insert(4, "--runtime")
        docker_cmd.insert(5, "nvidia")

    # Run the container
    print(f"{Colors.CYAN}Building packages...{Colors.RESET}")
    if verbose:
        mode_str = "development" if is_dev_mode else "wheel (PYTHONPATH)"
        print(f"  Mode: {mode_str}")
        print(f"  Workspace directory: {workspace_dir} -> /workspace")
        print(f"  Packages config directory: {packages_dir} -> /config")
        print(f"  Output directory: {output_dir} -> /output")
        print(f"  Package source: {colcon2deb_src} -> /opt/colcon2deb_pkg/colcon2deb (read-only)")
        print(f"  Using image: {image_name}")

    # Capture container output to log file
    container_log = output_dir / "log" / "container.log"
    container_log.parent.mkdir(parents=True, exist_ok=True)

    try:
        with open(container_log, 'w') as log_file:
            result = subprocess.run(
                docker_cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                check=True
            )
        print(f"{Colors.BOLD}{Colors.BRIGHT_GREEN}✓ Build completed successfully!{Colors.RESET}")
        print(f"  Debian packages: {Colors.GREEN}{output_dir}/dist/{Colors.RESET}")
        if verbose:
            print(f"  Container log: {container_log}")
    except subprocess.CalledProcessError as e:
        print(f"\n{Colors.BOLD}{Colors.BRIGHT_RED}✗ Build failed (exit code {e.returncode}){Colors.RESET}", file=sys.stderr)
        print(f"  See container log: {Colors.YELLOW}{container_log}{Colors.RESET}", file=sys.stderr)
        # Show last 50 lines of container log on failure
        if container_log.exists():
            print(f"\n{Colors.BRIGHT_YELLOW}--- Last 50 lines of container output ---{Colors.RESET}", file=sys.stderr)
            with open(container_log, 'r') as f:
                lines = f.readlines()
                for line in lines[-50:]:
                    print(line.rstrip(), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n{Colors.BRIGHT_YELLOW}✗ Interrupted by user{Colors.RESET}")
        sys.exit(0)


if __name__ == "__main__":
    main()
