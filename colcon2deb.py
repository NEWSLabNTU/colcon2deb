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


def run_command(cmd, check=True):
    """Run a shell command and return the result."""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=check, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr, file=sys.stderr)
    return result


def download_dockerfile(url, cache_dir=None):
    """Download Dockerfile from HTTP/HTTPS URL."""
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
            print(f"  ✓ Using cached Dockerfile")
            print(f"  Cache location: {cached_file}")
            return cached_file

    try:
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

        print(f"  ✓ Downloaded {content_length} bytes")

        # Save to temporary file or cache
        if cache_dir and cached_file:
            cached_file.write_bytes(content)
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


def build_image_from_dockerfile(dockerfile_path, image_name, build_context=None):
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

    print(f"Building Docker image '{image_name}' from {dockerfile_path}")
    print(f"Build context: {build_context}")
    run_command(cmd)
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

    # Determine the image to use
    if "dockerfile" in docker_config:
        dockerfile_value = docker_config["dockerfile"]

        # Check if it's a URL
        if dockerfile_value.startswith(("http://", "https://")):
            print("\n" + "=" * 60)
            print("Remote Dockerfile Configuration Detected")
            print("=" * 60)

            # Download Dockerfile from URL
            # Use cache directory in user's home or temp
            cache_dir = Path.home() / ".cache" / "colcon2deb" / "dockerfiles"
            dockerfile_path = download_dockerfile(dockerfile_value, cache_dir)

            # For remote Dockerfiles, use a minimal build context
            # Create a temporary directory with just the Dockerfile
            temp_context = tempfile.mkdtemp(prefix="colcon2deb_context_")

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

    # Helper directory must be in the same directory as this script
    helper_dir = script_dir / "helper"

    if not helper_dir.exists():
        print("Error: Helper scripts directory not found", file=sys.stderr)
        print(f"Expected at: {helper_dir}", file=sys.stderr)
        print(
            "Helper directory must be in the same directory as colcon2deb.py",
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

    # Get output directory configuration - this will be our main working directory
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

    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # The build_deb directory will be created inside the output directory
    # Users will find packages in output_dir/dist/

    # Get build configuration
    build_config = config.get("build", {})
    ros_distro = build_config.get("ros_distro", "humble")

    # Get install prefix configuration (optional)
    # Default: /opt/ros/{ros_distro}
    install_prefix = build_config.get("install_prefix", f"/opt/ros/{ros_distro}")

    print(f"\n  ROS Distribution: {ros_distro}")
    print(f"  Install Prefix: {install_prefix}")

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
        image_name,
        "/helper/entry.sh",
        f"--uid={uid}",
        f"--gid={gid}",
        f"--output=/output",
    ]

    # Add nvidia runtime if available and requested in config
    use_nvidia = build_config.get("use_nvidia_runtime", False)
    if use_nvidia:
        docker_cmd.insert(4, "--runtime")
        docker_cmd.insert(5, "nvidia")

    # Run the container
    print(f"\nStarting container:")
    print(f"  Workspace directory: {workspace_dir} -> /workspace")
    print(f"  Packages config directory: {packages_dir} -> /config")
    print(f"  Output directory: {output_dir} -> /output")
    print(f"  Helper directory: {helper_dir} -> /helper")
    print(f"  Using image: {image_name}")
    print(f"\n  Build artifacts will be in: {output_dir}/")
    print(f"  Final .deb packages will be in: {output_dir}/dist/")

    try:
        subprocess.run(docker_cmd)
    except subprocess.CalledProcessError as e:
        print(f"Error running container: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()
