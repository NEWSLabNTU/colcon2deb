#!/usr/bin/env python3
"""Generate Debian packaging metadata for ROS packages.

This script generates debian/ directories for each package using either:
1. Pre-defined debian-overrides (copied from config_dir)
2. rosdeb_bloom library (auto-generated)

Uses ProcessPoolExecutor for parallel execution.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Literal

# Import rosdeb_bloom library (pip installed from /rosdeb-bloom in container)
from rosdeb_bloom.api import generate_debian
from rosdeb_bloom.logging import disable_ANSI_colors


class DebianDirStatus(Enum):
    SUCCESS = "success"
    FAILED = "failed"


@dataclass
class DebianDirResult:
    """Result of generating debian directory for a package."""

    package: str
    pkg_dir: Path
    status: DebianDirStatus
    method: Literal["copy", "bloom"]
    error: str | None = None


def get_env_path(name: str) -> Path:
    """Get a path from environment variable."""
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"Environment variable {name} is not set")
    return Path(value)


def get_env_str(name: str, default: str | None = None) -> str:
    """Get a string from environment variable."""
    value = os.environ.get(name, default)
    if value is None:
        raise RuntimeError(f"Environment variable {name} is not set")
    return value


def get_package_list(colcon_work_dir: Path) -> list[tuple[str, Path]]:
    """Get list of packages from colcon."""
    result = subprocess.run(
        ["colcon", "list", "--base-paths", "src"],
        cwd=colcon_work_dir,
        capture_output=True,
        text=True,
        check=True,
    )

    packages = []
    for line in result.stdout.strip().split("\n"):
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) >= 2:
            pkg_name = parts[0]
            pkg_dir = (colcon_work_dir / parts[1]).resolve()
            packages.append((pkg_name, pkg_dir))

    return packages


def copy_or_create_debian_dir(
    pkg_name: str,
    pkg_dir: Path,
    config_dir: Path,
    pkg_build_dir: Path,
    script_dir: Path,
    ros_distro: str,
    ros_install_prefix: str,
    peer_packages: list[str],
    package_suffix: str | None = None,
) -> DebianDirResult:
    """Generate debian directory for a single package.

    Either copies from debian-overrides or uses rosdeb_bloom library.
    """
    pkg_work_dir = pkg_build_dir / pkg_name
    pkg_config_dir = config_dir / pkg_name
    out_file = pkg_work_dir / "gen_deb.out"
    err_file = pkg_work_dir / "gen_deb.err"
    src_debian_dir = config_dir / pkg_name / "debian"
    dst_debian_dir = pkg_work_dir / "debian"

    # Create directories
    pkg_work_dir.mkdir(parents=True, exist_ok=True)
    pkg_config_dir.mkdir(parents=True, exist_ok=True)

    # Clear log files
    out_file.write_text("")
    err_file.write_text("")

    try:
        if src_debian_dir.is_dir():
            # Copy pre-defined debian directory

            result = subprocess.run(
                ["rsync", "-av", "--delete", f"{src_debian_dir}/", f"{dst_debian_dir}/"],
                capture_output=True,
                text=True,
            )

            out_file.write_text(result.stdout)
            err_file.write_text(result.stderr)

            if result.returncode != 0:
                return DebianDirResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=DebianDirStatus.FAILED,
                    method="copy",
                    error=f"rsync failed: {result.stderr}",
                )

            return DebianDirResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=DebianDirStatus.SUCCESS,
                method="copy",
            )

        else:
            # Generate using rosdeb_bloom library
            # Disable ANSI colors in child process for clean output
            disable_ANSI_colors()

            # Ensure ~/.config exists for bloom
            home_config = Path.home() / ".config"
            home_config.mkdir(parents=True, exist_ok=True)

            # Generate debian files using the library API
            result = generate_debian(
                package_path=pkg_dir,
                ros_distro=ros_distro,
                install_prefix=ros_install_prefix,
                peer_packages=peer_packages,
                package_suffix=package_suffix,
            )

            if not result.success:
                err_file.write_text(result.error or "Unknown error")
                return DebianDirResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=DebianDirStatus.FAILED,
                    method="bloom",
                    error=f"rosdeb_bloom failed: {result.error}",
                )

            out_file.write_text(f"Generated debian directory at {result.debian_dir}\n")

            # The library generates debian/ in pkg_dir, copy to cache (config_dir) and work dir
            generated_debian_dir = pkg_dir / "debian"

            if generated_debian_dir.is_dir():
                # Cache to config_dir for subsequent builds
                cache_debian_dir = pkg_config_dir / "debian"
                if cache_debian_dir.exists():
                    shutil.rmtree(cache_debian_dir)
                shutil.copytree(generated_debian_dir, cache_debian_dir)

                # Copy to work dir
                if dst_debian_dir.exists():
                    shutil.rmtree(dst_debian_dir)
                shutil.copytree(generated_debian_dir, dst_debian_dir)

                # Clean up debian dir from source package dir
                shutil.rmtree(generated_debian_dir)

                out_file.write_text(
                    out_file.read_text() + f"Cached to {cache_debian_dir}\n"
                    f"Copied to {dst_debian_dir}\n"
                )

            return DebianDirResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=DebianDirStatus.SUCCESS,
                method="bloom",
            )

    except Exception as e:
        import traceback

        err_file.write_text(f"{e}\n{traceback.format_exc()}")
        return DebianDirResult(
            package=pkg_name,
            pkg_dir=pkg_dir,
            status=DebianDirStatus.FAILED,
            method="copy" if src_debian_dir.is_dir() else "bloom",
            error=str(e),
        )


def main() -> int:
    """Main entry point."""
    # Disable ANSI colors for clean TUI output
    disable_ANSI_colors()

    # Get configuration from environment
    colcon_work_dir = get_env_path("colcon_work_dir")
    config_dir = get_env_path("config_dir")
    pkg_build_dir = get_env_path("pkg_build_dir")
    script_dir = get_env_path("script_dir")
    ros_distro = get_env_str("ROS_DISTRO", "humble")
    ros_install_prefix = get_env_str("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")
    # Optional package suffix (e.g., "1.5.0" for ros-humble-pkg-1.5.0)
    package_suffix = os.environ.get("ROS_PACKAGE_SUFFIX") or None

    os.chdir(colcon_work_dir)

    # Get package list
    packages = get_package_list(colcon_work_dir)
    print(f"Processing {len(packages)} packages...")

    # Get all package names for peer_packages (to skip rosdep resolution for workspace packages)
    all_package_names = [pkg_name for pkg_name, _ in packages]

    # Add ROS 2 build type dependencies that should be skipped during rosdep resolution
    # These are buildtool dependencies that aren't rosdep keys
    ros2_build_types = ["ament_cmake", "ament_python", "ament_cmake_python", "cmake", "catkin"]
    all_package_names.extend(ros2_build_types)

    # Use half the CPU cores for I/O-heavy operations
    njobs = max(1, (os.cpu_count() or 1 + 1) // 2)

    # Process packages in parallel
    results: list[DebianDirResult] = []

    with ProcessPoolExecutor(max_workers=njobs) as executor:
        futures = {
            executor.submit(
                copy_or_create_debian_dir,
                pkg_name,
                pkg_dir,
                config_dir,
                pkg_build_dir,
                script_dir,
                ros_distro,
                ros_install_prefix,
                all_package_names,  # Pass all packages as peer_packages
                package_suffix,
            ): pkg_name
            for pkg_name, pkg_dir in packages
        }

        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                result = future.result()
                results.append(result)

                if result.status == DebianDirStatus.FAILED:
                    print(f"error: fail to generate Debian files for {pkg_name}", file=sys.stderr)
            except Exception as e:
                print(f"error: exception processing {pkg_name}: {e}", file=sys.stderr)
                results.append(
                    DebianDirResult(
                        package=pkg_name,
                        pkg_dir=Path("."),
                        status=DebianDirStatus.FAILED,
                        method="bloom",
                        error=str(e),
                    )
                )

    # Summary
    success_count = sum(1 for r in results if r.status == DebianDirStatus.SUCCESS)
    failed_count = sum(1 for r in results if r.status == DebianDirStatus.FAILED)

    if failed_count > 0:
        print(
            f"Generated {success_count}/{len(results)} debian directories ({failed_count} failed)"
        )
        for r in results:
            if r.status == DebianDirStatus.FAILED:
                print(f"  - {r.package}: {r.error}")
    else:
        print(f"Generated {success_count} debian directories")

    return 0 if failed_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
