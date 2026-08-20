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
from typing import Any, Literal, cast

import events

# Import rosdeb_bloom library (pip installed from /rosdeb-bloom in container)
from rosdeb_bloom.api import generate_debian  # type: ignore[import-untyped]
from rosdeb_bloom.logging import disable_ANSI_colors  # type: ignore[import-untyped]


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

    packages: list[tuple[str, Path]] = []
    for line in result.stdout.strip().split("\n"):
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) >= 2:
            pkg_name = parts[0]
            pkg_dir = (colcon_work_dir / parts[1]).resolve()
            # Skip non-ROS packages (no package.xml) — bloom requires it
            if not (pkg_dir / "package.xml").exists():
                continue
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
    log_packages_dir: Path | None = None,
    fingerprint_inputs: dict[str, str] | None = None,
) -> DebianDirResult:
    """Generate debian directory for a single package.

    Either copies from debian-overrides or uses rosdeb_bloom library.
    """
    from fingerprint import (
        compute_fingerprint,
        fingerprint_matches,
        fingerprint_path,
        read_fingerprint,
        write_fingerprint,
    )

    pkg_work_dir = pkg_build_dir / pkg_name
    src_debian_dir = config_dir / pkg_name / "debian"
    dst_debian_dir = pkg_work_dir / "debian"
    fp_file = fingerprint_path(pkg_work_dir, "debian")

    # Per-package log file
    if log_packages_dir:
        pkg_log_dir = log_packages_dir / pkg_name
        pkg_log_dir.mkdir(parents=True, exist_ok=True)
        log_file = pkg_log_dir / "generate_debian.log"
    else:
        log_file = pkg_work_dir / "generate_debian.log"

    # Create directories
    pkg_work_dir.mkdir(parents=True, exist_ok=True)
    log_file.write_text("")

    # Check fingerprint — skip if inputs unchanged and debian/ already generated
    if fingerprint_inputs is not None:
        current_fp = compute_fingerprint(
            pkg_name=pkg_name,
            pkg_dir=pkg_dir,
            overrides_dir=config_dir,
            **fingerprint_inputs,
        )
        stored_fp = read_fingerprint(fp_file)
        if fingerprint_matches(stored_fp, current_fp) and dst_debian_dir.is_dir():
            log_file.write_text("Skipped: fingerprint unchanged\n")
            return DebianDirResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=DebianDirStatus.SUCCESS,
                method="copy" if src_debian_dir.is_dir() else "bloom",
            )
    else:
        current_fp = None

    try:
        if src_debian_dir.is_dir():
            # Copy pre-defined debian directory

            result = subprocess.run(
                ["rsync", "-av", "--delete", f"{src_debian_dir}/", f"{dst_debian_dir}/"],
                capture_output=True,
                text=True,
            )

            log_file.write_text(
                result.stdout + ("\n--- STDERR ---\n" + result.stderr if result.stderr else "")
            )

            if result.returncode != 0:
                return DebianDirResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=DebianDirStatus.FAILED,
                    method="copy",
                    error=f"rsync failed: {result.stderr}",
                )

            if current_fp:
                write_fingerprint(fp_file, current_fp)
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
            bloom_result: Any = generate_debian(  # pyright: ignore[reportUnknownVariableType]
                package_path=pkg_dir,
                ros_distro=ros_distro,
                install_prefix=ros_install_prefix,
                peer_packages=peer_packages,
                package_suffix=package_suffix,
            )

            if not cast(bool, bloom_result.success):  # pyright: ignore[reportUnknownMemberType]
                error_msg = cast(str, bloom_result.error) or "Unknown error"  # pyright: ignore[reportUnknownMemberType]
                log_file.write_text(error_msg)
                return DebianDirResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=DebianDirStatus.FAILED,
                    method="bloom",
                    error=f"rosdeb_bloom failed: {error_msg}",
                )

            debian_dir_path = cast(str, bloom_result.debian_dir)  # pyright: ignore[reportUnknownMemberType]
            log_file.write_text(f"Generated debian directory at {debian_dir_path}\n")

            # The library generates debian/ in pkg_dir; move it to the work dir.
            # config_dir is user-provided overrides only — never write there:
            # caching bloom output into it made stale metadata (generated with
            # an old install_prefix/version) shadow regeneration forever.
            generated_debian_dir = pkg_dir / "debian"

            if generated_debian_dir.is_dir():
                if dst_debian_dir.exists():
                    shutil.rmtree(dst_debian_dir)
                shutil.copytree(generated_debian_dir, dst_debian_dir)

                # Clean up debian dir from source package dir
                shutil.rmtree(generated_debian_dir)

                log_file.write_text(log_file.read_text() + f"Copied to {dst_debian_dir}\n")

            if current_fp:
                write_fingerprint(fp_file, current_fp)
            return DebianDirResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=DebianDirStatus.SUCCESS,
                method="bloom",
            )

    except Exception as e:
        import traceback

        log_file.write_text(f"{e}\n{traceback.format_exc()}")
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
    log_packages_dir = Path(os.environ.get("log_packages_dir", ""))
    ros_distro = get_env_str("ROS_DISTRO", "humble")
    ros_install_prefix = get_env_str("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")
    # Optional package suffix (e.g., "1.5.0" for ros-humble-pkg-1.5.0)
    package_suffix = os.environ.get("ROS_PACKAGE_SUFFIX") or None

    # Build fingerprint inputs from environment
    from fingerprint import get_fingerprint_inputs_from_env

    fp_inputs = get_fingerprint_inputs_from_env()

    os.chdir(colcon_work_dir)

    # Join the orchestrator's event stream for per-package TUI progress
    events.attach(get_env_path("output_dir"))

    # Get package list
    packages = get_package_list(colcon_work_dir)
    print(f"Processing {len(packages)} packages...")

    # Get all package names for peer_packages (to skip rosdep resolution for workspace packages)
    all_package_names = [pkg_name for pkg_name, _ in packages]

    # Add ROS 2 build type dependencies that should be skipped during rosdep resolution
    # These are buildtool dependencies that aren't rosdep keys
    ros2_build_types = ["ament_cmake", "ament_python", "ament_cmake_python", "cmake", "catkin"]
    all_package_names.extend(ros2_build_types)

    # Read parallel_jobs from config (via environment variable)
    # For I/O-bound operations, use half the configured parallelism
    cpu_count = os.cpu_count() or 1
    config_parallel = int(os.environ.get("COLCON2DEB_PARALLEL_JOBS", 0))
    total_parallel = config_parallel if config_parallel > 0 else cpu_count
    njobs = max(1, total_parallel // 2)

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
                log_packages_dir if str(log_packages_dir) else None,
                fp_inputs,
            ): pkg_name
            for pkg_name, pkg_dir in packages
        }

        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                result = future.result()
                results.append(result)

                if result.status == DebianDirStatus.FAILED:
                    events.package_complete(7, pkg_name, success=False)
                    pkg_log = (log_packages_dir or pkg_build_dir) / pkg_name / "generate_debian.log"
                    print(
                        f"error: failed to generate Debian files for {pkg_name} (see {pkg_log})",
                        file=sys.stderr,
                    )
            except Exception as e:
                print(f"error: exception processing {pkg_name}: {e}", file=sys.stderr)
                events.package_complete(7, pkg_name, success=False)
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
