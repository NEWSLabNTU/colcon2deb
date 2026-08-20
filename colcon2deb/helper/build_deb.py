#!/usr/bin/env python3
"""Build Debian packages for ROS packages.

This script builds .deb packages from ROS packages that have debian/
directories already generated. It:
1. Checks for existing .deb files (skips if already built)
2. Copies debian/ from pkg_build_dir to pkg_dir
3. Runs fakeroot debian/rules binary
4. Moves output .deb files to release_dir

Uses ThreadPoolExecutor for parallel execution.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import events
from colcon_events import ColconGate


class BuildStatus(Enum):
    SUCCESS = "success"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class BuildResult:
    """Result of building a Debian package."""

    package: str
    pkg_dir: Path
    status: BuildStatus
    deb_file: Path | None = None
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
            # Skip non-ROS packages (no package.xml) — phase 7 deliberately
            # generates no debian/ for them, so building would always fail
            if not (pkg_dir / "package.xml").exists():
                continue
            packages.append((pkg_name, pkg_dir))

    return packages


def find_deb_file(
    directory: Path, ros_distro: str, pkg_name_dashed: str, package_suffix: str | None = None
) -> Path | None:
    """Find a .deb file matching the package pattern."""
    if package_suffix:
        pattern = f"ros-{ros_distro}-{pkg_name_dashed}-{package_suffix}_*.deb"
    else:
        pattern = f"ros-{ros_distro}-{pkg_name_dashed}_*.deb"
    matches = list(directory.glob(pattern))
    return matches[0] if matches else None


def find_ddeb_file(
    directory: Path, ros_distro: str, pkg_name_dashed: str, package_suffix: str | None = None
) -> Path | None:
    """Find a .ddeb debug file matching the package pattern."""
    if package_suffix:
        pattern = f"ros-{ros_distro}-{pkg_name_dashed}-{package_suffix}-dbgsym_*.ddeb"
    else:
        pattern = f"ros-{ros_distro}-{pkg_name_dashed}-dbgsym_*.ddeb"
    matches = list(directory.glob(pattern))
    return matches[0] if matches else None


def build_single_package(
    pkg_name: str,
    pkg_dir: Path,
    pkg_build_dir: Path,
    release_dir: Path,
    check_dir: Path,
    ros_distro: str,
    ros_install_prefix: str,
    colcon_install_path: str,
    package_suffix: str | None = None,
    log_packages_dir: Path | None = None,
    fingerprint_inputs: dict[str, str] | None = None,
    colcon_gate: ColconGate | None = None,
) -> BuildResult:
    """Build a single Debian package.

    Returns BuildResult with status and optional deb_file path.
    """
    from fingerprint import (
        compute_fingerprint,
        fingerprint_matches,
        fingerprint_path,
        read_fingerprint,
        write_fingerprint,
    )

    pkg_name_dashed = pkg_name.replace("_", "-")
    pkg_work_dir = pkg_build_dir / pkg_name
    fp_file = fingerprint_path(pkg_work_dir, "deb")

    # Per-package log file: logs/packages/{pkg}/build_deb.log
    if log_packages_dir:
        pkg_log_dir = log_packages_dir / pkg_name
        pkg_log_dir.mkdir(parents=True, exist_ok=True)
        log_file = pkg_log_dir / "build_deb.log"
    else:
        log_file = pkg_work_dir / "build_deb.log"

    # Ensure directories exist
    pkg_work_dir.mkdir(parents=True, exist_ok=True)
    log_file.write_text("")

    try:
        # Check fingerprint: skip only if inputs unchanged AND .deb exists
        existing_deb = find_deb_file(check_dir, ros_distro, pkg_name_dashed, package_suffix)
        if fingerprint_inputs is not None:
            current_fp = compute_fingerprint(
                pkg_name=pkg_name,
                pkg_dir=pkg_dir,
                overrides_dir=Path(os.environ.get("config_dir", "/config")),
                **fingerprint_inputs,
            )
            stored_fp = read_fingerprint(fp_file)
            if fingerprint_matches(stored_fp, current_fp) and existing_deb:
                log_file.write_text("Skipped: fingerprint unchanged\n")
                return BuildResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=BuildStatus.SKIPPED,
                    deb_file=existing_deb,
                )
        else:
            current_fp = None
            # Fallback: old behavior (skip if .deb exists)
            if existing_deb:
                return BuildResult(
                    package=pkg_name,
                    pkg_dir=pkg_dir,
                    status=BuildStatus.SKIPPED,
                    deb_file=existing_deb,
                )

        # Pipelined build: wait until colcon has finished this package (its
        # dependencies are then guaranteed present in the install tree).
        # Cached packages skip above without waiting.
        if colcon_gate is not None and not colcon_gate.wait(pkg_name):
            error_msg = "colcon build failed before this package finished; .deb build skipped"
            log_file.write_text(error_msg + "\n")
            return BuildResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=BuildStatus.FAILED,
                error=error_msg,
            )

        # Check debian directory exists in work dir
        src_debian_dir = pkg_work_dir / "debian"
        if not src_debian_dir.is_dir():
            error_msg = f"debian directory not found at {src_debian_dir}"
            log_file.write_text(error_msg)
            return BuildResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=BuildStatus.FAILED,
                error=error_msg,
            )

        # Clean up old build artifacts
        dst_debian_dir = pkg_dir / "debian"
        if dst_debian_dir.exists():
            shutil.rmtree(dst_debian_dir)

        # Clean up old .obj-* directories and .deb/.ddeb files.
        # dh_builddeb emits into pkg_dir.parent, so stale artifacts must be
        # removed there or find_deb_file below may pick up an old version.
        for obj_dir in pkg_dir.glob(".obj-*"):
            shutil.rmtree(obj_dir)
        suffix_part = f"-{package_suffix}" if package_suffix else ""
        artifact_stem = f"ros-{ros_distro}-{pkg_name_dashed}{suffix_part}"
        for old_artifact in (
            *pkg_dir.parent.glob(f"{artifact_stem}_*.deb"),
            *pkg_dir.parent.glob(f"{artifact_stem}-dbgsym_*.ddeb"),
        ):
            old_artifact.unlink()

        # Clean up debhelper state files from source debian directory
        # These files cause dh to skip configure/build phases if left from a previous run
        debhelper_dir = src_debian_dir / ".debhelper"
        if debhelper_dir.exists():
            shutil.rmtree(debhelper_dir)
        debhelper_stamp = src_debian_dir / "debhelper-build-stamp"
        if debhelper_stamp.exists():
            debhelper_stamp.unlink()

        # Copy debian directory to package
        shutil.copytree(src_debian_dir, dst_debian_dir)

        # Build the package
        # Use per_pkg_parallel from environment (set by main())
        per_pkg_parallel = int(os.environ.get("COLCON2DEB_PER_PKG_PARALLEL", os.cpu_count() or 1))
        env = os.environ.copy()
        env["ROS_INSTALL_PREFIX"] = ros_install_prefix
        env["COLCON_INSTALL_PATH"] = colcon_install_path
        deb_build_options = f"parallel={per_pkg_parallel}"
        if os.environ.get("COLCON2DEB_SKIP_TESTS") == "1":
            deb_build_options += " nocheck"
        env["DEB_BUILD_OPTIONS"] = deb_build_options

        # Stream output directly to the log file so it survives OOM kills
        # and interrupts mid-build (this is the full compile step).
        with open(log_file, "w") as log_fh:
            result = subprocess.run(
                ["fakeroot", "debian/rules", "binary"],
                cwd=pkg_dir,
                stdout=log_fh,
                stderr=subprocess.STDOUT,
                env=env,
            )

        if result.returncode != 0:
            return BuildResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=BuildStatus.FAILED,
                error=f"fakeroot failed with exit code {result.returncode} (see {log_file})",
            )

        # Find and move .deb file
        parent_dir = pkg_dir.parent
        deb_file = find_deb_file(parent_dir, ros_distro, pkg_name_dashed, package_suffix)

        if deb_file:
            dest_deb = release_dir / deb_file.name
            shutil.move(str(deb_file), str(dest_deb))

            # Also move .ddeb if exists
            ddeb_file = find_ddeb_file(parent_dir, ros_distro, pkg_name_dashed, package_suffix)
            if ddeb_file:
                dest_ddeb = release_dir / ddeb_file.name
                shutil.move(str(ddeb_file), str(dest_ddeb))

            if current_fp:
                write_fingerprint(fp_file, current_fp)
            return BuildResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=BuildStatus.SUCCESS,
                deb_file=dest_deb,
            )
        else:
            return BuildResult(
                package=pkg_name,
                pkg_dir=pkg_dir,
                status=BuildStatus.FAILED,
                error="build command succeeded but no .deb file found",
            )

    except Exception as e:
        log_file.write_text(str(e))
        return BuildResult(
            package=pkg_name,
            pkg_dir=pkg_dir,
            status=BuildStatus.FAILED,
            error=str(e),
        )


def main() -> int:
    """Main entry point."""
    # Get configuration from environment
    colcon_work_dir = get_env_path("colcon_work_dir")
    pkg_build_dir = get_env_path("pkg_build_dir")
    release_dir = get_env_path("release_dir")
    check_dir = get_env_path("check_dir")
    log_dir = get_env_path("log_dir")
    log_packages_dir = Path(os.environ.get("log_packages_dir", str(log_dir / "packages")))

    from fingerprint import get_fingerprint_inputs_from_env

    fp_inputs = get_fingerprint_inputs_from_env()

    ros_distro = get_env_str("ROS_DISTRO", "humble")
    ros_install_prefix = get_env_str("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")
    colcon_install_path = str(colcon_work_dir / "install")
    # Optional package suffix (e.g., "1.5.0" for ros-humble-pkg-1.5.0)
    package_suffix = os.environ.get("ROS_PACKAGE_SUFFIX") or None

    # Status files - use environment variables if set, else defaults (no number prefix for outcome files)
    successful_pkgs_file = Path(
        os.environ.get("successful_pkgs_file", str(log_dir / "successful_pkgs.txt"))
    )
    failed_pkgs_file = Path(os.environ.get("failed_pkgs_file", str(log_dir / "failed_pkgs.txt")))
    skipped_pkgs_file = Path(os.environ.get("skipped_pkgs_file", str(log_dir / "skipped_pkgs.txt")))

    os.chdir(colcon_work_dir)

    # Get package list
    packages = get_package_list(colcon_work_dir)
    print(f"Building {len(packages)} packages...")

    # Parallel package builds: balance cross-package and per-package parallelism
    # Total concurrent processes = njobs × per_pkg_parallel ≤ total_parallel
    # Using sqrt distribution: njobs ≈ √total, per_pkg_parallel ≈ total / njobs
    cpu_count = os.cpu_count() or 1

    # Read parallel_jobs from config (via environment variable)
    # If set to 0 or not set, auto-calculate based on CPU count
    config_parallel = int(os.environ.get("COLCON2DEB_PARALLEL_JOBS", 0))
    total_parallel = config_parallel if config_parallel > 0 else cpu_count

    njobs = max(1, int(total_parallel**0.5))
    per_pkg_parallel = max(1, total_parallel // njobs)

    print(
        f"Parallelism: {njobs} packages × {per_pkg_parallel} jobs/pkg = {njobs * per_pkg_parallel} total (config: {config_parallel}, CPU: {cpu_count})"
    )

    # Set environment variable for build_single_package to use
    os.environ["COLCON2DEB_PER_PKG_PARALLEL"] = str(per_pkg_parallel)

    # Join the orchestrator's event stream for per-package TUI progress
    events.attach(get_env_path("output_dir"))

    # Pipelined build: gate each package on colcon completing it. The
    # orchestrator provides the colcon log dir, the pre-run snapshot of
    # build_* dirs, and a status file it writes when colcon exits.
    colcon_gate: ColconGate | None = None
    if os.environ.get("COLCON2DEB_PIPELINE") == "1":
        exclude_env = os.environ.get("COLCON2DEB_COLCON_EXCLUDE", "")
        colcon_gate = ColconGate(
            log_base=Path(os.environ["COLCON2DEB_COLCON_LOG_BASE"]),
            status_file=Path(os.environ["COLCON2DEB_COLCON_STATUS_FILE"]),
            exclude_dirs=set(filter(None, exclude_env.split(","))),
        )
        colcon_gate.start()

    # Process packages in parallel
    results: list[BuildResult] = []

    with ThreadPoolExecutor(max_workers=njobs) as executor:
        futures = {
            executor.submit(
                build_single_package,
                pkg_name,
                pkg_dir,
                pkg_build_dir,
                release_dir,
                check_dir,
                ros_distro,
                ros_install_prefix,
                colcon_install_path,
                package_suffix,
                log_packages_dir,
                fp_inputs,
                colcon_gate,
            ): pkg_name
            for pkg_name, pkg_dir in packages
        }

        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                result = future.result()
                results.append(result)

                if result.status != BuildStatus.SKIPPED:
                    events.package_complete(
                        8, pkg_name, success=result.status == BuildStatus.SUCCESS
                    )
                if result.status == BuildStatus.FAILED:
                    pkg_log = log_packages_dir / pkg_name / "build_deb.log"
                    print(
                        f"error: failed to build Debian package for {pkg_name} (see {pkg_log})",
                        file=sys.stderr,
                    )
            except Exception as e:
                print(f"error: exception building {pkg_name}: {e}", file=sys.stderr)
                events.package_complete(8, pkg_name, success=False)
                results.append(
                    BuildResult(
                        package=pkg_name,
                        pkg_dir=Path("."),
                        status=BuildStatus.FAILED,
                        error=str(e),
                    )
                )

    if colcon_gate is not None:
        colcon_gate.stop()

    # Write status files
    successful = [r.package for r in results if r.status == BuildStatus.SUCCESS]
    failed = [r.package for r in results if r.status == BuildStatus.FAILED]
    skipped = [r.package for r in results if r.status == BuildStatus.SKIPPED]

    successful_pkgs_file.write_text("\n".join(successful) + "\n" if successful else "")
    failed_pkgs_file.write_text("\n".join(failed) + "\n" if failed else "")
    skipped_pkgs_file.write_text("\n".join(skipped) + "\n" if skipped else "")

    # Summary
    if failed:
        print(f"Built {len(successful)} packages ({len(skipped)} cached, {len(failed)} failed)")
        for r in results:
            if r.status == BuildStatus.FAILED:
                print(f"  - {r.package}: {r.error}")
    else:
        print(f"Built {len(successful)} packages ({len(skipped)} cached)")

    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
