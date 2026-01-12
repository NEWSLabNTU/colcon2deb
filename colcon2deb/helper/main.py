#!/usr/bin/env python3
"""Main orchestrator script for building Debian packages.

This script replaces main.sh and coordinates the entire build process:
1. Preparing working directories
2. Copying source files
3. Installing dependencies
4. Compiling packages
5. Generating rosdep list
6. Creating package list
7. Generating Debian metadata
8. Building Debian packages
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import events


class SimpleBuildUI:
    """Simple UI for build progress display."""

    def __init__(self) -> None:
        self.phases: dict[str, str] = {}  # phase_id -> description
        self.current_phase: str | None = None

    def add_phase(self, phase_id: str, description: str) -> None:
        """Add a new phase to track."""
        self.phases[phase_id] = description

    def start_phase(self, phase_id: str) -> None:
        """Mark a phase as running."""
        if phase_id in self.phases:
            self.current_phase = phase_id
            print(f"● {self.phases[phase_id]}...")

    def complete_phase(self, phase_id: str, success: bool = True) -> None:
        """Mark a phase as completed or failed."""
        if phase_id in self.phases:
            if success:
                print("  ✓ Done")
            else:
                print("  ✗ Failed")
            if self.current_phase == phase_id:
                self.current_phase = None

    def skip_phase(self, phase_id: str) -> None:
        """Mark a phase as skipped."""
        if phase_id in self.phases:
            print(f"○ {self.phases[phase_id]} (skipped)")


def run_script(script_name: str, script_dir: Path, env: dict) -> bool:
    """Run a shell script from the helper directory."""
    script_path = script_dir / script_name
    # Explicitly inherit stdout/stderr for proper TUI display
    result = subprocess.run(
        ["bash", str(script_path)],
        cwd=script_dir,
        env=env,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    sys.stdout.flush()
    sys.stderr.flush()
    return result.returncode == 0


def run_python_script(script_name: str, script_dir: Path, env: dict) -> bool:
    """Run a Python script from the helper directory."""
    script_path = script_dir / script_name
    # Explicitly inherit stdout/stderr for proper TUI display
    result = subprocess.run(
        [sys.executable, str(script_path)],
        cwd=script_dir,
        env=env,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    sys.stdout.flush()
    sys.stderr.flush()
    return result.returncode == 0


def count_lines(file_path: Path) -> int:
    """Count non-empty lines in a file."""
    try:
        with open(file_path) as f:
            return sum(1 for line in f if line.strip())
    except FileNotFoundError:
        return 0


def count_files(directory: Path, pattern: str) -> int:
    """Count files matching a pattern in a directory."""
    try:
        return len(list(directory.glob(pattern)))
    except Exception:
        return 0


def write_summary_file(
    log_reports_dir: Path,
    last_failing_phase: str | None,
    successful_pkgs: int,
    failed_pkgs: int,
    skipped_pkgs: int,
    total_pkgs: int,
    output_debs: int,
) -> None:
    """Write summary.txt file to reports directory."""
    summary_file = log_reports_dir / "summary.txt"

    with open(summary_file, "w") as f:
        f.write("=" * 50 + "\n")
        f.write("               BUILD SUMMARY\n")
        f.write("=" * 50 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("\n")

        # Status
        if last_failing_phase:
            f.write("Status: FAILED\n")
            f.write(f"Last failing step: {last_failing_phase}\n")
        else:
            f.write("Status: SUCCESS\n")
        f.write("\n")

        # Package statistics
        f.write("Package Statistics:\n")
        f.write(f"  Total packages:       {total_pkgs}\n")
        f.write(f"  Successfully built:   {successful_pkgs}\n")
        f.write(f"  Skipped (cached):     {skipped_pkgs}\n")
        f.write(f"  Failed:               {failed_pkgs}\n")
        f.write(f"  Output .deb files:    {output_debs}\n")

        # Rates
        attempted_pkgs = successful_pkgs + failed_pkgs
        if attempted_pkgs > 0:
            build_success_rate = successful_pkgs * 100 // attempted_pkgs
            f.write(f"  Build success rate:   {build_success_rate}%\n")

        completed_pkgs = successful_pkgs + skipped_pkgs
        if total_pkgs > 0:
            completion_rate = completed_pkgs * 100 // total_pkgs
            f.write(f"  Overall completion:   {completion_rate}%\n")

        f.write("\n")
        f.write("=" * 50 + "\n")


def print_summary(
    deb_pkgs_file: Path,
    successful_pkgs_file: Path,
    failed_pkgs_file: Path,
    skipped_pkgs_file: Path,
    release_dir: Path,
    log_reports_dir: Path,
    last_failing_phase: str | None = None,
) -> int:
    """Print the build summary, write summary.txt, and return exit code."""
    # Count packages
    total_pkgs = count_lines(deb_pkgs_file)
    successful_pkgs = count_lines(successful_pkgs_file)
    failed_pkgs = count_lines(failed_pkgs_file)
    skipped_pkgs = count_lines(skipped_pkgs_file)
    output_debs = count_files(release_dir, "*.deb")

    # Write summary.txt file
    write_summary_file(
        log_reports_dir=log_reports_dir,
        last_failing_phase=last_failing_phase,
        successful_pkgs=successful_pkgs,
        failed_pkgs=failed_pkgs,
        skipped_pkgs=skipped_pkgs,
        total_pkgs=total_pkgs,
        output_debs=output_debs,
    )

    # Print concise summary
    print("")
    if failed_pkgs > 0:
        print(
            f"Build completed with failures: {successful_pkgs}/{total_pkgs} succeeded, {failed_pkgs} failed"
        )
        # Show failed packages
        try:
            with open(failed_pkgs_file) as f:
                for i, line in enumerate(f):
                    if i >= 5:
                        print(f"  ... and {failed_pkgs - 5} more")
                        break
                    pkg = line.strip()
                    if pkg:
                        print(f"  ✗ {pkg}")
        except FileNotFoundError:
            pass
        return 1
    else:
        print(f"Build completed: {successful_pkgs} packages built, {output_debs} .deb files")
        return 0


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Build Debian packages for ROS packages")
    parser.add_argument(
        "--workspace",
        required=True,
        help="Source workspace directory",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory for build artifacts and .deb files",
    )
    parser.add_argument(
        "--log-dir",
        help="Log directory (created by host if provided)",
    )
    parser.add_argument(
        "--skip-rosdep-install",
        action="store_true",
        help="Do not run rosdep install",
    )
    parser.add_argument(
        "--skip-copy-src",
        action="store_true",
        help="Do not copy source files to the build cache",
    )
    parser.add_argument(
        "--skip-gen-rosdep-list",
        action="store_true",
        help="Do not modify the system rosdep list",
    )
    parser.add_argument(
        "--skip-colcon-build",
        action="store_true",
        help="Do not run colcon build",
    )
    parser.add_argument(
        "--skip-gen-debian",
        action="store_true",
        help="Do not generate Debian metadata (Phase 7)",
    )
    parser.add_argument(
        "--skip-build-deb",
        action="store_true",
        help="Do not build .deb packages (Phase 8)",
    )

    args = parser.parse_args()

    # Initialize UI
    ui = SimpleBuildUI()

    # Add all phases upfront
    ui.add_phase("phase1", "Phase 1: Preparing working directories")
    ui.add_phase("phase2", "Phase 2: Copying source files")
    ui.add_phase("phase3", "Phase 3: Installing dependencies")
    ui.add_phase("phase4", "Phase 4: Compiling packages")
    ui.add_phase("phase5", "Phase 5: Generating rosdep list")
    ui.add_phase("phase6", "Phase 6: Creating package list")
    ui.add_phase("phase7", "Phase 7: Generating Debian metadata")
    ui.add_phase("phase8", "Phase 8: Building Debian packages")

    # Resolve paths
    script_dir = Path(__file__).resolve().parent
    workspace_dir = Path(args.workspace).resolve()
    output_dir = Path(args.output).resolve()

    # Initialize event emitter for TUI communication
    events.init(output_dir)
    events.build_start(total_phases=8)

    # Get ROS distribution and install prefix from environment
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    ros_install_prefix = os.environ.get("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")
    # Optional package suffix (e.g., "1.5.0" for ros-humble-pkg-1.5.0)
    ros_package_suffix = os.environ.get("ROS_PACKAGE_SUFFIX", "")

    # Set up directory paths
    top_work_dir = output_dir
    colcon_work_dir = top_work_dir / "workspace"
    config_dir = Path("/config")
    release_dir = top_work_dir / "debs"
    pkg_build_dir = top_work_dir / "packaging"
    check_dir = release_dir

    # Set up log directory
    if args.log_dir:
        log_dir = Path(args.log_dir)
    else:
        # Create timestamped log directory for standalone usage
        log_base_dir = top_work_dir / "logs"
        log_base_dir.mkdir(parents=True, exist_ok=True)
        log_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = log_base_dir / log_timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        # Create/update 'latest' symlink
        latest_link = log_base_dir / "latest"
        if latest_link.is_symlink() or latest_link.exists():
            latest_link.unlink()
        latest_link.symlink_to(log_timestamp)

    # Create log subdirectories
    log_logs_dir = log_dir / "logs"
    log_reports_dir = log_dir / "reports"
    log_scripts_dir = log_dir / "scripts"
    log_logs_dir.mkdir(parents=True, exist_ok=True)
    log_reports_dir.mkdir(parents=True, exist_ok=True)
    log_scripts_dir.mkdir(parents=True, exist_ok=True)

    # Report file paths (in reports/ subdirectory)
    deb_pkgs_file = log_reports_dir / "packages.txt"
    successful_pkgs_file = log_reports_dir / "successful.txt"
    failed_pkgs_file = log_reports_dir / "failed.txt"
    skipped_pkgs_file = log_reports_dir / "skipped.txt"

    # Prepare environment for shell scripts
    env = os.environ.copy()
    env.update(
        {
            "script_dir": str(script_dir),
            "workspace_dir": str(workspace_dir),
            "output_dir": str(output_dir),
            "top_work_dir": str(top_work_dir),
            "colcon_work_dir": str(colcon_work_dir),
            "config_dir": str(config_dir),
            "release_dir": str(release_dir),
            "pkg_build_dir": str(pkg_build_dir),
            "check_dir": str(check_dir),
            "log_dir": str(log_dir),
            "log_logs_dir": str(log_logs_dir),
            "log_reports_dir": str(log_reports_dir),
            "log_scripts_dir": str(log_scripts_dir),
            "deb_pkgs_file": str(deb_pkgs_file),
            "successful_pkgs_file": str(successful_pkgs_file),
            "failed_pkgs_file": str(failed_pkgs_file),
            "skipped_pkgs_file": str(skipped_pkgs_file),
            "ROS_DISTRO": ros_distro,
            "ROS_INSTALL_PREFIX": ros_install_prefix,
            "ROS_PACKAGE_SUFFIX": ros_package_suffix,
            "rosdep_install": "n" if args.skip_rosdep_install else "y",
            "copy_src": "n" if args.skip_copy_src else "y",
            "gen_rosdep_list": "n" if args.skip_gen_rosdep_list else "y",
            "colcon_build": "n" if args.skip_colcon_build else "y",
            "gen_debian": "n" if args.skip_gen_debian else "y",
            "build_deb": "n" if args.skip_build_deb else "y",
        }
    )

    # Track which phase failed (for summary.txt)
    last_failing_phase: str | None = None

    # Phase 1: Prepare working directories
    ui.start_phase("phase1")
    events.phase_start(1, "Preparing working directories")
    if not run_script("prepare.sh", script_dir, env):
        ui.complete_phase("phase1", success=False)
        events.phase_complete(1, success=False)
        last_failing_phase = "Phase 1: Preparing working directories"
    else:
        ui.complete_phase("phase1", success=True)
        events.phase_complete(1, success=True)

    # Phase 2: Copy source files
    if last_failing_phase is None:
        ui.start_phase("phase2")
        events.phase_start(2, "Copying source files")
        if not run_script("copy-src.sh", script_dir, env):
            ui.complete_phase("phase2", success=False)
            events.phase_complete(2, success=False)
            last_failing_phase = "Phase 2: Copying source files"
        else:
            ui.complete_phase("phase2", success=True)
            events.phase_complete(2, success=True)

    # Phase 3: Install dependencies
    if last_failing_phase is None:
        ui.start_phase("phase3")
        events.phase_start(3, "Installing dependencies")
        if not run_script("install-deps.sh", script_dir, env):
            ui.complete_phase("phase3", success=False)
            events.phase_complete(3, success=False)
            last_failing_phase = "Phase 3: Installing dependencies"
        else:
            ui.complete_phase("phase3", success=True)
            events.phase_complete(3, success=True)

    # Phase 4: Compile packages
    if last_failing_phase is None:
        ui.start_phase("phase4")
        events.phase_start(4, "Compiling packages")
        if not run_script("build-src.sh", script_dir, env):
            ui.complete_phase("phase4", success=False)
            events.phase_complete(4, success=False)
            last_failing_phase = "Phase 4: Compiling packages"
        else:
            ui.complete_phase("phase4", success=True)
            events.phase_complete(4, success=True)

    # Source the setup.bash for subsequent phases
    if last_failing_phase is None:
        setup_bash = colcon_work_dir / "install" / "setup.bash"
        if setup_bash.exists():
            # Capture environment changes from sourcing setup.bash
            result = subprocess.run(
                ["bash", "-c", f"source {setup_bash} && env"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                for line in result.stdout.split("\n"):
                    if "=" in line:
                        key, _, value = line.partition("=")
                        env[key] = value

    # Phase 5: Generate rosdep list
    if last_failing_phase is None:
        ui.start_phase("phase5")
        events.phase_start(5, "Generating rosdep list")
        if not run_script("create-rosdep-list.sh", script_dir, env):
            ui.complete_phase("phase5", success=False)
            events.phase_complete(5, success=False)
            last_failing_phase = "Phase 5: Generating rosdep list"
        else:
            ui.complete_phase("phase5", success=True)
            events.phase_complete(5, success=True)

    # Phase 6: Create package list
    if last_failing_phase is None:
        ui.start_phase("phase6")
        events.phase_start(6, "Creating package list")
        if not run_script("create-package-list.sh", script_dir, env):
            ui.complete_phase("phase6", success=False)
            events.phase_complete(6, success=False)
            last_failing_phase = "Phase 6: Creating package list"
        else:
            ui.complete_phase("phase6", success=True)
            events.phase_complete(6, success=True)

    # Phase 7: Generate Debian metadata
    if last_failing_phase is None:
        if not args.skip_gen_debian:
            ui.start_phase("phase7")
            events.phase_start(7, "Generating Debian metadata")
            if not run_python_script("generate_debian_dir.py", script_dir, env):
                ui.complete_phase("phase7", success=False)
                events.phase_complete(7, success=False)
                last_failing_phase = "Phase 7: Generating Debian metadata"
            else:
                ui.complete_phase("phase7", success=True)
                events.phase_complete(7, success=True)
        else:
            ui.skip_phase("phase7")
            events.phase_skip(7)

    # Phase 8: Build Debian packages
    if last_failing_phase is None:
        if not args.skip_build_deb:
            ui.start_phase("phase8")
            events.phase_start(8, "Building Debian packages")
            if not run_python_script("build_deb.py", script_dir, env):
                # build_deb.py may return non-zero if some packages fail
                # but we still want to show the summary (packages may have partial success)
                ui.complete_phase("phase8", success=True)  # Partial success is still "done"
                events.phase_complete(8, success=True)
            else:
                ui.complete_phase("phase8", success=True)
                events.phase_complete(8, success=True)
        else:
            ui.skip_phase("phase8")
            events.phase_skip(8)

    # Emit build complete event
    events.build_complete(success=(last_failing_phase is None))

    # Print summary (always runs, even on failure, to generate summary.txt)
    return print_summary(
        deb_pkgs_file=deb_pkgs_file,
        successful_pkgs_file=successful_pkgs_file,
        failed_pkgs_file=failed_pkgs_file,
        skipped_pkgs_file=skipped_pkgs_file,
        release_dir=release_dir,
        log_reports_dir=log_reports_dir,
        last_failing_phase=last_failing_phase,
    )


if __name__ == "__main__":
    sys.exit(main())
