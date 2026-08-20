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
import threading
from datetime import datetime
from pathlib import Path

import events
from colcon_events import snapshot_build_dirs


class SimpleBuildUI:
    """Simple UI for build progress display."""

    def __init__(self) -> None:
        self.phases: dict[str, str] = {}
        self.current_phase: str | None = None

    def add_phase(self, phase_id: str, description: str) -> None:
        self.phases[phase_id] = description

    def start_phase(self, phase_id: str) -> None:
        if phase_id in self.phases:
            self.current_phase = phase_id
            print(f"● {self.phases[phase_id]}...")

    def complete_phase(self, phase_id: str, success: bool = True) -> None:
        if phase_id in self.phases:
            if success:
                print("  ✓ Done")
            else:
                print("  ✗ Failed")
            if self.current_phase == phase_id:
                self.current_phase = None

    def skip_phase(self, phase_id: str) -> None:
        if phase_id in self.phases:
            print(f"○ {self.phases[phase_id]} (skipped)")


def run_script(
    script_name: str,
    script_dir: Path,
    env: dict[str, str],
    log_file: Path | None = None,
) -> bool:
    """Run a shell script, tee-ing output to both terminal and log file."""
    script_path = script_dir / script_name
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        with open(log_file, "w") as lf:
            process = subprocess.Popen(
                ["bash", str(script_path)],
                cwd=script_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                bufsize=1,
            )
            if process.stdout:
                for line in process.stdout:
                    sys.stdout.write(line)
                    lf.write(line)
            process.wait()
            sys.stdout.flush()
            return process.returncode == 0
    else:
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


def run_python_script(
    script_name: str,
    script_dir: Path,
    env: dict[str, str],
    log_file: Path | None = None,
) -> bool:
    """Run a Python script, tee-ing output to both terminal and log file."""
    script_path = script_dir / script_name
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        with open(log_file, "w") as lf:
            process = subprocess.Popen(
                [sys.executable, str(script_path)],
                cwd=script_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                bufsize=1,
            )
            if process.stdout:
                for line in process.stdout:
                    sys.stdout.write(line)
                    lf.write(line)
            process.wait()
            sys.stdout.flush()
            return process.returncode == 0
    else:
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
    try:
        with open(file_path) as f:
            return sum(1 for line in f if line.strip())
    except FileNotFoundError:
        return 0


def count_files(directory: Path, pattern: str) -> int:
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
    summary_file = log_reports_dir / "summary.txt"
    with open(summary_file, "w") as f:
        f.write("=" * 50 + "\n")
        f.write("               BUILD SUMMARY\n")
        f.write("=" * 50 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        if last_failing_phase:
            f.write("Status: FAILED\n")
            f.write(f"Last failing step: {last_failing_phase}\n")
        elif failed_pkgs > 0:
            f.write("Status: FAILED\n")
            f.write(f"Last failing step: {failed_pkgs} package(s) failed to build\n")
        else:
            f.write("Status: SUCCESS\n")
        f.write("\n")

        f.write("Package Statistics:\n")
        f.write(f"  Total packages:       {total_pkgs}\n")
        f.write(f"  Successfully built:   {successful_pkgs}\n")
        f.write(f"  Skipped (cached):     {skipped_pkgs}\n")
        f.write(f"  Failed:               {failed_pkgs}\n")
        f.write(f"  Output .deb files:    {output_debs}\n")

        attempted_pkgs = successful_pkgs + failed_pkgs
        if attempted_pkgs > 0:
            f.write(f"  Build success rate:   {successful_pkgs * 100 // attempted_pkgs}%\n")

        completed_pkgs = successful_pkgs + skipped_pkgs
        if total_pkgs > 0:
            f.write(f"  Overall completion:   {completed_pkgs * 100 // total_pkgs}%\n")

        f.write("\n" + "=" * 50 + "\n")


def print_summary(
    deb_pkgs_file: Path,
    successful_pkgs_file: Path,
    failed_pkgs_file: Path,
    skipped_pkgs_file: Path,
    release_dir: Path,
    log_reports_dir: Path,
    last_failing_phase: str | None = None,
) -> int:
    total_pkgs = count_lines(deb_pkgs_file)
    successful_pkgs = count_lines(successful_pkgs_file)
    failed_pkgs = count_lines(failed_pkgs_file)
    skipped_pkgs = count_lines(skipped_pkgs_file)
    output_debs = count_files(release_dir, "*.deb")

    write_summary_file(
        log_reports_dir=log_reports_dir,
        last_failing_phase=last_failing_phase,
        successful_pkgs=successful_pkgs,
        failed_pkgs=failed_pkgs,
        skipped_pkgs=skipped_pkgs,
        total_pkgs=total_pkgs,
        output_debs=output_debs,
    )

    print("")
    if last_failing_phase:
        print(f"Build failed at {last_failing_phase}")
        print(f"See logs in {log_reports_dir.parent}")
        return 1
    if failed_pkgs > 0:
        print(
            f"Build completed with failures: {successful_pkgs} built, "
            f"{skipped_pkgs} cached, {failed_pkgs} failed"
        )
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
        print(
            f"Build completed: {successful_pkgs} built, {skipped_pkgs} cached, "
            f"{output_debs} .deb files"
        )
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Debian packages for ROS packages")
    parser.add_argument("--workspace", required=True, help="Source workspace directory")
    parser.add_argument("--output", required=True, help="Output directory for build artifacts")
    parser.add_argument("--log-dir", help="Log directory (created by host if provided)")
    parser.add_argument("--skip-rosdep-install", action="store_true")
    parser.add_argument("--skip-copy-src", action="store_true")
    parser.add_argument("--skip-gen-rosdep-list", action="store_true")
    parser.add_argument("--skip-colcon-build", action="store_true")
    parser.add_argument("--skip-gen-debian", action="store_true")
    parser.add_argument("--skip-build-deb", action="store_true")

    args = parser.parse_args()

    # Initialize UI
    ui = SimpleBuildUI()
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

    # Initialize event emitter
    events.init(output_dir)
    events.build_start(total_phases=8)

    # Get ROS configuration from environment
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    ros_install_prefix = os.environ.get("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")
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
        log_base_dir = top_work_dir / "logs"
        log_base_dir.mkdir(parents=True, exist_ok=True)
        log_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = log_base_dir / log_timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        latest_link = log_base_dir / "latest"
        if latest_link.is_symlink() or latest_link.exists():
            latest_link.unlink()
        latest_link.symlink_to(log_timestamp)

    # Create log subdirectories (new structure)
    log_phases_dir = log_dir / "phases"
    log_packages_dir = log_dir / "packages"
    log_reports_dir = log_dir / "reports"
    log_scripts_dir = log_dir / "scripts"
    log_phases_dir.mkdir(parents=True, exist_ok=True)
    log_packages_dir.mkdir(parents=True, exist_ok=True)
    log_reports_dir.mkdir(parents=True, exist_ok=True)
    log_scripts_dir.mkdir(parents=True, exist_ok=True)

    # Report file paths
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
            "log_phases_dir": str(log_phases_dir),
            "log_packages_dir": str(log_packages_dir),
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

    last_failing_phase: str | None = None

    def run_phase(
        phase_num: int,
        description: str,
        script: str,
        *,
        is_python: bool = False,
    ) -> bool:
        """Run a build phase with logging and event emission. Returns False if failed."""
        nonlocal last_failing_phase
        if last_failing_phase is not None:
            return False

        phase_id = f"phase{phase_num}"
        log_file = (
            log_phases_dir
            / f"phase{phase_num}_{script.removesuffix('.sh').removesuffix('.py').replace('-', '_')}.log"
        )

        ui.start_phase(phase_id)
        events.phase_start(phase_num, description)

        runner = run_python_script if is_python else run_script
        success = runner(script, script_dir, env, log_file=log_file)

        if not success:
            ui.complete_phase(phase_id, success=False)
            events.phase_complete(phase_num, success=False)
            last_failing_phase = f"Phase {phase_num}: {description}"
            return False

        ui.complete_phase(phase_id, success=True)
        events.phase_complete(phase_num, success=True)
        return True

    # Phase 1-3: Setup
    run_phase(1, "Preparing working directories", "prepare.sh")
    run_phase(2, "Copying source files", "copy-src.sh")
    run_phase(3, "Installing dependencies", "install-deps.sh")

    # Phase 4: Compiling packages.
    #
    # Pipelined mode (default): colcon runs in a background thread while
    # phases 5-7 proceed — they need only the base ROS environment, not
    # the workspace build. Phase 8 then gates each package's .deb build on
    # colcon having finished that package (its dependencies are complete
    # by then; the dh build recompiles the package itself from source and
    # resolves dependencies by sourcing local_setup.bash at build time).
    pipeline = (
        env.get("COLCON2DEB_PIPELINE", "1") == "1"
        and not args.skip_colcon_build
        and last_failing_phase is None
    )

    colcon_status_file = output_dir / ".colcon.status"
    colcon_status_file.unlink(missing_ok=True)
    colcon_result: dict[str, bool] = {}
    colcon_thread: threading.Thread | None = None

    def run_colcon_phase() -> None:
        phase_id = "phase4"
        log_file = log_phases_dir / "phase4_build_src.log"
        ui.start_phase(phase_id)
        events.phase_start(4, "Compiling packages")
        ok = False
        try:
            ok = run_script("build-src.sh", script_dir, env, log_file=log_file)
        finally:
            # The status file must always be written — the phase-8 gate
            # blocks on it if colcon dies without per-package events.
            ui.complete_phase(phase_id, success=ok)
            events.phase_complete(4, success=ok)
            colcon_result["ok"] = ok
            try:
                colcon_status_file.write_text("0" if ok else "1")
            except OSError:
                pass

    if pipeline:
        # Snapshot colcon's log dirs before launching so the phase-8 gate
        # can identify this run's events.log among previous runs'.
        exclude_dirs = snapshot_build_dirs(colcon_work_dir / "log")
        env["COLCON2DEB_PIPELINE"] = "1"
        env["COLCON2DEB_COLCON_LOG_BASE"] = str(colcon_work_dir / "log")
        env["COLCON2DEB_COLCON_STATUS_FILE"] = str(colcon_status_file)
        env["COLCON2DEB_COLCON_EXCLUDE"] = ",".join(sorted(exclude_dirs))
        colcon_thread = threading.Thread(target=run_colcon_phase, daemon=True)
        colcon_thread.start()
    else:
        env["COLCON2DEB_PIPELINE"] = "0"
        if last_failing_phase is None:
            run_colcon_phase()
            if colcon_result.get("ok") is False:
                last_failing_phase = "Phase 4: Compiling packages"

    # Phase 5-6: Dependency resolution (source-only; no build needed)
    run_phase(5, "Generating rosdep list", "create-rosdep-list.sh")
    run_phase(6, "Creating package list", "create-package-list.sh")

    # Phase 7: Generate Debian metadata
    if last_failing_phase is None:
        if not args.skip_gen_debian:
            run_phase(7, "Generating Debian metadata", "generate_debian_dir.py", is_python=True)
        else:
            ui.skip_phase("phase7")
            events.phase_skip(7)

    # Phase 8: Build Debian packages
    if last_failing_phase is None:
        if not args.skip_build_deb:
            phase_id = "phase8"
            log_file = log_phases_dir / "phase8_build_deb.log"
            ui.start_phase(phase_id)
            events.phase_start(8, "Building Debian packages")
            # build_deb.py returns non-zero for partial failures; per-package
            # failures are reported via failed.txt in the summary below.
            phase8_ok = run_python_script("build_deb.py", script_dir, env, log_file=log_file)
            ui.complete_phase(phase_id, success=phase8_ok)
            events.phase_complete(8, success=phase8_ok)
            if not phase8_ok and count_lines(failed_pkgs_file) == 0:
                # build_deb.py died before recording any package failure
                last_failing_phase = "Phase 8: Building Debian packages"
        else:
            ui.skip_phase("phase8")
            events.phase_skip(8)

    # Join the colcon thread (pipelined mode); phase 8's per-package gate
    # already blocked on colcon results, so this normally returns at once.
    if colcon_thread is not None:
        colcon_thread.join()
    if colcon_result.get("ok") is False and last_failing_phase is None:
        last_failing_phase = "Phase 4: Compiling packages"
    if colcon_result.get("ok") and not (colcon_work_dir / "install" / "setup.bash").exists():
        print(
            "warning: colcon build succeeded but install/setup.bash is missing",
            file=sys.stderr,
        )

    events.build_complete(success=(last_failing_phase is None))

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
