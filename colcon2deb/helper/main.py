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


def print_phase(message: str) -> None:
    """Print a timestamped phase message."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"\n[{timestamp}] {message}")


def run_script(script_name: str, script_dir: Path, env: dict) -> bool:
    """Run a shell script from the helper directory."""
    script_path = script_dir / script_name
    result = subprocess.run(
        ["bash", str(script_path)],
        cwd=script_dir,
        env=env,
    )
    return result.returncode == 0


def run_python_script(script_name: str, script_dir: Path, env: dict) -> bool:
    """Run a Python script from the helper directory."""
    script_path = script_dir / script_name
    result = subprocess.run(
        [sys.executable, str(script_path)],
        cwd=script_dir,
        env=env,
    )
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
    log_dir: Path,
    last_failing_phase: str | None,
    successful_pkgs: int,
    failed_pkgs: int,
    skipped_pkgs: int,
    total_pkgs: int,
    output_debs: int,
) -> None:
    """Write summary.txt file to log directory."""
    summary_file = log_dir / "summary.txt"

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
    pkg_build_dir: Path,
    top_work_dir: Path,
    log_dir: Path,
    workspace_dir: Path,
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
        log_dir=log_dir,
        last_failing_phase=last_failing_phase,
        successful_pkgs=successful_pkgs,
        failed_pkgs=failed_pkgs,
        skipped_pkgs=skipped_pkgs,
        total_pkgs=total_pkgs,
        output_debs=output_debs,
    )

    # Continue with original summary display (using counts computed above)

    print("")
    print("=" * 47)
    print("           BUILD SUMMARY                      ")
    print("=" * 47)

    print("")
    print("Package Statistics:")
    print(f"  Total packages found:      {total_pkgs}")
    print(f"  Previously built:          {skipped_pkgs} (skipped)")
    print(f"  Successfully built:        {successful_pkgs} (new)")
    print(f"  Failed to build:           {failed_pkgs}")
    print(f"  Output .deb files:         {output_debs}")

    # Calculate build success rate
    attempted_pkgs = successful_pkgs + failed_pkgs
    if attempted_pkgs > 0:
        build_success_rate = successful_pkgs * 100 // attempted_pkgs
        print(f"  Build success rate:        {build_success_rate}% (of {attempted_pkgs} attempted)")

    # Calculate overall completion rate
    completed_pkgs = successful_pkgs + skipped_pkgs
    if total_pkgs > 0:
        completion_rate = completed_pkgs * 100 // total_pkgs
        print(f"  Overall completion:        {completion_rate}% ({completed_pkgs}/{total_pkgs})")

    # Sanity check
    accounted_pkgs = successful_pkgs + failed_pkgs + skipped_pkgs
    if accounted_pkgs != total_pkgs:
        unaccounted = total_pkgs - accounted_pkgs
        print(f"  Unaccounted packages:      {unaccounted}")

    print("")
    print("Directories:")
    print(f"  Output directory:   {top_work_dir}")
    print(f"  Packages directory: {release_dir}")
    print(f"  Log directory:      {log_dir}")

    # Show failed packages if any
    if failed_pkgs > 0:
        print("")
        print("Failed Packages (first 10):")
        try:
            with open(failed_pkgs_file) as f:
                for i, line in enumerate(f):
                    if i >= 10:
                        break
                    pkg = line.strip()
                    if pkg:
                        print(f"  - {pkg}")
                        err_file = pkg_build_dir / pkg / "build.err"
                        if err_file.exists() and err_file.stat().st_size > 0:
                            print(f"    -> Error log: {err_file}")
                            # Print last 10 lines of error log for immediate visibility
                            try:
                                err_content = err_file.read_text().strip().split("\n")
                                last_lines = err_content[-10:] if len(err_content) > 10 else err_content
                                for line in last_lines:
                                    print(f"       {line}")
                            except Exception:
                                pass
        except FileNotFoundError:
            pass

        if failed_pkgs > 10:
            print(f"  ... and {failed_pkgs - 10} more")

        print("")
        print("To investigate failures:")
        print("  1. Check individual error logs:")
        print(f"     cat {pkg_build_dir}/PACKAGE_NAME/build.err")
        print("  2. View complete failed list:")
        print(f"     cat {failed_pkgs_file}")
        print("  3. Run diagnostic tool:")
        print(f"     ./check-build-results.py --workspace {workspace_dir} --output {top_work_dir}")

    print("")
    print("=" * 47)

    # Exit with appropriate code
    if failed_pkgs > 0:
        print("Build completed with failures")
        print("")
        print("Next steps:")
        print("  1. Check build logs for failed packages")
        print("  2. Try installing successfully built packages:")
        print("     ./install-partial.py --config CONFIG --mode safe")
        return 1
    else:
        print("Build completed successfully!")
        print("")
        print("Next steps:")
        print("  1. Install packages:")
        print("     ./install-packages.py --config CONFIG")
        print("  2. Test installation:")
        print("     ./test-installation.sh --config CONFIG")
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

    # Resolve paths
    script_dir = Path(__file__).resolve().parent
    workspace_dir = Path(args.workspace).resolve()
    output_dir = Path(args.output).resolve()

    # Get ROS distribution and install prefix from environment
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    ros_install_prefix = os.environ.get("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}")

    print_phase("Build Configuration")
    print(f"  ROS Distribution: {ros_distro}")
    print(f"  Install Prefix: {ros_install_prefix}")
    # Optional package suffix (e.g., "1.5.0" for ros-humble-pkg-1.5.0)
    ros_package_suffix = os.environ.get("ROS_PACKAGE_SUFFIX", "")

    # Set up directory paths
    top_work_dir = output_dir
    colcon_work_dir = top_work_dir / "sources"
    config_dir = Path("/config")
    release_dir = top_work_dir / "dist"
    pkg_build_dir = top_work_dir / "build"
    check_dir = release_dir

    # Set up log directory
    if args.log_dir:
        log_dir = Path(args.log_dir)
    else:
        # Create timestamped log directory for standalone usage
        log_base_dir = top_work_dir / "log"
        log_base_dir.mkdir(parents=True, exist_ok=True)
        log_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = log_base_dir / log_timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        # Create/update 'latest' symlink
        latest_link = log_base_dir / "latest"
        if latest_link.is_symlink() or latest_link.exists():
            latest_link.unlink()
        latest_link.symlink_to(log_timestamp)

    # Log file paths (no number prefixes for outcome files)
    deb_pkgs_file = log_dir / "deb_pkgs.txt"
    successful_pkgs_file = log_dir / "successful_pkgs.txt"
    failed_pkgs_file = log_dir / "failed_pkgs.txt"
    skipped_pkgs_file = log_dir / "skipped_pkgs.txt"

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
    print_phase("Phase 1: Preparing working directories")
    if not run_script("prepare.sh", script_dir, env):
        print("error: Phase 1 failed", file=sys.stderr)
        last_failing_phase = "Phase 1: Preparing working directories"

    # Phase 2: Copy source files
    if last_failing_phase is None:
        print_phase("Phase 2: Copying source files")
        if not run_script("copy-src.sh", script_dir, env):
            print("error: Phase 2 failed", file=sys.stderr)
            last_failing_phase = "Phase 2: Copying source files"

    # Phase 3: Install dependencies
    if last_failing_phase is None:
        print_phase("Phase 3: Installing dependencies")
        if not run_script("install-deps.sh", script_dir, env):
            print("error: Phase 3 failed", file=sys.stderr)
            last_failing_phase = "Phase 3: Installing dependencies"

    # Phase 4: Compile packages
    if last_failing_phase is None:
        print_phase("Phase 4: Compiling packages")
        if not run_script("build-src.sh", script_dir, env):
            print("error: Phase 4 failed", file=sys.stderr)
            last_failing_phase = "Phase 4: Compiling packages"

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

            # Ensure /bloom is at the FRONT of PYTHONPATH for bloom_gen imports
            pythonpath = env.get("PYTHONPATH", "")
            if "/bloom" not in pythonpath:
                env["PYTHONPATH"] = f"/bloom:{pythonpath}" if pythonpath else "/bloom"
            elif not pythonpath.startswith("/bloom"):
                # /bloom is present but not at front - move it to front
                paths = pythonpath.split(":")
                paths = [p for p in paths if p != "/bloom"]
                env["PYTHONPATH"] = "/bloom:" + ":".join(paths)

    # Phase 5: Generate rosdep list
    if last_failing_phase is None:
        print_phase("Phase 5: Generating rosdep list")
        if not run_script("create-rosdep-list.sh", script_dir, env):
            print("error: Phase 5 failed", file=sys.stderr)
            last_failing_phase = "Phase 5: Generating rosdep list"

    # Phase 6: Create package list
    if last_failing_phase is None:
        print_phase("Phase 6: Creating package list")
        if not run_script("create-package-list.sh", script_dir, env):
            print("error: Phase 6 failed", file=sys.stderr)
            last_failing_phase = "Phase 6: Creating package list"

    # Phase 7: Generate Debian metadata
    if last_failing_phase is None:
        if not args.skip_gen_debian:
            print_phase("Phase 7: Generating Debian metadata")
            if not run_python_script("generate_debian_dir.py", script_dir, env):
                print("error: Phase 7 failed", file=sys.stderr)
                last_failing_phase = "Phase 7: Generating Debian metadata"
        else:
            print_phase("Phase 7: Skipping Debian metadata generation (--skip-gen-debian)")

    # Phase 8: Build Debian packages
    if last_failing_phase is None:
        if not args.skip_build_deb:
            print_phase("Phase 8: Building Debian packages")
            if not run_python_script("build_deb.py", script_dir, env):
                # build_deb.py may return non-zero if some packages fail
                # but we still want to show the summary (packages may have partial success)
                pass
        else:
            print_phase("Phase 8: Skipping Debian package builds (--skip-build-deb)")

    if last_failing_phase is None:
        print(f"Packages are in: {release_dir}")

    # Print summary (always runs, even on failure, to generate summary.txt)
    return print_summary(
        deb_pkgs_file=deb_pkgs_file,
        successful_pkgs_file=successful_pkgs_file,
        failed_pkgs_file=failed_pkgs_file,
        skipped_pkgs_file=skipped_pkgs_file,
        release_dir=release_dir,
        pkg_build_dir=pkg_build_dir,
        top_work_dir=top_work_dir,
        log_dir=log_dir,
        workspace_dir=workspace_dir,
        last_failing_phase=last_failing_phase,
    )


if __name__ == "__main__":
    sys.exit(main())
