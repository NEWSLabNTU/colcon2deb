"""Build Debian packages from generated metadata."""

import os
import subprocess
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import List, Tuple, Dict

from .config import BuildConfig
from .utils import logger, print_phase, ensure_dir


class PackageBuildResult:
    """Result of building a single package."""

    def __init__(self, pkg_name: str, success: bool, deb_file: Path = None, error: str = ""):
        self.pkg_name = pkg_name
        self.success = success
        self.deb_file = deb_file
        self.error = error


def build_single_package(pkg_name: str, config: BuildConfig) -> PackageBuildResult:
    """Build a single Debian package.

    Args:
        pkg_name: Name of the package to build
        config: Build configuration

    Returns:
        PackageBuildResult with build outcome
    """
    pkg_work_dir = config.get_pkg_work_dir(pkg_name)
    debian_dir = pkg_work_dir / "debian"

    out_file = pkg_work_dir / "build.out"
    err_file = pkg_work_dir / "build.err"

    # Check if debian directory exists
    if not debian_dir.exists():
        return PackageBuildResult(
            pkg_name, False, error="Debian directory not found"
        )

    # Check if package already built (skip if .deb exists in release dir)
    deb_pattern = f"*{pkg_name.replace('_', '-')}*.deb"
    existing_debs = list(config.release_dir.glob(deb_pattern))

    if existing_debs:
        logger.debug(f"  Skipping {pkg_name} (already built)")
        with open(config.skipped_pkgs_file, 'a') as f:
            f.write(f"{pkg_name}\n")
        return PackageBuildResult(pkg_name, True, deb_file=existing_debs[0])

    # Copy package source to work directory (bloom expects source + debian together)
    # Find the package in the workspace
    # Try direct path first (src/pkg_name), then nested (src/*/pkg_name)
    package_list = list(config.colcon_work_dir.glob(f"src/{pkg_name}"))
    if not package_list:
        package_list = list(config.colcon_work_dir.glob(f"src/*/{pkg_name}"))

    if package_list:
        pkg_src_dir = package_list[0]
        logger.info(f"  Copying source from {pkg_src_dir} to {pkg_work_dir}")

        # Copy all source files except build artifacts
        rsync_cmd = [
            "rsync", "-av",
            "--exclude=build", "--exclude=install", "--exclude=log",
            "--exclude=debian",  # Don't overwrite bloom-generated debian directory
            f"{pkg_src_dir}/", str(pkg_work_dir) + "/"
        ]
        result = subprocess.run(rsync_cmd, check=True, capture_output=True, text=True)
        logger.info(f"  rsync output: {result.stdout[:500]}")
        if result.stderr:
            logger.warning(f"  rsync warnings: {result.stderr[:500]}")
    else:
        logger.warning(f"  Could not find package source for {pkg_name}")

    # Build the package using fakeroot debian/rules binary
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        cd {pkg_work_dir}
        fakeroot debian/rules binary
        """
    ]

    try:
        with open(out_file, 'w') as out, open(err_file, 'w') as err:
            result = subprocess.run(
                cmd,
                stdout=out,
                stderr=err,
                check=True,
                timeout=1800,  # 30 minute timeout per package
            )

        # Find generated .deb file
        deb_files = list(pkg_work_dir.parent.glob(f"*{pkg_name.replace('_', '-')}*.deb"))

        if deb_files:
            # Move .deb to release directory
            deb_file = deb_files[0]
            dest_file = config.release_dir / deb_file.name
            deb_file.rename(dest_file)

            # Log success
            with open(config.successful_pkgs_file, 'a') as f:
                f.write(f"{pkg_name}\n")
            with open(config.deb_pkgs_file, 'a') as f:
                f.write(f"{dest_file}\n")

            logger.debug(f"  ✓ Built {pkg_name}")
            return PackageBuildResult(pkg_name, True, deb_file=dest_file)
        else:
            error = "No .deb file generated"
            with open(config.failed_pkgs_file, 'a') as f:
                f.write(f"{pkg_name}\n")
            return PackageBuildResult(pkg_name, False, error=error)

    except subprocess.TimeoutExpired:
        error = "Build timeout (30 minutes)"
        logger.error(f"  ✗ Timeout: {pkg_name}")
        with open(config.failed_pkgs_file, 'a') as f:
            f.write(f"{pkg_name}\n")
        return PackageBuildResult(pkg_name, False, error=error)

    except subprocess.CalledProcessError as e:
        # Read error from log file
        error_msg = ""
        if err_file.exists():
            error_msg = err_file.read_text()[-500:]  # Last 500 chars

        logger.error(f"  ✗ Failed: {pkg_name}")
        with open(config.failed_pkgs_file, 'a') as f:
            f.write(f"{pkg_name}\n")

        return PackageBuildResult(pkg_name, False, error=error_msg)


def build_packages_parallel(config: BuildConfig) -> Dict[str, PackageBuildResult]:
    """Build all Debian packages in parallel.

    Uses ProcessPoolExecutor for CPU-intensive package building.
    Limits parallelism to 1/4 of CPU cores to avoid resource exhaustion.

    Returns:
        Dictionary mapping package names to build results
    """
    print_phase("Phase 8: Building Debian packages")
    logger.info("info: build Debian packages")

    # Get list of packages from colcon workspace
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        source {config.colcon_work_dir}/install/setup.bash
        cd {config.colcon_work_dir}
        colcon list --base-paths src | cut -f1
        """
    ]

    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    packages = [line.strip() for line in result.stdout.split('\n') if line.strip()]

    logger.info(f"  Building {len(packages)} packages")

    # Use 1/4 of CPU cores for package building to prevent resource exhaustion
    max_workers = max(1, os.cpu_count() // 4)
    logger.info(f"  Using {max_workers} parallel workers")

    results = {}
    succeeded = 0
    failed = 0
    skipped = 0

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(build_single_package, pkg_name, config): pkg_name
            for pkg_name in packages
        }

        completed = 0
        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                result = future.result()
                results[pkg_name] = result

                completed += 1
                if result.success:
                    if result.deb_file and result.deb_file.exists():
                        # Check if it was skipped or built
                        with open(config.skipped_pkgs_file, 'r') as f:
                            if pkg_name in f.read():
                                skipped += 1
                            else:
                                succeeded += 1
                    else:
                        succeeded += 1
                else:
                    failed += 1

                # Progress update every 10 packages
                if completed % 10 == 0:
                    logger.info(f"  Progress: {completed}/{len(packages)} packages processed")

            except Exception as e:
                logger.error(f"  ✗ Exception building {pkg_name}: {e}")
                results[pkg_name] = PackageBuildResult(pkg_name, False, error=str(e))
                failed += 1

    logger.info(f"  ✓ Build complete: {succeeded} succeeded, {skipped} skipped, {failed} failed")
    return results


def print_build_summary(config: BuildConfig, results: Dict[str, PackageBuildResult]):
    """Print detailed build summary."""
    print("")
    print("=" * 60)
    print("  Build Summary")
    print("=" * 60)

    total = len(results)
    succeeded = sum(1 for r in results.values() if r.success)
    failed = sum(1 for r in results.values() if not r.success)

    # Read skipped count from file
    skipped_count = 0
    if config.skipped_pkgs_file.exists():
        with open(config.skipped_pkgs_file, 'r') as f:
            skipped_count = len([line for line in f if line.strip()])

    print(f"  Total packages: {total}")
    print(f"  Succeeded: {succeeded}")
    print(f"  Skipped (already built): {skipped_count}")
    print(f"  Failed: {failed}")
    print("")

    if failed > 0:
        print("  Failed packages:")
        for pkg_name, result in results.items():
            if not result.success:
                print(f"    - {pkg_name}")
                if result.error:
                    error_preview = result.error[:100].replace('\n', ' ')
                    print(f"      Error: {error_preview}")
        print("")

    print(f"  Debian packages: {config.release_dir}")
    print(f"  Build logs: {config.log_dir}")
    print("=" * 60)
    print("")
