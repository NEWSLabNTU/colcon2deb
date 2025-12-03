"""Build Debian packages from generated metadata."""

import os
import subprocess
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import List, Tuple, Dict, Set, Optional
import sys

from .config import BuildConfig
from .utils import logger, print_phase, ensure_dir, add_subtask, get_display


# Cache for package dependencies to avoid repeated XML parsing
_dependency_cache: Dict[str, Set[str]] = {}

# Cache for package CMake detection
_cmake_package_cache: Dict[str, bool] = {}

# Cache for package name to directory mapping
_package_name_map: Dict[str, Path] = {}

# Log file for Debian package building (set during build_packages_parallel)
_debian_build_log: Optional[Path] = None


def log_debian_build(message: str, level: str = "info"):
    """Log a message to both the logger and the Debian build log file.

    Args:
        message: The message to log
        level: Log level (info, warning, error, debug)
    """
    global _debian_build_log

    # Log to regular logger
    getattr(logger, level)(message)

    # Also log to dedicated Debian build log file
    if _debian_build_log:
        try:
            with open(_debian_build_log, 'a') as f:
                f.write(f"[{level.upper()}] {message}\n")
        except Exception:
            pass  # Don't fail on logging errors


def build_package_name_map(config: BuildConfig) -> Dict[str, Path]:
    """Build mapping of package names to their directories.

    Some packages have directory names that differ from their package name
    (e.g., boost_io_context lives in io_context/).
    This scans all package.xml files to build a correct mapping.

    Args:
        config: Build configuration

    Returns:
        Dict mapping package names to their directory paths
    """
    global _package_name_map

    if _package_name_map:
        return _package_name_map

    logger.debug("Building package name map...")
    for pkg_xml in config.colcon_work_dir.glob("src/**/package.xml"):
        try:
            tree = ET.parse(pkg_xml)
            name_elem = tree.find("name")
            if name_elem is not None and name_elem.text:
                pkg_name = name_elem.text.strip()
                _package_name_map[pkg_name] = pkg_xml.parent
        except Exception as e:
            logger.debug(f"Error parsing {pkg_xml}: {e}")

    logger.debug(f"Found {len(_package_name_map)} packages")
    return _package_name_map


def get_package_path(pkg_name: str, config: BuildConfig) -> Optional[Path]:
    """Get the source directory path for a package.

    Uses the package name map to handle cases where directory name
    differs from package name.

    Args:
        pkg_name: Name of the package
        config: Build configuration

    Returns:
        Path to the package directory, or None if not found
    """
    pkg_map = build_package_name_map(config)
    return pkg_map.get(pkg_name)


class PackageBuildResult:
    """Result of building a single package."""

    def __init__(self, pkg_name: str, success: bool, deb_file: Path = None, error: str = ""):
        self.pkg_name = pkg_name
        self.success = success
        self.deb_file = deb_file
        self.error = error


def get_package_dependencies(pkg_name: str, config: BuildConfig) -> Set[str]:
    """Extract dependencies from package.xml.

    Args:
        pkg_name: Name of the package
        config: Build configuration

    Returns:
        Set of dependency package names
    """
    global _dependency_cache

    if pkg_name in _dependency_cache:
        return _dependency_cache[pkg_name]

    # Use package name map to find the package path
    pkg_path = get_package_path(pkg_name, config)
    if not pkg_path:
        _dependency_cache[pkg_name] = set()
        return set()

    pkg_xml_file = pkg_path / "package.xml"
    if not pkg_xml_file.exists():
        _dependency_cache[pkg_name] = set()
        return set()

    try:
        tree = ET.parse(pkg_xml_file)
        root = tree.getroot()
        deps = set()

        # Extract all dependency types
        for dep_tag in ['depend', 'build_depend', 'exec_depend', 'build_export_depend']:
            for dep in root.findall(dep_tag):
                if dep.text:
                    deps.add(dep.text.strip())

        _dependency_cache[pkg_name] = deps
        return deps
    except ET.ParseError as e:
        logger.warning(f"Failed to parse package.xml for {pkg_name}: {e}")
        _dependency_cache[pkg_name] = set()
        return set()


def build_dependency_graph(packages: List[str], config: BuildConfig) -> Dict[str, Set[str]]:
    """Build dependency graph for all packages.

    Args:
        packages: List of package names
        config: Build configuration

    Returns:
        Dictionary mapping package names to their internal dependencies
    """
    graph = {}
    pkg_set = set(packages)

    for pkg in packages:
        deps = get_package_dependencies(pkg, config)
        # Only include dependencies that are in our package list
        graph[pkg] = deps & pkg_set

    return graph


def topological_sort_packages(packages: List[str], config: BuildConfig) -> List[List[str]]:
    """Sort packages into build levels based on dependencies.

    Returns list of package groups where each group can be built in parallel
    after all previous groups complete.

    Args:
        packages: List of package names
        config: Build configuration

    Returns:
        List of build levels, each level is a list of packages
    """
    graph = build_dependency_graph(packages, config)

    # Calculate in-degrees (number of dependencies within our package set)
    in_degree = {}
    for pkg in packages:
        in_degree[pkg] = len(graph.get(pkg, set()))

    levels = []
    remaining = set(packages)
    built = set()

    while remaining:
        # Find all packages with no unbuilt dependencies
        level = [
            pkg for pkg in remaining
            if all(dep in built or dep not in remaining for dep in graph.get(pkg, set()))
        ]

        if not level:
            # Circular dependency detected - add remaining as final level
            logger.warning(f"Circular dependency detected, {len(remaining)} packages affected")
            # Try to break circular dependency by adding packages with fewest remaining deps
            remaining_list = list(remaining)
            remaining_list.sort(key=lambda p: len(graph.get(p, set()) & remaining))
            levels.append(remaining_list)
            break

        levels.append(level)

        # Mark these packages as built
        for pkg in level:
            remaining.remove(pkg)
            built.add(pkg)

    return levels


def can_build_package(pkg_name: str, config: BuildConfig) -> bool:
    """Check if package can be built (supports CMake or ament_python).

    Args:
        pkg_name: Name of the package
        config: Build configuration

    Returns:
        True if package has CMakeLists.txt (CMake) or is ament_python, False otherwise
    """
    global _cmake_package_cache

    if pkg_name in _cmake_package_cache:
        return _cmake_package_cache[pkg_name]

    # Use package name map to find the package path
    pkg_path = get_package_path(pkg_name, config)
    if not pkg_path:
        # Can't find package, assume buildable
        _cmake_package_cache[pkg_name] = True
        return True

    # Check for CMake package
    if (pkg_path / "CMakeLists.txt").exists():
        _cmake_package_cache[pkg_name] = True
        return True

    # Check for ament_python in package.xml
    pkg_xml_file = pkg_path / "package.xml"
    if pkg_xml_file.exists():
        try:
            tree = ET.parse(pkg_xml_file)
            build_type = tree.find(".//export/build_type")
            if build_type is not None and build_type.text == "ament_python":
                _cmake_package_cache[pkg_name] = True
                return True
        except Exception:
            pass

    _cmake_package_cache[pkg_name] = False
    return False


def build_single_package(pkg_name: str, config: BuildConfig, lib_paths: Optional[List[str]] = None) -> PackageBuildResult:
    """Build a single Debian package.

    Args:
        pkg_name: Name of the package to build
        config: Build configuration
        lib_paths: Optional list of additional library search paths for dpkg-shlibdeps

    Returns:
        PackageBuildResult with build outcome
    """
    pkg_work_dir = config.get_pkg_work_dir(pkg_name)
    debian_dir = pkg_work_dir / "debian"

    out_file = pkg_work_dir / "build.out"
    err_file = pkg_work_dir / "build.err"

    # Check if this is a buildable package type (CMake or ament_python)
    if not can_build_package(pkg_name, config):
        logger.debug(f"  Skipping {pkg_name}: unsupported build type")
        with open(config.skipped_pkgs_file, 'a') as f:
            f.write(f"{pkg_name}\n")
        return PackageBuildResult(pkg_name, False, error="Unsupported build type (not CMake or ament_python)")

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
    # Use package name map to handle cases where directory name differs from package name
    pkg_src_dir = get_package_path(pkg_name, config)

    if pkg_src_dir:
        logger.debug(f"  Copying source from {pkg_src_dir} to {pkg_work_dir}")

        # Copy all source files except build artifacts
        rsync_cmd = [
            "rsync", "-av",
            "--exclude=build", "--exclude=install", "--exclude=log",
            "--exclude=debian",  # Don't overwrite bloom-generated debian directory
            f"{pkg_src_dir}/", str(pkg_work_dir) + "/"
        ]
        result = subprocess.run(rsync_cmd, check=True, capture_output=True, text=True)
        logger.debug(f"  rsync output: {result.stdout[:500]}")
        if result.stderr:
            logger.debug(f"  rsync warnings: {result.stderr[:500]}")
    else:
        logger.warning(f"  Could not find package.xml for {pkg_name}")

    # Package-specific fixes for debian/rules
    rules_file = debian_dir / "rules"
    if rules_file.exists():
        rules_content = rules_file.read_text()
        patched = False

        # eagleye_rt: CMake tries to install from a "log" directory that doesn't exist
        # Fix: Create empty log directory before dh_auto_install
        if pkg_name == "eagleye_rt":
            # Patch override_dh_auto_install to create log directory
            old_pattern = 'dh_auto_install\n'
            new_pattern = 'mkdir -p log && dh_auto_install\n'
            if old_pattern in rules_content and 'mkdir -p log' not in rules_content:
                rules_content = rules_content.replace(old_pattern, new_pattern)
                patched = True
                logger.debug(f"  Patched {pkg_name} debian/rules to create log directory")

        if patched:
            rules_file.write_text(rules_content)

    # Build library path arguments for dpkg-shlibdeps
    lib_path_args = ""
    # Start with provided lib_paths (if any)
    existing_paths = [p for p in (lib_paths or []) if Path(p).exists()]
    # Always add system library paths for libraries like libglog, libopencv, etc.
    system_lib_paths = [
        "/usr/lib/x86_64-linux-gnu",
        "/usr/lib",
    ]
    for sys_path in system_lib_paths:
        if Path(sys_path).exists() and sys_path not in existing_paths:
            existing_paths.append(sys_path)
    if existing_paths:
        lib_path_args = " ".join(f"-l{p}" for p in existing_paths)

    # Update debhelper compat level to 12 for parallel build support
    # Level 9 is deprecated; level 12+ enables parallel builds by default
    compat_file = debian_dir / "compat"
    if compat_file.exists():
        try:
            current_compat = compat_file.read_text().strip()
            if current_compat.isdigit() and int(current_compat) < 12:
                compat_file.write_text("12\n")
                logger.debug(f"  Updated debian/compat from {current_compat} to 12")
        except Exception as e:
            logger.debug(f"  Could not update debian/compat: {e}")

    # Build the package using fakeroot debian/rules binary
    # Source setup.bash first to set CMAKE_PREFIX_PATH for finding dependencies
    # Pass EXTRA_LIB_PATHS as a Make variable on the command line
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        cd {pkg_work_dir}

        # Source colcon install setup to set CMAKE_PREFIX_PATH and other env vars
        # This allows CMake to find packages built by colcon (e.g., autoware_cmake)
        source {config.colcon_work_dir}/install/setup.bash

        fakeroot debian/rules binary EXTRA_LIB_PATHS='{lib_path_args}'
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

    Since colcon build has already compiled all packages and created all
    required libraries, we can build Debian packages in parallel without
    any dependency ordering. The colcon install/setup.bash provides all
    needed headers and .so files.

    Uses ProcessPoolExecutor for CPU-intensive package building.

    Returns:
        Dictionary mapping package names to build results
    """
    global _debian_build_log

    # Set up dedicated log file for Debian package building
    _debian_build_log = config.log_dir / "debian_packages.log"
    with open(_debian_build_log, 'w') as f:
        f.write("=== Debian Package Building Log ===\n\n")

    # Clear caches at start of build
    global _dependency_cache, _cmake_package_cache
    _dependency_cache = {}
    _cmake_package_cache = {}

    # Note: Stale debian directories are now cleaned in debian.py (Phase 7)
    # before metadata generation, so they get regenerated with updated templates

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

    log_debian_build(f"Found {len(packages)} packages to build")

    # Use 1/4 of CPU cores for package building to prevent resource exhaustion
    max_workers = max(1, os.cpu_count() // 4)

    add_subtask(8, f"Building {len(packages)} packages (using {max_workers} workers)...")
    log_debian_build(f"Using {max_workers} parallel workers")

    results = {}
    succeeded = 0
    failed = 0
    skipped = 0

    # Initialize library search paths with system libraries and ROS libraries
    # Since colcon already built everything, all .so files are in the install directory
    lib_paths = [
        "/usr/lib",
        "/usr/lib/x86_64-linux-gnu",
        "/opt/ros/humble/lib",
        str(config.colcon_work_dir / "install" / "lib"),
    ]

    # Add all package install lib paths from colcon build
    for pkg in packages:
        pkg_install_lib = config.colcon_work_dir / "install" / pkg / "lib"
        if pkg_install_lib.exists():
            lib_paths.append(str(pkg_install_lib))

    # Get display instance for progress bar
    display = get_display()
    progress_task = None
    if display:
        progress_task = display.create_progress_task(
            "Building Debian packages",
            total=len(packages)
        )

    # Build all packages in parallel - no dependency ordering needed
    # since colcon build already created all required libraries
    log_debian_build("Starting parallel Debian package builds...")

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(build_single_package, pkg, config, lib_paths): pkg
            for pkg in packages
        }

        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                build_result = future.result()
                results[pkg_name] = build_result

                if build_result.success:
                    if build_result.deb_file and config.skipped_pkgs_file.exists():
                        with open(config.skipped_pkgs_file, 'r') as f:
                            if pkg_name in f.read():
                                skipped += 1
                                log_debian_build(f"Skipped (already built): {pkg_name}")
                            else:
                                succeeded += 1
                                log_debian_build(f"Built successfully: {pkg_name}")
                    else:
                        succeeded += 1
                        log_debian_build(f"Built successfully: {pkg_name}")
                else:
                    failed += 1
                    log_debian_build(f"Failed: {pkg_name} - {build_result.error[:200] if build_result.error else 'Unknown error'}", "error")

            except Exception as e:
                log_debian_build(f"Exception building {pkg_name}: {e}", "error")
                results[pkg_name] = PackageBuildResult(pkg_name, False, error=str(e))
                failed += 1

            # Update progress
            if display and progress_task:
                display.update_progress(progress_task, advance=1)

    log_debian_build(f"\n=== Build Summary ===")
    log_debian_build(f"Total: {len(packages)}, Built: {succeeded}, Skipped: {skipped}, Failed: {failed}")

    if failed > 0:
        add_subtask(8, f"⚠ {failed} packages failed (see logs)")

    add_subtask(8, f"✓ Build complete: {succeeded} built, {skipped} skipped, {failed} failed")
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
                    error_preview = result.error[:200].replace('\n', ' ')
                    print(f"      Error: {error_preview}")
                    if len(result.error) > 200:
                        print(f"      (Full error in build/{pkg_name}/build.err)")
        print("")

    print(f"  Debian packages: {config.release_dir}")
    print(f"  Build logs: {config.log_dir}")
    print("=" * 60)
    print("")
