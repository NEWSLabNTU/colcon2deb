"""Generate Debian packaging metadata using bloom."""

import os
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import List, Tuple

from .config import BuildConfig
from .utils import logger, print_phase, run_command, ensure_dir, add_subtask, get_display


def create_rosdep_list(config: BuildConfig):
    """Create custom rosdep list for package dependencies.

    This generates a rosdep.yaml file mapping package names to
    their install locations.
    """
    if config.skip_gen_rosdep_list:
        add_subtask(5, "Skipped (--skip-gen-rosdep-list)")
        return

    rosdep_file = config.output_dir / "rosdep.yaml"

    # Get list of packages from install space
    install_dir = config.colcon_work_dir / "install"

    if not install_dir.exists():
        add_subtask(5, f"Install directory not found: {install_dir}")
        rosdep_file.touch()
        return

    add_subtask(5, "Scanning installed packages...")

    # Generate rosdep mappings
    # Format: package_name: {ubuntu: [ros-{distro}-package-name]}
    mappings = {}

    # List packages in install space
    for pkg_dir in install_dir.iterdir():
        if pkg_dir.is_dir() and pkg_dir.name not in ["_local_setup_util_sh.py", "_local_setup_util_ps1.py"]:
            pkg_name = pkg_dir.name
            ros_pkg_name = f"ros-{config.ros_distro}-{pkg_name.replace('_', '-')}"
            mappings[pkg_name] = {"ubuntu": [ros_pkg_name]}

    # Write rosdep.yaml
    import yaml
    with open(rosdep_file, 'w') as f:
        yaml.dump(mappings, f, default_flow_style=False)

    add_subtask(5, f"✓ Created rosdep list with {len(mappings)} packages")


def get_package_list(config: BuildConfig) -> List[Tuple[str, Path]]:
    """Get list of packages to build.

    Returns list of (package_name, package_path) tuples.
    """
    add_subtask(6, "Listing packages in workspace...")

    # Use colcon list to get packages
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        source {config.colcon_work_dir}/install/setup.bash
        cd {config.colcon_work_dir}
        colcon list --base-paths src
        """
    ]

    result = run_command(cmd, capture_output=True)

    packages = []
    for line in result.stdout.strip().split('\n'):
        if line:
            parts = line.split('\t')
            if len(parts) >= 2:
                pkg_name = parts[0]
                # Convert relative path to absolute path from workspace root
                pkg_path = config.colcon_work_dir / parts[1]
                packages.append((pkg_name, pkg_path))

    add_subtask(6, f"✓ Found {len(packages)} packages")
    return packages


def generate_debian_metadata_for_package(
    pkg_name: str,
    pkg_path: Path,
    config: BuildConfig
) -> Tuple[str, bool, str]:
    """Generate Debian metadata for a single package.

    Returns: (pkg_name, success, error_message)
    """
    pkg_work_dir = config.get_pkg_work_dir(pkg_name)
    pkg_config_dir = config.get_pkg_config_dir(pkg_name)

    ensure_dir(pkg_work_dir)
    ensure_dir(pkg_config_dir)

    out_file = pkg_work_dir / "gen_deb.out"
    err_file = pkg_work_dir / "gen_deb.err"
    src_debian_dir = pkg_config_dir / "debian"
    dst_debian_dir = pkg_work_dir / "debian"

    # Check if custom debian directory exists
    if src_debian_dir.exists():
        logger.debug(f"Using provided debian directory for {pkg_name}")
        cmd = ["rsync", "-av", "--delete", f"{src_debian_dir}/", str(dst_debian_dir)]

        try:
            with open(out_file, 'w') as out, open(err_file, 'w') as err:
                subprocess.run(cmd, stdout=out, stderr=err, check=True)
            return (pkg_name, True, "")
        except subprocess.CalledProcessError as e:
            return (pkg_name, False, f"Failed to copy debian directory: {e}")

    # Generate with bloom
    logger.debug(f"Running bloom-generate for {pkg_name}")

    # Fix bloom .config and .bloom_logs directory issues
    os.makedirs(os.path.expanduser("~/.config"), exist_ok=True)
    os.makedirs(os.path.expanduser("~/.bloom_logs"), exist_ok=True)

    # Create bloom.conf with InstallationPrefix
    bloom_conf = pkg_config_dir / "bloom.conf"
    with open(bloom_conf, 'w') as f:
        f.write("# bloom.conf - Configuration for bloom-generate\n")
        f.write("# This file sets template variables for EmPy processing\n\n")
        f.write("[variables]\n")
        f.write(f"InstallationPrefix={config.ros_install_prefix}\n")

    # Run bloom-generate
    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        cd {pkg_config_dir}
        export InstallationPrefix="{config.ros_install_prefix}"
        bloom-generate rosdebian --ros-distro {config.ros_distro} {pkg_path}

        # Apply custom rules.em template if exists
        custom_rules="/helper/templates/debian/rules.em"
        if [ -f "$custom_rules" ]; then
            cp "$custom_rules" debian/rules.em
            bloom-generate rosdebian --process-template-files --ros-distro {config.ros_distro} {pkg_path}
        fi

        # Copy to work directory
        rsync -av --delete debian/ {dst_debian_dir}/
        """
    ]

    try:
        with open(out_file, 'w') as out, open(err_file, 'w') as err:
            subprocess.run(cmd, stdout=out, stderr=err, check=True)
        return (pkg_name, True, "")
    except subprocess.CalledProcessError as e:
        error_msg = err_file.read_text() if err_file.exists() else str(e)
        return (pkg_name, False, error_msg)


def generate_debian_metadata(config: BuildConfig):
    """Generate Debian metadata for all packages in parallel."""
    packages = get_package_list(config)

    # Use thread pool for I/O-heavy bloom operations
    # Use half the CPU cores to avoid file system contention
    max_workers = max(1, os.cpu_count() // 2)

    add_subtask(7, f"Generating metadata for {len(packages)} packages (using {max_workers} workers)...")

    failed_packages = []
    succeeded = 0

    # Get display instance for progress bar
    display = get_display()
    progress_task = None
    if display:
        progress_task = display.create_progress_task(
            "Generating Debian metadata",
            total=len(packages)
        )

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(generate_debian_metadata_for_package, pkg_name, pkg_path, config): pkg_name
            for pkg_name, pkg_path in packages
        }

        for future in as_completed(futures):
            pkg_name = futures[future]
            try:
                result_pkg_name, success, error_msg = future.result()
                if success:
                    succeeded += 1
                    logger.debug(f"  ✓ Generated metadata for {result_pkg_name}")
                else:
                    failed_packages.append((result_pkg_name, error_msg))
                    logger.error(f"  ✗ Failed: {result_pkg_name}: {error_msg[:100]}")
            except Exception as e:
                failed_packages.append((pkg_name, str(e)))
                logger.error(f"  ✗ Exception for {pkg_name}: {e}")

            # Update progress
            if display and progress_task:
                display.update_progress(progress_task, advance=1)

    if failed_packages:
        add_subtask(7, f"⚠ {len(failed_packages)} packages failed (see logs)")
        for pkg_name, error in failed_packages[:3]:  # Show first 3
            logger.warning(f"    - {pkg_name}: {error[:60]}")

    add_subtask(7, f"✓ Generated metadata for {succeeded}/{len(packages)} packages")
