"""Generate Debian packaging metadata using bloom."""

import os
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import List, Tuple

from .config import BuildConfig
from .utils import logger, print_phase, run_command, ensure_dir, add_subtask, get_display


def clean_stale_debian_dirs(config: BuildConfig) -> int:
    """Remove debian directories that don't have EXTRA_LIB_PATHS support.

    This forces regeneration with the updated template.
    Must be called BEFORE generate_debian_metadata() to ensure fresh generation.

    Args:
        config: Build configuration

    Returns:
        Number of directories cleaned
    """
    build_dir = config.pkg_build_dir
    if not build_dir.exists():
        return 0

    cleaned = 0
    for pkg_dir in build_dir.iterdir():
        if not pkg_dir.is_dir():
            continue

        debian_dir = pkg_dir / "debian"
        rules_file = debian_dir / "rules"

        if rules_file.exists():
            try:
                content = rules_file.read_text()
                if "EXTRA_LIB_PATHS" not in content:
                    logger.debug(f"Removing stale debian dir for {pkg_dir.name}")
                    shutil.rmtree(debian_dir)
                    cleaned += 1
            except Exception as e:
                logger.warning(f"Error checking {rules_file}: {e}")

    if cleaned > 0:
        logger.info(f"Cleaned {cleaned} stale debian directories")
    return cleaned


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


def validate_rosdep_coverage(config: BuildConfig):
    """Validate that all packages are in rosdep.yaml.

    Generates a report of packages missing from rosdep mappings.
    """
    rosdep_file = config.output_dir / "rosdep.yaml"

    if not rosdep_file.exists():
        logger.warning("Cannot validate rosdep coverage - rosdep.yaml not found")
        return

    # Load rosdep mappings
    import yaml
    with open(rosdep_file) as f:
        mappings = yaml.safe_load(f) or {}

    # Get list of packages from workspace
    packages = get_package_list(config)
    package_names = [pkg[0] for pkg in packages]

    # Check coverage
    missing = []
    for pkg_name in package_names:
        if pkg_name not in mappings:
            missing.append(pkg_name)

    # Generate report
    report_file = config.log_dir / "rosdep_coverage_report.txt"
    with open(report_file, 'w') as f:
        f.write(f"Rosdep Coverage Report\n")
        f.write(f"{'='*80}\n\n")
        f.write(f"Total packages: {len(package_names)}\n")
        f.write(f"In rosdep.yaml: {len(package_names) - len(missing)}\n")
        f.write(f"Missing: {len(missing)}\n\n")

        if missing:
            f.write(f"Missing packages:\n")
            for pkg in sorted(missing):
                f.write(f"  - {pkg}\n")

    logger.debug(f"Rosdep coverage: {len(package_names) - len(missing)}/{len(package_names)}")
    logger.debug(f"Coverage report: {report_file}")

    if missing:
        logger.warning(f"{len(missing)} packages not in rosdep.yaml - may cause bloom failures")


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

            # Patch debian/rules to add EXTRA_LIB_PATHS if missing
            rules_file = dst_debian_dir / "rules"
            if rules_file.exists():
                content = rules_file.read_text()
                if "EXTRA_LIB_PATHS" not in content and "dh_shlibdeps" in content:
                    # Add $(EXTRA_LIB_PATHS) to dh_shlibdeps line
                    patched = content.replace(
                        "dh_shlibdeps -l",
                        "dh_shlibdeps $(EXTRA_LIB_PATHS) -l"
                    )
                    if patched != content:
                        rules_file.write_text(patched)
                        logger.debug(f"  Patched {pkg_name}/debian/rules with EXTRA_LIB_PATHS")

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
    rosdep_yaml = config.output_dir / "rosdep.yaml"
    rosdep_sources_list = "/etc/ros/rosdep/sources.list.d/50-autoware-local.list"

    cmd = [
        "bash",
        "-c",
        f"""
        set -eo pipefail
        cd {pkg_config_dir}

        # Export environment variables for rosdep
        export InstallationPrefix="{config.ros_install_prefix}"
        export ROS_DISTRO="{config.ros_distro}"

        # Ensure rosdep knows about our custom mappings
        # Method 1: Point to our custom rosdep.yaml via sources list
        if [ -f "{rosdep_sources_list}" ]; then
            echo "Using custom rosdep sources: {rosdep_sources_list}"
        fi

        # Method 2: Also set ROSDEP_SOURCE_PATH (bloom may use this)
        export ROSDEP_SOURCE_PATH="{rosdep_yaml}"

        # Run rosdep update to ensure cache is fresh
        rosdep update --rosdistro {config.ros_distro} 2>&1 | head -5 || true

        # Debug: Check if autoware_universe_utils can be resolved
        echo "Checking rosdep resolution for autoware_universe_utils..."
        rosdep resolve autoware_universe_utils --rosdistro {config.ros_distro} 2>&1 | head -3 || echo "Resolution test: autoware_universe_utils not resolvable"

        # Run bloom-generate with full rosdep context
        bloom-generate rosdebian --ros-distro {config.ros_distro} {pkg_path}

        # Apply custom rules.em template if exists
        custom_rules="/opt/colcon2deb_pkg/colcon2deb/templates/debian/rules.em"
        echo "Checking for custom template at: $custom_rules"
        if [ -f "$custom_rules" ]; then
            echo "✓ Custom template found! Applying..."
            cp "$custom_rules" debian/rules.em
            bloom-generate rosdebian --process-template-files --ros-distro {config.ros_distro} {pkg_path}
            echo "✓ Custom template applied"
        else
            echo "✗ Custom template NOT found at $custom_rules"
        fi

        # Copy to work directory
        rsync -av --delete debian/ {dst_debian_dir}/
        """
    ]

    try:
        with open(out_file, 'w') as out, open(err_file, 'w') as err:
            subprocess.run(cmd, stdout=out, stderr=err, check=True)

        # Patch debian/rules to add EXTRA_LIB_PATHS if missing (backup for template issues)
        rules_file = dst_debian_dir / "rules"
        if rules_file.exists():
            content = rules_file.read_text()
            if "EXTRA_LIB_PATHS" not in content and "dh_shlibdeps" in content:
                patched = content.replace(
                    "dh_shlibdeps -l",
                    "dh_shlibdeps $(EXTRA_LIB_PATHS) -l"
                )
                if patched != content:
                    rules_file.write_text(patched)
                    logger.debug(f"  Patched {pkg_name}/debian/rules with EXTRA_LIB_PATHS")

        return (pkg_name, True, "")
    except subprocess.CalledProcessError as e:
        # Read FULL error output from both stdout and stderr
        error_stdout = out_file.read_text() if out_file.exists() else ""
        error_stderr = err_file.read_text() if err_file.exists() else ""

        # Combine both for complete context
        error_msg = f"Exit code: {e.returncode}\n"
        if error_stdout:
            error_msg += f"\n--- STDOUT ---\n{error_stdout}\n"
        if error_stderr:
            error_msg += f"\n--- STDERR ---\n{error_stderr}\n"

        return (pkg_name, False, error_msg)


def configure_custom_rosdep(config: BuildConfig):
    """Configure rosdep to use custom rosdep.yaml for internal packages.

    This allows bloom-generate to resolve dependencies on packages being built
    in the same workspace.
    """
    rosdep_yaml = config.output_dir / "rosdep.yaml"

    if not rosdep_yaml.exists():
        logger.warning(f"Custom rosdep.yaml not found at {rosdep_yaml}")
        return False

    # Create custom rosdep source file
    source_file = "/etc/ros/rosdep/sources.list.d/50-autoware-local.list"

    # Use file:// URL for local rosdep.yaml
    rosdep_url = f"yaml file://{rosdep_yaml.absolute()}"

    logger.debug(f"Creating rosdep source file: {source_file}")
    logger.debug(f"  URL: {rosdep_url}")

    try:
        # Create the sources list file using sudo (system directory)
        cmd = [
            "bash", "-c",
            f"sudo mkdir -p /etc/ros/rosdep/sources.list.d && "
            f"echo '# Custom rosdep mappings for local packages' | sudo tee {source_file} > /dev/null && "
            f"echo '{rosdep_url}' | sudo tee -a {source_file} > /dev/null"
        ]
        run_command(cmd, capture_output=True)

        # Run rosdep update to load the custom mappings
        logger.debug("Running rosdep update to load custom mappings...")
        cmd = ["rosdep", "update"]
        result = run_command(cmd, capture_output=True)

        logger.debug("Rosdep configured with custom package mappings")
        return True

    except Exception as e:
        logger.error(f"Failed to configure custom rosdep: {e}")
        return False


def verify_rosdep_configuration(config: BuildConfig) -> bool:
    """Verify that rosdep can resolve internal package dependencies.

    Returns:
        True if rosdep is properly configured, False otherwise
    """
    rosdep_yaml = config.output_dir / "rosdep.yaml"

    if not rosdep_yaml.exists():
        logger.error(f"Rosdep file not found: {rosdep_yaml}")
        return False

    # Test resolution of known internal packages
    test_packages = [
        'autoware_universe_utils',
        'autoware_utils',
        'autoware_common_msgs'
    ]

    logger.debug("Testing rosdep resolution for internal packages...")

    for pkg in test_packages:
        cmd = ["rosdep", "resolve", pkg, "--rosdistro", config.ros_distro]
        result = run_command(cmd, capture_output=True, check=False)

        if result.returncode != 0:
            logger.warning(f"  Cannot resolve '{pkg}': {result.stderr[:100]}")
            return False
        else:
            logger.debug(f"  ✓ Can resolve '{pkg}'")

    logger.debug("Rosdep configuration verified successfully")
    return True


def generate_debian_metadata(config: BuildConfig):
    """Generate Debian metadata for all packages in parallel."""
    # Clean stale debian directories BEFORE generating new metadata
    # This ensures old debian/rules without EXTRA_LIB_PATHS are removed
    add_subtask(7, "Checking for stale debian directories...")
    cleaned = clean_stale_debian_dirs(config)
    if cleaned > 0:
        add_subtask(7, f"Cleaned {cleaned} stale debian directories")

    packages = get_package_list(config)

    # Configure rosdep with custom package mappings
    add_subtask(7, "Configuring rosdep with custom package mappings...")
    rosdep_ok = configure_custom_rosdep(config)

    if not rosdep_ok:
        logger.error("Failed to configure rosdep")
        add_subtask(7, "⚠ Rosdep configuration failed - bloom may not resolve dependencies")
    else:
        # Verify rosdep can resolve internal packages
        if not verify_rosdep_configuration(config):
            logger.warning("Rosdep verification failed - internal packages cannot be resolved")
            add_subtask(7, "⚠ Rosdep verification failed - check rosdep.yaml and sources.list")
        else:
            add_subtask(7, "✓ Rosdep configured and verified")

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
                    logger.error(f"  ✗ Failed: {result_pkg_name}: {error_msg}")
            except Exception as e:
                failed_packages.append((pkg_name, str(e)))
                logger.error(f"  ✗ Exception for {pkg_name}: {e}")

            # Update progress
            if display and progress_task:
                display.update_progress(progress_task, advance=1)

    if failed_packages:
        add_subtask(7, f"⚠ {len(failed_packages)} packages failed (see logs)")

        # Write full errors to dedicated log file
        error_log = config.log_dir / "debian_gen_errors.log"
        with open(error_log, 'w') as f:
            f.write(f"Debian Metadata Generation Errors\n")
            f.write(f"{'='*80}\n")
            f.write(f"Total failed: {len(failed_packages)}\n")
            f.write(f"{'='*80}\n\n")

            for pkg_name, error in failed_packages:
                f.write(f"\n{'='*80}\n")
                f.write(f"Package: {pkg_name}\n")
                f.write(f"{'-'*80}\n")
                f.write(f"Error:\n{error}\n")

        logger.info(f"Full error details written to {error_log}")

        # Show first 3 in console
        for pkg_name, error in failed_packages[:3]:
            logger.warning(f"    - {pkg_name}: {error}")

    add_subtask(7, f"✓ Generated metadata for {succeeded}/{len(packages)} packages")
