"""Main orchestrator for building Debian packages."""

import argparse
import sys
from pathlib import Path

from . import __version__
from .config import BuildConfig
from .utils import logger, print_phase
from . import prepare, dependencies, compiler, debian, packager


def main():
    """Main entry point for the builder."""
    parser = argparse.ArgumentParser(
        description="Build Debian packages from ROS colcon workspace",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--workspace",
        type=Path,
        required=True,
        help="Path to colcon workspace directory",
    )

    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output directory for build artifacts and .deb files",
    )

    parser.add_argument(
        "--ros-distro",
        default="humble",
        help="ROS distribution (default: humble)",
    )

    parser.add_argument(
        "--install-prefix",
        help="Installation prefix for packages (default: /opt/ros/{distro})",
    )

    # Skip flags
    parser.add_argument(
        "--skip-rosdep-install",
        action="store_true",
        help="Skip rosdep dependency installation",
    )

    parser.add_argument(
        "--skip-copy-src",
        action="store_true",
        help="Skip copying source files",
    )

    parser.add_argument(
        "--skip-gen-rosdep-list",
        action="store_true",
        help="Skip generating rosdep list",
    )

    parser.add_argument(
        "--skip-colcon-build",
        action="store_true",
        help="Skip colcon build (use existing install)",
    )

    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )

    args = parser.parse_args()

    # Create build configuration
    config = BuildConfig(
        workspace_dir=args.workspace,
        output_dir=args.output,
        ros_distro=args.ros_distro,
        ros_install_prefix=args.install_prefix,
        skip_rosdep_install=args.skip_rosdep_install,
        skip_copy_src=args.skip_copy_src,
        skip_gen_rosdep_list=args.skip_gen_rosdep_list,
        skip_colcon_build=args.skip_colcon_build,
    )

    try:
        # Print configuration
        print_phase("Build Configuration")
        print(f"  ROS Distribution: {config.ros_distro}")
        print(f"  Install Prefix: {config.ros_install_prefix}")
        print(f"  Workspace: {config.workspace_dir}")
        print(f"  Output: {config.output_dir}")

        # Execute build phases
        prepare.setup_directories(config)
        prepare.copy_workspace(config)

        dependencies.install_dependencies(config)

        compiler.build_workspace(config)
        compiler.source_install_setup(config)

        debian.create_rosdep_list(config)

        print_phase("Phase 6: Creating package list")
        packages = debian.get_package_list(config)
        logger.info(f"  Found {len(packages)} packages")

        debian.generate_debian_metadata(config)

        results = packager.build_packages_parallel(config)

        packager.print_build_summary(config, results)

        # Check if any packages failed
        failed_count = sum(1 for r in results.values() if not r.success)
        if failed_count > 0:
            logger.warning(f"{failed_count} packages failed to build")
            sys.exit(1)

        logger.info("Build completed successfully!")
        sys.exit(0)

    except KeyboardInterrupt:
        logger.error("\nBuild interrupted by user")
        sys.exit(130)

    except Exception as e:
        logger.error(f"Build failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
