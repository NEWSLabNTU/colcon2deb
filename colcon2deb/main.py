"""Main orchestrator for building Debian packages."""

import argparse
import subprocess
import sys
import os
from pathlib import Path

from . import __version__
from .config import BuildConfig
from .utils import logger, print_phase, set_display, start_phase, complete_phase, Colors
from .display import BuildDisplay
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

    parser.add_argument(
        "--plain-output",
        action="store_true",
        help="Use plain text output (disable rich formatting)",
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show verbose output (debug messages and command details)",
    )

    args = parser.parse_args()

    # Set logging level based on verbose flag
    if args.verbose:
        import logging
        logger.setLevel(logging.DEBUG)
        # Also set root logger to DEBUG to catch all module loggers
        logging.getLogger().setLevel(logging.DEBUG)

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
        # Determine if we should use rich display
        use_rich = not args.plain_output and not os.getenv("CI") == "true"

        # Create and configure display
        with BuildDisplay(total_phases=8, enable_live=use_rich, verbose=args.verbose) as display:
            set_display(display)

            # Print configuration
            display.log(f"{Colors.BOLD}{Colors.CYAN}Build Configuration{Colors.RESET}", level="info")
            display.log(f"  ROS Distribution: {Colors.GREEN}{config.ros_distro}{Colors.RESET}")
            display.log(f"  Install Prefix: {Colors.GREEN}{config.ros_install_prefix}{Colors.RESET}")
            display.log(f"  Workspace: {config.workspace_dir}")
            display.log(f"  Output: {config.output_dir}")

            # Execute build phases
            start_phase(1, "Preparing working directories")
            prepare.setup_directories(config)
            complete_phase(1)

            start_phase(2, "Copying source files")
            prepare.copy_workspace(config)
            complete_phase(2)

            start_phase(3, "Installing dependencies")
            dependencies.install_dependencies(config)
            complete_phase(3)

            start_phase(4, "Compiling packages")
            compiler.build_workspace(config)
            compiler.source_install_setup(config)
            complete_phase(4)

            start_phase(5, "Generating rosdep list")
            debian.create_rosdep_list(config)
            complete_phase(5)

            start_phase(6, "Creating package list")
            packages = debian.get_package_list(config)
            logger.info(f"  Found {len(packages)} packages")
            complete_phase(6)

            start_phase(7, "Generating Debian metadata")
            debian.generate_debian_metadata(config)
            complete_phase(7)

            start_phase(8, "Building Debian packages")
            results = packager.build_packages_parallel(config)
            complete_phase(8)

            packager.print_build_summary(config, results)

            # Check if any packages failed
            failed_count = sum(1 for r in results.values() if not r.success)
            if failed_count > 0:
                logger.warning(f"{failed_count} packages failed to build")
                sys.exit(1)

            logger.info(f"{Colors.BOLD}{Colors.GREEN}Build completed successfully!{Colors.RESET}")
            sys.exit(0)

    except KeyboardInterrupt:
        logger.error("\nBuild interrupted by user")
        sys.exit(130)

    except subprocess.CalledProcessError as e:
        # Clean error message for command failures without full traceback
        logger.error(f"Build failed: Command failed with exit code {e.returncode}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Build failed: {e}")
        # Only show traceback in verbose mode
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
