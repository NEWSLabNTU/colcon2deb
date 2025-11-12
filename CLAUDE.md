# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Practices

### Temporary Files
- **ALWAYS** create temporary files in `$project/tmp/` directory using Write/Edit tools
- **NEVER** use `/tmp/` or other system temporary directories for project-related temp files
- This keeps temporary files organized and makes cleanup easier

## Repository Overview

This project builds Debian packages for Autoware (autonomous driving platform) in isolated Docker containers. It implements a **two-program architecture**:

1. **Host-side CLI** (`colcon2deb.py`) - Python orchestrator that launches Docker containers
2. **Container-side Python package** (`autoware_debian_packager`) - Python modules that run inside containers

The output is Debian packages in a specified output directory that can be installed or assembled into an APT repository.

**Package Management**: The project uses uv for Python dependency management and package building.

**Architecture**: All build logic is implemented in Python modules (no bash scripts). The `autoware_debian_packager` package is mounted into Docker containers and installed on-the-fly.

## Key Commands

### Package Installation and Development
```bash
# Install uv (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Set up development environment
uv sync

# Build the project wheel
uv build --wheel

# Build Debian package
make deb

# Install the Debian package
sudo dpkg -i dist/colcon2deb_0.2.0-1_all.deb
```

### Build Debian Packages for Autoware
```bash
# Prepare Autoware workspace first (see examples/*/README.md)
cd examples/autoware-2025.02-amd64
git clone --branch 2025.02-ws https://github.com/NEWSLabNTU/autoware.git source
cd source && vcs import src < autoware.repos

# Run the build script
cd ..
./build.sh

# Or use colcon2deb directly with a prepared workspace
colcon2deb --workspace ~/repos/autoware/0.45.1-ws/ --config examples/autoware-0.45.1-amd64/config.yaml
```

### Docker Image Management
```bash
# Build Docker images
docker build -t colcon2deb:0.45.1-amd64 docker/0.45.1/amd64/
docker build -t colcon2deb:2025.02-amd64 docker/2025.02/amd64/

# For ARM64
docker build -t colcon2deb:0.45.1-arm64 docker/0.45.1/arm64/

# For Jetpack
docker build -t colcon2deb:0.45.1-jetpack docker/0.45.1/jetpack-6.0/
```

### Testing
```bash
# Run tests with uv
uv run pytest tests/
uv run pytest tests/ --cov

# Specific test files
uv run pytest tests/unit/test_colcon2deb.py
```

## Architecture

### Directory Structure
- `colcon2deb.py` - Main entry point CLI (host-side)
- `src/autoware_debian_packager/` - Container-side Python package
  - `__init__.py` - Package initialization
  - `config.py` - BuildConfig dataclass
  - `utils.py` - Logging and command utilities
  - `prepare.py` - Directory setup and workspace copying
  - `dependencies.py` - Dependency installation
  - `compiler.py` - Workspace compilation
  - `debian.py` - Debian metadata generation
  - `packager.py` - Parallel package building
  - `main.py` - Main orchestrator
- `config/` - Pre-configured Debian templates for problematic packages (shlibs.local files)
- `docker/` - Dockerfiles for different platforms
  - `0.45.1/` - For Autoware 0.45.1
  - `2025.02/` - For Autoware 2025.02
- `examples/` - Example configuration files with build scripts
  - `autoware-0.45.1-amd64/` - Example for Autoware 0.45.1
    - `build.sh` - Builds Debian packages (requires prepared source)
    - `config.yaml` - Configuration file
    - `debian-overrides/` - Package-specific overrides
    - `source/` - User prepares Autoware workspace here (git submodule or clone)
    - `README.md` - Setup and usage instructions
  - `autoware-2025.02-amd64/` - Example for Autoware 2025.02
    - Same structure as 0.45.1 example
- `makedeb/` - Debian package build configuration
  - `PKGBUILD` - Package build script for makedeb
  - `colcon2deb` - Wrapper script for /usr/bin
- `tests/` - Test suite
- `doc/` - Documentation
  - `roadmap.md` - Development roadmap
  - `script-execution-model.md` - How scripts execute
  - `troubleshooting-guide.md` - Common issues and solutions
  - `parallel-optimization.md` - Performance tuning
- `pyproject.toml` - Python project configuration (uv compatible)
- `Makefile` - Build automation (wheel, deb, clean)

### Build Process Flow
1. `colcon2deb.py` reads config and launches Docker container
2. Container mounts the `autoware_debian_packager` source and installs it with pip3
3. Container creates ubuntu user for specified uid/gid
4. `autoware_debian_packager.main` orchestrates the build:
   - `prepare.setup_directories()` - Initialize working directories
   - `prepare.copy_workspace()` - Copy Autoware source
   - `dependencies.install_dependencies()` - Install ROS dependencies
   - `compiler.build_workspace()` - Compile with colcon
   - `debian.create_rosdep_list()` - Generate custom rosdep mappings
   - `debian.get_package_list()` - List all packages
   - `debian.generate_debian_metadata()` - Create Debian metadata with bloom (parallel)
   - `packager.build_packages_parallel()` - Build .deb packages (parallel)

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Read-only source mounts** - Prevents accidental modifications
- **Isolated builds** - Everything runs in Docker containers
- **Clean interfaces** - Communication via volumes, Python dataclasses
- **Type safety** - BuildConfig dataclass with type hints
- **Parallel execution** - ThreadPoolExecutor (I/O) and ProcessPoolExecutor (CPU)

## Testing

**IMPORTANT: This project uses uv for dependency management.**

```bash
# Run all tests
uv run pytest tests/

# Run with coverage
uv run pytest tests/ --cov

# Run specific test file
uv run pytest tests/unit/test_colcon2deb.py

# Run tests in verbose mode
uv run pytest tests/ -v
```

## Important Files

- `colcon2deb.py` - Host-side CLI entry point
- `src/autoware_debian_packager/main.py` - Container-side build orchestrator
- `src/autoware_debian_packager/config.py` - BuildConfig dataclass
- `pyproject.toml` - Project configuration and dependencies
- `justfile` - Task automation (targets: build, test, install-dev, clean)
- `MIGRATION.md` - Complete migration guide from bash to Python
- `config/*/debian/shlibs.local` - Package-specific dependency mappings
- `docker/*/Dockerfile` - Platform-specific container definitions
- `makedeb/PKGBUILD` - Debian package build configuration
- `examples/*/build.sh` - Example build scripts (require prepared Autoware workspace)
- `examples/*/README.md` - Instructions for preparing Autoware workspace

## Common Development Tasks

### Preparing Autoware Workspace for Examples
Each example requires a prepared Autoware workspace in the `source/` directory:

```bash
cd examples/autoware-2025.02-amd64

# Clone Autoware repository
git clone --branch 2025.02-ws https://github.com/NEWSLabNTU/autoware.git source

# Import all package dependencies
cd source
vcs import src < autoware.repos
cd ..

# Now run the build
./build.sh
```

See `examples/*/README.md` for detailed instructions.

### Building and Installing colcon2deb
```bash
# Build wheel package
make wheel

# Build Debian package
make deb

# Install the package
sudo dpkg -i dist/colcon2deb_0.1.0-1_all.deb
```

### Adding New Package Configuration
1. Create directory in `config/<package_name>/`
2. Add `debian/control`, `debian/rules`, `debian/changelog`
3. Test with individual package build

### Creating New Example Configuration
1. Create directory in `examples/<name>/`
2. Add `config.yaml` with Docker and build settings
3. Create `build.sh` script to clone source and run colcon2deb
4. Add `debian-overrides/` for package-specific overrides

### Debugging Build Issues
1. Run colcon2deb with verbose output
2. Check logs in the build output directory
3. Run individual helper scripts manually in container

### Modifying Build Process
- Host-side changes: Edit `colcon2deb.py`
- Container-side changes: Edit scripts in `helper/`
- Docker images: Edit Dockerfiles in `docker/`
- Package configuration: Edit `pyproject.toml` and `makedeb/PKGBUILD`
- Always maintain separation between host and container responsibilities

## Known Issues and Recent Fixes

### Rosdep Installation (Fixed)
- **Issue**: rosdep install was not running because Autoware depends on `pacmod3_msgs` which has no rosdep entry, causing rosdep to fail with exit code 1
- **Fix**: Modified `src/autoware_debian_packager/dependencies.py:11-65` to:
  1. First detect unresolvable packages by parsing rosdep error messages
  2. Extract package names that cannot be resolved (e.g., `pacmod3_msgs`)
  3. Re-run rosdep with `--skip-keys` for those unresolvable packages
  4. This allows rosdep to successfully generate install commands for resolvable dependencies
- **Location**: `/home/aeon/repos/colcon2deb/src/autoware_debian_packager/dependencies.py`

### Autoware 2025.02 Build Requirements
- **Docker Image**: Uses `ghcr.io/autowarefoundation/autoware-base:cuda-latest` which includes CUDA 12.3, cuDNN 8.9.5, TensorRT 8.6.1
- **System Dependencies Required**:
  - Build tools: `parallel`, `fakeroot`, `debhelper`, `dh-python`, `rsync`
  - System libraries: `libboost-date-time-dev`, `libpcl-dev`, `nlohmann-json3-dev`, `libgeographic-dev`, `libpugixml-dev`, `libxrandr-dev`
  - ROS packages (already in base image): `ros-humble-geographic-msgs`, `ros-humble-cudnn-cmake-module`, `ros-humble-tensorrt-cmake-module`, `ros-humble-udp-msgs`, `ros-humble-can-msgs`
- **Known Build Issues**:
  - `embree_vendor`: Requires `libxrandr-dev` for GLFW
  - `tamagawa_imu_driver`: Has include path issues with can_msgs headers (may require source patching)
  - `pointcloud_to_laserscan`, `ros2_socketcan`: Additional investigation needed

### Dependency Installation
- PCL (Point Cloud Library) must be installed in Docker images
- ROS setup.bash has issues with `set -u` (unbound variables)
- Dependencies are installed by generating and executing a script

### Performance
- Use unique semaphore IDs to avoid blocking between build stages
- I/O operations use fewer parallel jobs than CPU operations
- Package builds limited to 1/4 of cores to prevent resource exhaustion
- Large example directories excluded from wheel build to prevent hanging

### Docker Images
- Must include ROS 2 APT repository setup
- Must run rosdep init and update
- Must install libpcl-dev, libopencv-dev, python3-xmlschema
- Autoware setup script must be run with appropriate flags

### uv and Virtual Environment
- apt_pkg module not available via pip (system package only)
- Use `make setup-apt` to link system apt packages into venv
- makedeb requires apt_pkg for dependency checking
- PKGBUILD uses `arch=('all')` for architecture-independent package
- uv is significantly faster than pip and rye for dependency resolution

## Configuration File Format

```yaml
# Basic configuration
workspace_dir: /path/to/autoware/ws
output_dir: /path/to/output
dockerfile: docker/0.45.1/amd64/Dockerfile

# Or with remote Dockerfile
workspace_dir: /path/to/autoware/ws
output_dir: /path/to/output
dockerfile: https://example.com/Dockerfile

# Or with prebuilt image
workspace_dir: /path/to/autoware/ws
output_dir: /path/to/output
image: colcon2deb:0.45.1-amd64
```