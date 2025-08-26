# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This project builds Debian packages for Autoware (autonomous driving platform) in isolated Docker containers. It implements a **two-program architecture**:

1. **Host-side CLI** (`colcon2deb.py`) - Python orchestrator that launches Docker containers
2. **Container-side scripts** (`helper/`) - Bash build scripts that run inside containers

The output is Debian packages in a specified output directory that can be installed or assembled into an APT repository.

**Package Management**: The project uses uv for Python dependency management and package building.

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
# Using installed colcon2deb command
colcon2deb --workspace ~/repos/autoware/0.45.1-ws/ --config examples/config.yaml

# Or using the example build scripts
cd examples/autoware-0.45.1-amd64
./build.sh  # This clones Autoware and runs colcon2deb

# With remote Dockerfile URL
colcon2deb --workspace ~/repos/autoware/0.45.1-ws/ --config examples/config-with-dockerfile.yaml
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
- `colcon2deb.py` - Main entry point CLI
- `helper/` - Container-side build scripts
  - `entry.sh` - Docker container entry point
  - `main.sh` - Build orchestrator
  - `prepare.sh` - Initialize directories
  - `copy-src.sh` - Copy source to build area
  - `install-deps.sh` - Install dependencies via rosdep
  - `build-src.sh` - Compile with colcon
  - `create-rosdep-list.sh` - Generate custom rosdep mappings
  - `create-package-list.sh` - List packages to build
  - `generate-debian-dir.sh` - Create Debian metadata with bloom
  - `build-deb.sh` - Build .deb packages
- `config/` - Pre-configured Debian templates for problematic packages (shlibs.local files)
- `docker/` - Dockerfiles for different platforms
  - `0.45.1/` - For Autoware 0.45.1
  - `2025.02/` - For Autoware 2025.02
- `examples/` - Example configuration files with build scripts
  - `autoware-0.45.1-amd64/` - Example for Autoware 0.45.1
    - `build.sh` - Clones Autoware and builds packages
    - `config.yaml` - Configuration file
    - `debian-overrides/` - Package-specific overrides
  - `autoware-2025.02-amd64/` - Example for Autoware 2025.02
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
2. `helper/entry.sh` creates ubuntu user and starts build
3. `helper/main.sh` orchestrates the build:
   - `prepare.sh` - Initialize working directories
   - `copy-src.sh` - Copy Autoware source
   - `install-deps.sh` - Install ROS dependencies
   - `build-src.sh` - Compile with colcon
   - `create-rosdep-list.sh` - Generate custom rosdep mappings
   - `create-package-list.sh` - List all packages
   - `generate-debian-dir.sh` - Create Debian metadata with bloom
   - `build-deb.sh` - Build .deb packages in parallel

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Read-only source mounts** - Prevents accidental modifications
- **Isolated builds** - Everything runs in Docker containers
- **Clean interfaces** - Communication via volumes, env vars, exit codes

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

- `colcon2deb.py` - Main CLI entry point
- `pyproject.toml` - Project configuration and dependencies
- `Makefile` - Build automation (targets: wheel, deb, clean, setup-apt)
- `helper/main.sh` - Container-side build orchestrator
- `helper/entry.sh` - Docker container entry point
- `helper/generate-rosdep-commands.sh` - Dependency resolution
- `helper/build-deb.sh` - Parallel package building
- `config/*/debian/shlibs.local` - Package-specific dependency mappings
- `docker/*/Dockerfile` - Platform-specific container definitions
- `makedeb/PKGBUILD` - Debian package build configuration
- `examples/*/build.sh` - Example build scripts that clone Autoware

## Common Development Tasks

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