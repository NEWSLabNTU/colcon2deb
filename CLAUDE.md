# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Practices

### Temporary Files
- **ALWAYS** create temporary files in `$project/tmp/` directory using Write/Edit tools
- **NEVER** use `/tmp/` or other system temporary directories for project-related temp files
- This keeps temporary files organized and makes cleanup easier

## Repository Overview

This project builds Debian packages from ROS 2 colcon workspaces in isolated Docker containers.

**Package Management**: The project uses uv for Python dependency management and package building.

**Architecture**: All build logic is implemented in Python modules. The `colcon2deb` package is mounted into Docker containers and installed on-the-fly.

**Autoware Examples**: For Autoware-specific build configurations and examples, see [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo).

## Key Commands

### Package Installation and Development
```bash
# Build the wheel package
just build

# Install the wheel
just install

# Or use uv for development
uv sync
```

### Running colcon2deb
```bash
# Basic usage
colcon2deb --workspace /path/to/ros_ws --config config.yaml

# See help
colcon2deb --help
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
- `colcon2deb/` - Main Python package
  - `cli.py` - CLI entry point
  - `main.py` - Build orchestrator
  - `config.py` - BuildConfig dataclass
  - `utils.py` - Logging and command utilities
  - `prepare.py` - Directory setup and workspace copying
  - `dependencies.py` - Dependency installation (rosdep integration)
  - `compiler.py` - Workspace compilation
  - `debian.py` - Debian metadata generation
  - `packager.py` - Parallel package building
- `templates/` - Templates for package generation
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

### Build Process Flow
1. `colcon2deb.cli` reads config and launches Docker container
2. Container mounts the `colcon2deb` source and installs it
3. Container creates ubuntu user for specified uid/gid
4. Container runs `sudo rosdep init` then `rosdep update` as ubuntu user
5. `colcon2deb.main` orchestrates the build:
   - `prepare.setup_directories()` - Initialize working directories
   - `prepare.copy_workspace()` - Copy source files
   - `dependencies.install_dependencies()` - Install dependencies via rosdep
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

- `colcon2deb/cli.py` - CLI entry point
- `colcon2deb/main.py` - Build orchestrator
- `colcon2deb/dependencies.py` - Rosdep integration for dependency management
- `colcon2deb/config.py` - BuildConfig dataclass
- `pyproject.toml` - Project configuration and dependencies
- `justfile` - Main build automation (build, install, clean)

## Common Development Tasks

### Building and Installing colcon2deb
```bash
# Build wheel package
just build

# Install the wheel
just install
```

### Debugging Build Issues
1. Check logs in `build/log/` directory:
   - `container.log` - Full container output
   - `rosdep_detection.log` - Rosdep key detection
   - `rosdep_full_output.log` - Full rosdep output
   - `colcon_build.log` - Colcon build output
2. Run with verbose output
3. Test rosdep commands manually in the container

### Modifying Build Process
- CLI changes: Edit `colcon2deb/cli.py`
- Build orchestration: Edit `colcon2deb/main.py`
- Dependency management: Edit `colcon2deb/dependencies.py`
- Package configuration: Edit `pyproject.toml`

## Known Issues and Fixes

### Rosdep Integration (Fixed)
- **Issue**: rosdep wasn't detecting dependencies correctly
- **Fix**: Modified `colcon2deb/dependencies.py` to:
  1. Run `sudo rosdep init` before `rosdep update`
  2. Add `--rosdistro` flag explicitly to all rosdep commands
  3. Detect unresolvable packages and skip them with `--skip-keys`
  4. Fix grep pattern to match `sudo -H apt-get install -y` format
  5. Save full rosdep output to `rosdep_full_output.log` for debugging
- **Result**: Successfully detects 130+ packages for installation

### Package Structure
- Package renamed from `autoware_debian_packager` to `colcon2deb`
- CLI entry point: `colcon2deb.cli:main`
- Development mode: Uses editable install (`pip3 install -e .`)
- Wheel mode: Uses PYTHONPATH to access mounted package
- Mount path: `/opt/colcon2deb_pkg/colcon2deb`

## Configuration File Format

```yaml
# Basic configuration
workspace_dir: /path/to/ros_ws
output_dir: /path/to/output
dockerfile: Dockerfile

# Or with remote Dockerfile
workspace_dir: /path/to/ros_ws
output_dir: /path/to/output
dockerfile: https://example.com/Dockerfile

# Or with prebuilt image
workspace_dir: /path/to/ros_ws
output_dir: /path/to/output
image: my-builder:latest
```

## Related Projects

- [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo) - Autoware build configurations and APT repository builder
