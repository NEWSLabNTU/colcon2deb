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

**Vendored bloom**: The project includes a vendored `bloom_gen` package (fork of ros/bloom) in `colcon2deb/bloom/` as a git submodule. This provides custom install prefix support and ament_python environment hooks.

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

### Testing and Linting
```bash
# Run tests
uv run pytest tests/ -v

# Run with coverage
uv run pytest tests/ --cov

# Lint check
uv run ruff check colcon2deb/

# Format check
uv run ruff format --check colcon2deb/

# Auto-fix lint issues
uv run ruff check --fix colcon2deb/
uv run ruff format colcon2deb/
```

### Version Management
```bash
# Bump version (updates pyproject.toml)
just bump-version 0.3.0
```

## CI/CD

### GitHub Workflows

**CI** (`.github/workflows/ci.yml`) - Triggered on push/PR to main:
- `lint`: Runs `ruff check` and `ruff format --check`
- `build`: Builds wheel, verifies installation
- `test`: Runs pytest on Python 3.10, 3.11, 3.12

**Release** (`.github/workflows/release.yml`) - Triggered on `v*` tags:
- `build`: Builds wheel and sdist
- `pypi-publish`: Publishes to PyPI via trusted publishing (OIDC)
- `github-release`: Creates GitHub release with artifacts

### Creating a Release
```bash
# Bump version
just bump-version X.Y.Z

# Commit and tag
git add pyproject.toml
git commit -m "Bump version to X.Y.Z"
git push origin main
git tag -a vX.Y.Z -m "Release vX.Y.Z"
git push origin vX.Y.Z
```

## Architecture

### Directory Structure
- `colcon2deb/` - Main Python package
  - `main.py` - Host-side CLI and Docker orchestration
  - `helper/` - Scripts that run inside Docker container
    - `entry.sh` - Container entry point
    - `main.py` - Build orchestrator inside container
    - `generate_debian_dir.py` - Debian metadata generation
    - `build_deb.py` - Package building
  - `bloom/` - Vendored bloom_gen submodule (git submodule)
    - `bloom_gen/` - Modified bloom package with install prefix support
    - `bloom_gen/generators/debian/templates/` - Debian packaging templates
- `templates/` - Templates for package generation
- `tests/` - Test suite
- `examples/` - Example configurations
  - `simple-example/` - Simple test workspace
- `.github/workflows/` - CI/CD workflows
- `pyproject.toml` - Python project configuration (uv compatible)
- `justfile` - Build automation

### Build Process Flow
1. Host `colcon2deb/main.py` reads config and launches Docker container
2. Container mounts workspace, output dir, helper scripts, and bloom_gen
3. Container entry point (`helper/entry.sh`) sets up user and environment
4. `helper/main.py` orchestrates the build phases:
   - Phase 1-3: Setup directories, copy sources
   - Phase 4: Build workspace with colcon
   - Phase 5: Install rosdep dependencies
   - Phase 6: Generate rosdep mappings
   - Phase 7: Generate debian directories (using bloom_gen)
   - Phase 8: Build .deb packages

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Read-only source mounts** - Prevents accidental modifications
- **Isolated builds** - Everything runs in Docker containers
- **Custom install prefix** - Configurable via `install_prefix` in config.yaml
- **Vendored bloom** - Modified bloom_gen with install prefix and ament_python fixes

## Configuration File Format

```yaml
# Basic configuration with Dockerfile
workspace_dir: ./source
output_dir: ./build
dockerfile: Dockerfile

# With remote Dockerfile
workspace_dir: ./source
output_dir: ./build
dockerfile: https://example.com/Dockerfile

# With prebuilt image
workspace_dir: ./source
output_dir: ./build
image: my-builder:latest

# With custom install prefix (default: /opt/ros/{ros_distro})
workspace_dir: ./source
output_dir: ./build
image: my-builder:latest
install_prefix: /opt/autoware/custom
```

## Important Files

- `colcon2deb/main.py` - Host-side CLI and Docker orchestration
- `colcon2deb/helper/main.py` - Container-side build orchestrator
- `colcon2deb/helper/generate_debian_dir.py` - Uses bloom_gen to generate debian/
- `colcon2deb/bloom/bloom_gen/api.py` - Library API for debian generation
- `colcon2deb/bloom/bloom_gen/generators/debian/templates/` - Package templates
- `pyproject.toml` - Project configuration, dependencies, ruff config
- `justfile` - Build automation (build, install, test, bump-version)

## Vendored bloom_gen

The `colcon2deb/bloom/` directory is a git submodule containing a modified bloom package:

- Renamed from `bloom` to `bloom_gen` to avoid import conflicts
- Added `--install-prefix` support for custom installation paths
- Fixed `ament_python/rules.em` template to generate environment hooks:
  - Creates `hook/pythonpath.{sh,dsv}` for PYTHONPATH setup
  - Creates `hook/ament_prefix_path.{sh,dsv}` for AMENT_PREFIX_PATH
  - Creates `local_setup.{bash,sh,zsh}` for environment sourcing

### Updating bloom submodule
```bash
cd colcon2deb/bloom
git pull origin colcon2deb
cd ../..
git add colcon2deb/bloom
git commit -m "Update bloom submodule"
```

## Debugging

### Build Logs
Logs are in `build/log/<timestamp>/`:
- `summary.txt` - Build summary
- `<package>/gen_deb.{out,err}` - Debian generation logs
- `<package>/build.{out,err}` - Package build logs

### Common Issues

1. **debian-overrides caching**: If changing `install_prefix`, delete cached debian directories:
   ```bash
   rm -rf debian-overrides/*/debian/
   ```

2. **bloom_gen import errors**: Ensure PYTHONPATH includes `/bloom` in container

3. **Permission errors**: Helper scripts need read permission (`chmod a+r`)

## Related Projects

- [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo) - Autoware build configurations and APT repository builder
- [bloom](https://github.com/jerry73204/bloom) - Forked bloom with colcon2deb branch
