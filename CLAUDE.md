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

**Vendored rosdeb-bloom**: The project includes a vendored `rosdeb-bloom` package (fork of ros/bloom) in `colcon2deb/rosdeb-bloom/`. This provides custom install prefix support and ament_python environment hooks. It is pip-installed in the container at runtime.

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
    - `entry.sh` - Container entry point (pip installs rosdeb-bloom)
    - `main.py` - Build orchestrator inside container
    - `generate_debian_dir.py` - Debian metadata generation
    - `build_deb.py` - Package building
  - `rosdeb-bloom/` - Vendored Debian generator (pip-installed in container)
    - `rosdeb_bloom/` - Modified bloom package with install prefix support
    - `rosdeb_bloom/generators/debian/templates/` - Debian packaging templates
- `templates/` - Templates for package generation
- `tests/` - Test suite
- `examples/` - Example configurations
  - `simple-example/` - Simple test workspace
  - `custom-example/` - Demonstrates package suffix and custom install prefix
- `.github/workflows/` - CI/CD workflows
- `pyproject.toml` - Python project configuration (uv compatible)
- `justfile` - Build automation

### Build Process Flow
1. Host `colcon2deb/main.py` reads config and launches Docker container
2. Container mounts workspace, output dir, helper scripts, and rosdeb-bloom
3. Container entry point (`helper/entry.sh`) pip installs rosdeb-bloom and sets up user
4. `helper/main.py` orchestrates the build phases:
   - Phase 1-3: Setup directories, copy sources
   - Phase 4: Build workspace with colcon
   - Phase 5: Install rosdep dependencies
   - Phase 6: Generate rosdep mappings
   - Phase 7: Generate debian directories (using rosdeb_bloom)
   - Phase 8: Build .deb packages

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Read-only source mounts** - Prevents accidental modifications
- **Isolated builds** - Everything runs in Docker containers
- **Custom install prefix** - Configurable via `install_prefix` in config.yaml
- **Package suffix support** - Append version suffix to package names via `package_suffix`
- **Vendored rosdeb-bloom** - Modified bloom with install prefix and ament_python fixes

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

# With package suffix (e.g., ros-humble-pkg-1.0.0 instead of ros-humble-pkg)
workspace_dir: ./source
output_dir: ./build
image: my-builder:latest
install_prefix: /opt/myproject
package_suffix: "1.0.0"
```

## Important Files

- `colcon2deb/main.py` - Host-side CLI and Docker orchestration
- `colcon2deb/helper/main.py` - Container-side build orchestrator
- `colcon2deb/helper/generate_debian_dir.py` - Uses rosdeb_bloom to generate debian/
- `colcon2deb/rosdeb-bloom/rosdeb_bloom/api.py` - Library API for debian generation
- `colcon2deb/rosdeb-bloom/rosdeb_bloom/generators/debian/templates/` - Package templates
- `pyproject.toml` - Project configuration, dependencies, ruff config
- `justfile` - Build automation (build, install, test, bump-version)

## Vendored rosdeb-bloom

The `colcon2deb/rosdeb-bloom/` directory contains an embedded modified bloom package:

- Renamed from `bloom` to `rosdeb_bloom` to avoid import conflicts
- Added `--install-prefix` support for custom installation paths
- Fixed `ament_python/rules.em` template to generate environment hooks:
  - Creates `hook/pythonpath.{sh,dsv}` for PYTHONPATH setup
  - Creates `hook/ament_prefix_path.{sh,dsv}` for AMENT_PREFIX_PATH
  - Creates `local_setup.{bash,sh,zsh}` for environment sourcing

The package is pip-installed from `/rosdeb-bloom` inside the container by `entry.sh`.

## Output Directory Structure

The build output directory (`build/` by default) contains:

```
build/
├── debs/                    # Output .deb files
├── logs/                    # Build logs
│   ├── <timestamp>/         # Timestamped log directory
│   │   ├── logs/            # Phase log files
│   │   │   ├── docker_build.log
│   │   │   ├── phase3_apt_update.log
│   │   │   ├── phase3_apt_install.log
│   │   │   ├── phase4_colcon_build.log
│   │   │   └── phase5_rosdep.log
│   │   ├── reports/         # Status and summary files
│   │   │   ├── summary.txt
│   │   │   ├── packages.txt
│   │   │   ├── successful.txt
│   │   │   ├── failed.txt
│   │   │   └── skipped.txt
│   │   └── scripts/         # Generated scripts
│   │       └── install_deps.sh
│   └── latest -> <timestamp>
├── packaging/               # Debian packaging work directory
│   └── <package>/
│       └── debian/          # Generated debian files
└── workspace/               # Colcon workspace copy
    ├── build/               # Colcon build artifacts
    ├── install/             # Colcon install artifacts
    ├── log/                 # Colcon logs
    └── src/                 # Source files
```

## Debugging

### Build Logs
Logs are organized in `build/logs/latest/`:
- `logs/` - Phase execution logs (docker_build, apt, colcon, rosdep)
- `reports/summary.txt` - Build summary with statistics
- `reports/{successful,failed,skipped}.txt` - Package status lists
- `packaging/<package>/gen_deb.{out,err}` - Debian generation logs
- `packaging/<package>/build.{out,err}` - Package build logs

### Common Issues

1. **debian-overrides caching**: If changing `install_prefix`, delete cached debian directories:
   ```bash
   rm -rf debian-overrides/*/debian/
   ```

2. **rosdeb_bloom import errors**: Ensure rosdeb-bloom is properly pip-installed in container

3. **Permission errors**: Helper scripts need read permission (`chmod a+r`)

## Related Projects

- [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo) - Autoware build configurations and APT repository builder
