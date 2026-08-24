# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Practices

### Temporary Files
- **ALWAYS** create temporary files in `$project/tmp/` directory using Write/Edit tools
- **NEVER** use `/tmp/` or other system temporary directories for project-related temp files
- This keeps temporary files organized and makes cleanup easier

### Phase Documents
- Work phases are tracked in `docs/phases/<number>-<name>.md`
- Each document has work items and acceptance criteria with checkboxes
- Update checkboxes as items are completed and tests pass

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
# Run unit tests (fast, no Docker)
just test

# Run all tests including integration (requires Docker)
just test-all

# Run integration tests only (requires Docker)
just test-integ

# Run lint, format, and type checks
just check

# Run all CI checks (lint + unit tests)
just ci

# Auto-fix lint issues
just check-fix

# Format code
just format
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
   - Phase 1: Prepare working directories
   - Phase 2: Copy source files
   - Phase 3: Install rosdep dependencies
   - Phase 4: Build workspace with colcon
   - Phase 5: Generate rosdep list
   - Phase 6: Create package list
   - Phase 7: Generate debian directories (using rosdeb_bloom)
   - Phase 8: Build .deb packages

**Pipelined execution** (default, `build.pipeline: true`): phase 4 runs in a
background thread while phases 5-7 proceed — they only need sources and the
base ROS environment. Phase 8 gates each package's .deb build on colcon
having finished that package (tracked via colcon's per-run
`log/build_<ts>/events.log`; see `helper/colcon_events.py`), because the dh
build recompiles the package from source and only needs its *dependencies*
present in the workspace install tree. Wall-clock is roughly
`max(colcon, packaging)` instead of their sum. `build.pipeline: false`
restores strict phase ordering.

### Key Design Principles
- **No ROS on host** - Only Python and Docker required
- **Sources copied before build** - The workspace is rsync'd into the output directory; builds never modify the original sources
- **Isolated builds** - Everything runs in Docker containers
- **Custom install prefix** - Configurable via `install_prefix` in config.yaml
- **Package suffix support** - Append version suffix to package names via `package_suffix`
- **Parallel builds** - Configurable via `parallel_jobs` in config.yaml (affects colcon build, debian generation, and package building)
- **Vendored rosdeb-bloom** - Modified bloom with install prefix and ament_python fixes
- **Prerequisite checks** - docker binary + daemon verified before any work; the container verifies the image provides python3/colcon/rosdep/cmake and fails with a hint otherwise
- **Writable install prefix in-container** - entry.sh hands the (ephemeral) install prefix to the build user, tolerating packages that write into it at build time
- **Pipelined build** - phase 4 (colcon) runs concurrently with phases 5-7; phase 8 gates per package on colcon's events.log (`build.pipeline: false` restores serial order)
- **Remote Dockerfile caching** - cached by URL; `--refresh-dockerfile` re-downloads

## Configuration File Format

The schema is validated by `colcon2deb/config.py`. Required keys: `version: 1`,
`docker.image` XOR `docker.dockerfile`, `output.directory`, `packages.directory`.
The workspace is NOT a config key — it comes from the `--workspace` CLI flag.
Relative paths are resolved against the config file's directory.

```yaml
version: 1                       # required, must be 1

docker:                          # required: exactly one of image / dockerfile
  image: my-builder:latest       # prebuilt image, OR:
  # dockerfile: ./Dockerfile     # local path or https:// URL
  # image_name: my-builder      # name for the built image (default: colcon2deb_builder)
  # build_context: .            # docker build context (default: Dockerfile's dir)
  # platform: linux/arm64       # for cross-builds

output:
  directory: ./build             # required: build output directory

packages:
  directory: ./debian-overrides  # required: per-package debian/ overrides
                                 # (must exist; empty dir is fine)

build:                           # optional
  ros_distro: humble             # default: humble
  install_prefix: /opt/ros/humble  # default: /opt/ros/{ros_distro}; must be absolute
  package_suffix: "1.0.0"        # optional: ros-humble-pkg-1.0.0 naming
  parallel_jobs: 4               # default: 0 = auto-detect from CPU count
  use_nvidia_runtime: false      # default: false
  skip_tests: false              # true = -DBUILD_TESTING=OFF + DEB_BUILD_OPTIONS=nocheck
  pipeline: true                 # default: true — run colcon build and
                                 # packaging concurrently; false = serial phases
```

Unknown keys are warned about and ignored.

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
│   │   ├── docker_build.log # Host-side Docker image build log
│   │   ├── phases/          # One log per build phase (predictable names, no dates)
│   │   │   ├── phase1_prepare.log
│   │   │   ├── phase2_copy_src.log
│   │   │   ├── phase3_install_deps.log
│   │   │   ├── phase4_build_src.log
│   │   │   ├── phase5_create_rosdep_list.log
│   │   │   ├── phase6_create_package_list.log
│   │   │   ├── phase7_generate_debian_dir.log
│   │   │   └── phase8_build_deb.log
│   │   ├── packages/        # Per-package logs
│   │   │   └── <package>/
│   │   │       ├── generate_debian.log
│   │   │       └── build_deb.log
│   │   ├── reports/         # Build reports
│   │   │   ├── summary.txt
│   │   │   ├── packages.txt
│   │   │   ├── successful.txt
│   │   │   ├── failed.txt
│   │   │   └── skipped.txt
│   │   └── scripts/         # Generated scripts for reference
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
- `phases/phase{1-8}_*.log` - One log per build phase (predictable names)
- `packages/<pkg>/generate_debian.log` - Per-package debian generation log
- `packages/<pkg>/build_deb.log` - Per-package deb build log
- `reports/summary.txt` - Build summary with statistics
- `reports/{successful,failed,skipped}.txt` - Package status lists
- `docker_build.log` - Docker image build log (host-side)

### Common Issues

1. **Stale caches from colcon2deb ≤ 0.4.1**: Older versions cached bloom output
   into the user's `debian-overrides/` directory, where it shadowed regeneration.
   Current versions never write there — `packages.directory` is user input only.
   If upgrading from an old version, delete any auto-generated cache once:
   ```bash
   rm -rf debian-overrides/*/debian/   # only if you did not write them yourself
   ```

2. **rosdeb_bloom import errors**: Ensure rosdeb-bloom is properly pip-installed in container

3. **Permission errors**: Helper scripts need read permission (`chmod a+r`)

## Related Projects

- [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo) - Autoware build configurations and APT repository builder
