# colcon2deb v0.2.0

Major release introducing uv package management, streamlined Debian packaging, and simplified example structure.

## What's New

### Package Management & Build System
- **uv Integration** - Modern Python dependency management with uv (10-100x faster than pip)
- **Automated Debian Packaging** - One-command build with `make deb`
- **Direct Installation** - System-wide `colcon2deb` command after package install
- **Entry Point Support** - Proper Python package with console script entry point

### Simplified Examples
- **No More Submodules** - Removed 27GB of Autoware submodules from examples
- **On-Demand Cloning** - Example build scripts clone Autoware when needed
- **Faster Repository** - Significantly reduced repository size and clone time

### Technical Improvements
- **Remote Dockerfile Support** - Download and build from Dockerfiles hosted on HTTP/HTTPS URLs
- **Simplified Output Structure** - All build artifacts go directly to the configured output directory
- **Intelligent Caching** - Downloaded Dockerfiles are cached to speed up subsequent builds
- **Automatic Cleanup** - Temporary directories are cleaned up automatically
- **Virtual Environment Compatibility** - Proper handling of system packages like apt_pkg

## Breaking Changes from v0.1.0

- **uv Required for Development** - Development now requires uv instead of plain Python
- **Examples Structure** - Autoware source no longer included as submodules
- **Test Commands** - Use `uv run pytest` instead of `python -m pytest`
- **Package Location** - Installed to `/opt/colcon2deb` with wrapper in `/usr/bin`

## Installation

```bash
# Build and install from source
make deb
sudo dpkg -i dist/colcon2deb_0.2.0-1_all.deb
```

## Migration from v0.1.0

1. Install uv: `curl -LsSf https://astral.sh/uv/install.sh | sh`
2. Remove old repository with submodules
3. Clone fresh repository
4. Run `uv sync` for development environment
5. Use `make deb` to create Debian package
6. Use new `build.sh` scripts in example directories

## Changelog

### Added
- uv package management with pyproject.toml
- Makefile with wheel, deb, and setup-apt targets
- makedeb integration with PKGBUILD
- Example build.sh scripts for cloning Autoware
- Entry point configuration for direct command access
- Virtual environment apt_pkg compatibility

### Changed
- Examples now use build scripts instead of submodules
- Package builds to /opt/colcon2deb with system wrapper
- Tests run through uv instead of direct Python
- Single architecture-independent package output

### Removed
- Autoware source submodules from examples
- Direct Python dependency management
- Multiple architecture-specific package outputs

See the [README](README.md) for detailed usage instructions, dependencies, and configuration details.
