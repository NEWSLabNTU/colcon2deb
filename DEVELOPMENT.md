# Development Guide

This guide covers development setup, testing, building, and contributing to colcon2deb.

## Development Setup

### Prerequisites

- Python >= 3.10
- uv (package management)
- Docker or Docker CE
- makedeb (for building Debian packages)

### Setting Up Development Environment

1. **Install uv** (if not already installed):
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. **Clone the repository**:
```bash
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb
```

3. **Set up the environment**:
```bash
uv sync
```

4. **Link system packages** (for apt_pkg):
```bash
make setup-apt
```

## Project Structure

```
colcon2deb/
├── colcon2deb.py           # Main CLI entry point
├── helper/                 # Container-side build scripts
│   ├── entry.sh            # Docker container entry point
│   ├── main.sh             # Build orchestrator
│   ├── prepare.sh          # Initialize directories
│   ├── copy-src.sh         # Copy source to build area
│   ├── install-deps.sh     # Install dependencies via rosdep
│   ├── build-src.sh        # Compile with colcon
│   ├── create-rosdep-list.sh    # Generate custom rosdep mappings
│   ├── create-package-list.sh   # List packages to build
│   ├── generate-debian-dir.sh   # Create Debian metadata with bloom
│   └── build-deb.sh        # Build .deb packages
├── config/                 # Package-specific Debian templates
├── docker/                 # Dockerfiles for different platforms
├── examples/               # Example configurations with build scripts
├── makedeb/                # Debian package build configuration
│   ├── PKGBUILD            # Package build script
│   └── colcon2deb          # Wrapper script for /usr/bin
├── tests/                  # Test suite
├── doc/                    # Documentation
├── pyproject.toml          # Python project configuration
├── Makefile                # Build automation
└── CLAUDE.md               # AI assistant guidelines
```

## Running Tests

### Basic Testing

```bash
# Run all tests
uv run pytest tests/

# Run with coverage
uv run pytest tests/ --cov

# Run specific test file
uv run pytest tests/unit/test_colcon2deb.py

# Run tests in verbose mode
uv run pytest tests/ -v

# Run tests with output
uv run pytest tests/ -s
```

### Testing During Development

```bash
# Run colcon2deb from development environment
uv run colcon2deb --help

# Test with example configuration
uv run colcon2deb --workspace /path/to/workspace --config examples/autoware-0.45.1-amd64/config.yaml
```

## Building the Project

### Building Python Wheel

```bash
# Build wheel package
make wheel
# or directly with uv
uv build --wheel
```

### Building Debian Package

```bash
# Build Debian package (includes wheel build)
make deb

# The package will be created in dist/
ls -la dist/colcon2deb_0.2.0-1_all.deb
```

### Clean Build Artifacts

```bash
# Clean all build artifacts
make clean
```

## Development Workflow

### Adding New Features

1. Create a feature branch
2. Make changes to the code
3. Add/update tests as needed
4. Run tests to ensure nothing breaks
5. Build and test the package locally
6. Submit a pull request

### Modifying Build Scripts

#### Host-side Changes
- Edit `colcon2deb.py` for main CLI logic
- Update `pyproject.toml` for dependencies
- Modify `Makefile` for build automation

#### Container-side Changes
- Edit scripts in `helper/` directory
- Test changes by running builds with your modifications
- Ensure scripts work in Docker environment

#### Docker Images
- Edit Dockerfiles in `docker/` directory
- Test with different architectures as needed
- Update examples if Docker configuration changes

### Adding Package Overrides

1. Create directory: `config/<package_name>/debian/`
2. Add necessary files:
   - `control` - Package metadata
   - `rules` - Build rules
   - `changelog` - Version history
   - `shlibs.local` - Dependency mappings (if needed)
3. Test the override with a build

## Code Style and Standards

### Python Code
- Follow PEP 8 style guidelines
- Use type hints where appropriate
- Add docstrings to functions and classes
- Keep functions focused and single-purpose

### Shell Scripts
- Use `set -e` for error handling
- Add comments for complex logic
- Use meaningful variable names
- Follow consistent indentation (2 spaces)

### Documentation
- Update README.md for user-facing changes
- Update DEVELOPMENT.md for development changes
- Keep CLAUDE.md current for AI assistance
- Add inline comments for complex code

## Debugging

### Debug Output

```bash
# Enable verbose output in colcon2deb
# (Add debug prints to colcon2deb.py as needed)

# Check Docker container logs
docker logs <container_id>

# Run helper scripts manually in container
docker run -it --rm \
  -v /path/to/workspace:/workspace:ro \
  -v /path/to/output:/output \
  <image_name> \
  /bin/bash
```

### Common Issues

#### apt_pkg Module Not Found
```bash
# Link system package to virtual environment
make setup-apt
```

#### Docker Permission Issues
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in for changes to take effect
```

#### Build Failures
- Check Docker image has all dependencies
- Verify workspace structure is correct
- Review logs in output directory
- Test with simpler packages first

## Testing Package Installation

```bash
# Build the package
make deb

# Install locally
sudo dpkg -i dist/colcon2deb_0.2.0-1_all.deb

# Test the installed command
colcon2deb --help

# Uninstall if needed
sudo apt remove colcon2deb
```

## Contributing

### Before Submitting

1. **Test your changes**:
   - Run the test suite
   - Build the package
   - Test with real workspaces

2. **Update documentation**:
   - Update relevant .md files
   - Add/update code comments
   - Update CHANGELOG if significant

3. **Check code quality**:
   - Ensure consistent style
   - Remove debug prints
   - Check for security issues

### Pull Request Guidelines

- Create focused, single-purpose PRs
- Write clear commit messages
- Include test cases for new features
- Update documentation as needed
- Ensure CI passes (if configured)

## Release Process

1. Update version in:
   - `pyproject.toml`
   - `Makefile`
   - `makedeb/PKGBUILD`

2. Update RELEASE.md with changes

3. Build and test the package

4. Create git tag:
   ```bash
   git tag v0.2.0
   git push origin v0.2.0
   ```

5. Upload package to GitHub releases

## Additional Resources

- [CLAUDE.md](CLAUDE.md) - AI assistant guidelines
- [README.md](README.md) - User documentation
- [Docker Documentation](https://docs.docker.com/)
- [uv Documentation](https://docs.astral.sh/uv/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)

## Support

For development questions and discussions:
- Open an issue on GitHub
- Check existing issues for solutions
- Review pull requests for examples

## License

Apache License 2.0 - See LICENSE file for details.
