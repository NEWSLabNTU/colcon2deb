# Development Setup

## Prerequisites

Before starting development on colcon2deb, ensure you have:

- Python >= 3.10
- Docker or Docker CE
- Git
- uv (for Python package management)
- makedeb (for building Debian packages)

## Setting Up Your Environment

### 1. Install uv

uv is a fast Python package manager used for development:

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Add to PATH (add to ~/.bashrc for persistence)
export PATH="$HOME/.cargo/bin:$PATH"
```

### 2. Clone the Repository

```bash
git clone https://github.com/autowarefoundation/colcon2deb.git
cd colcon2deb
```

### 3. Initialize Development Environment

```bash
# Create virtual environment and install dependencies
uv sync

# Link system apt_pkg module
make setup-apt
```

### 4. Install makedeb

For building Debian packages:

```bash
# On Ubuntu/Debian
wget -qO - 'https://proget.makedeb.org/debian-feeds/makedeb.pub' | \
    gpg --dearmor | \
    sudo tee /usr/share/keyrings/makedeb-archive-keyring.gpg

echo 'deb [signed-by=/usr/share/keyrings/makedeb-archive-keyring.gpg arch=all] https://proget.makedeb.org/ makedeb main' | \
    sudo tee /etc/apt/sources.list.d/makedeb.list

sudo apt update
sudo apt install makedeb
```

## Project Structure

Understanding the codebase organization:

```
colcon2deb/
├── colcon2deb.py           # Main CLI entry point
├── helper/                 # Container-side build scripts
│   ├── entry.sh           # Docker entry point
│   ├── main.sh           # Build orchestrator
│   └── ...               # Individual build scripts
├── config/                # Package-specific overrides
├── docker/               # Dockerfiles for platforms
├── examples/             # Example configurations
├── makedeb/              # Debian package build files
├── tests/                # Test suite
├── book/                 # Documentation (mdBook)
├── pyproject.toml        # Python project config
├── Makefile              # Build automation
└── CLAUDE.md             # AI assistant guidelines
```

## Development Workflow

### Running from Source

Test changes without installation:

```bash
# Activate virtual environment
source .venv/bin/activate

# Run directly
python colcon2deb.py --help

# Or using uv
uv run colcon2deb --help
```

### Testing Changes

Run the test suite:

```bash
# Run all tests
uv run pytest tests/

# Run specific test
uv run pytest tests/test_colcon2deb.py

# Run with coverage
uv run pytest tests/ --cov

# Run with verbose output
uv run pytest tests/ -v
```

### Building Packages

Build wheel and Debian packages:

```bash
# Build Python wheel
make wheel

# Build Debian package
make deb

# Clean build artifacts
make clean
```

## Making Changes

### Code Style Guidelines

Follow these conventions:

**Python Code**:
- Follow PEP 8
- Use type hints
- Add docstrings
- Keep functions focused

**Shell Scripts**:
- Use `set -e` for error handling
- Quote variables: `"$var"`
- Use `#!/usr/bin/env bash`
- 2-space indentation

### Adding Features

1. **Plan the feature**:
   - Update roadmap.md if significant
   - Consider architecture impact
   - Design clean interfaces

2. **Implement changes**:
   - Write tests first (TDD)
   - Keep commits focused
   - Update documentation

3. **Test thoroughly**:
   - Run unit tests
   - Test with real workspaces
   - Verify Docker compatibility

### Modifying Build Scripts

When changing helper scripts:

1. **Understand the execution model**:
   - Scripts are sourced by main.sh
   - Use `exit 1` for errors
   - Share environment variables

2. **Test in container**:
   ```bash
   # Enter container
   docker run -it --rm \
       -v $(pwd):/workspace:ro \
       -v /tmp/output:/output \
       --entrypoint /bin/bash \
       colcon2deb:0.45.1-amd64
   
   # Test your script
   /helper/your-script.sh
   ```

3. **Verify error handling**:
   - Test failure cases
   - Check exit codes propagate
   - Ensure cleanup happens

### Adding Package Overrides

For packages that need custom Debian configuration:

1. Create directory structure:
   ```bash
   mkdir -p config/<package_name>/debian
   ```

2. Add required files:
   - `control`: Package metadata
   - `rules`: Build instructions
   - `changelog`: Version history
   - `shlibs.local`: Dependency mappings

3. Test with single package:
   ```bash
   colcon2deb --packages-select <package_name>
   ```

## Debugging

### Enable Verbose Output

Add debug output to scripts:

```bash
# In shell scripts
set -x  # Print commands

# In Python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Interactive Debugging

Debug Python code:

```python
import pdb; pdb.set_trace()
# Or with IPython
import IPython; IPython.embed()
```

Debug shell scripts:

```bash
# Add breakpoint
echo "Debug point reached"
read -p "Press Enter to continue..."

# Or use bash -x
bash -x /helper/script.sh
```

### Container Debugging

Keep container running for inspection:

```bash
# Override entrypoint
docker run -it --rm \
    --entrypoint /bin/bash \
    -v $(pwd):/workspace:ro \
    colcon2deb:0.45.1-amd64

# Manually run build
su - ubuntu -c "/helper/main.sh"
```

### Log Analysis

Check logs systematically:

1. Main process log: `/output/build.log`
2. Stage logs: `/workdir/logs/*.log`
3. Package logs: `/workdir/logs/build/<package>.log`
4. System logs: `journalctl -u docker`

## Testing Strategies

### Unit Tests

Test individual functions:

```python
def test_parse_config():
    config = parse_config("test.yaml")
    assert config['workspace_dir'] == '/test'
```

### Integration Tests

Test complete workflows:

```python
def test_full_build():
    result = run_colcon2deb(test_config)
    assert result.returncode == 0
    assert Path(output_dir / "package.deb").exists()
```

### Docker Tests

Test container behavior:

```bash
# Test script in container
docker run --rm \
    -v $(pwd)/tests:/tests:ro \
    colcon2deb:test \
    /tests/container_test.sh
```

## Performance Profiling

### Python Profiling

```python
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()
# ... code to profile ...
profiler.disable()

stats = pstats.Stats(profiler)
stats.sort_stats('cumulative')
stats.print_stats(20)
```

### Shell Script Timing

```bash
# Time individual commands
time command

# Profile entire script
PS4='+ $(date "+%s.%N") ' bash -x script.sh
```

### Docker Performance

```bash
# Monitor container resources
docker stats <container_id>

# Analyze build cache
docker system df
docker builder du
```