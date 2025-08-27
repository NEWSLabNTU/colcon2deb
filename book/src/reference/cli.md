# CLI Reference

## Command Synopsis

```bash
colcon2deb [OPTIONS]
```

## Options

### Required Options

One of these must be provided:

#### `--config FILE`
Path to configuration YAML file containing all build parameters.

#### `--workspace DIR`
Path to the ROS workspace directory containing source code.

### Optional Arguments

#### `--output DIR`
Directory where built packages will be saved.
- Default: `./output`
- Created if it doesn't exist

#### `--dockerfile PATH|URL`
Path or URL to Dockerfile for building the container.
- Can be local file path
- Can be remote URL (https://)
- Alternative to `--image`

#### `--image NAME`
Name of pre-built Docker image to use.
- Example: `colcon2deb:0.45.1-amd64`
- Alternative to `--dockerfile`

#### `--parallel-jobs N`
Number of parallel jobs for building.
- Default: Number of CPU cores
- Affects compilation speed and memory usage

#### `--packages-select PACKAGE [PACKAGE...]`
Build only specified packages.
- Space-separated list
- Useful for testing individual packages

#### `--packages-skip PACKAGE [PACKAGE...]`
Skip specified packages during build.
- Space-separated list
- Useful for excluding problematic packages

#### `--no-cache`
Build Docker image without cache.
- Forces fresh package downloads
- Useful for getting latest updates

#### `--verbose`
Enable verbose output.
- Shows detailed build progress
- Helpful for debugging

#### `--debug`
Enable debug mode.
- Maximum verbosity
- Keeps intermediate files

#### `--help`
Show help message and exit.

#### `--version`
Show version information and exit.

## Configuration File

### Basic Structure

```yaml
# Required fields
workspace_dir: /path/to/workspace
output_dir: /path/to/output

# Docker configuration (one of these required)
image: colcon2deb:0.45.1-amd64
# OR
dockerfile: /path/to/Dockerfile
# OR
dockerfile: https://example.com/Dockerfile

# Optional fields
parallel_jobs: 8
packages_select:
  - package1
  - package2
packages_skip:
  - problematic_package
verbose: true
debug: false
```

### Environment Variables

Configuration can be overridden with environment variables:

```bash
export COLCON2DEB_WORKSPACE=/path/to/workspace
export COLCON2DEB_OUTPUT=/path/to/output
export COLCON2DEB_PARALLEL_JOBS=4
```

## Examples

### Basic Usage

Build all packages from a workspace:

```bash
colcon2deb --workspace ~/ros_ws --config config.yaml
```

### Using Remote Dockerfile

Build with Dockerfile from URL:

```bash
colcon2deb \
    --workspace ~/ros_ws \
    --dockerfile https://raw.githubusercontent.com/autowarefoundation/colcon2deb/main/docker/0.45.1/amd64/Dockerfile \
    --output ./packages
```

### Selective Package Building

Build only specific packages:

```bash
colcon2deb \
    --workspace ~/ros_ws \
    --image colcon2deb:humble \
    --packages-select my_package another_package
```

### Excluding Packages

Skip problematic packages:

```bash
colcon2deb \
    --workspace ~/ros_ws \
    --config config.yaml \
    --packages-skip broken_package
```

### Debug Mode

Run with maximum verbosity:

```bash
colcon2deb \
    --workspace ~/ros_ws \
    --config config.yaml \
    --debug \
    --verbose
```

### Using Configuration File

Create a `config.yaml`:

```yaml
workspace_dir: /home/user/autoware_ws
output_dir: /home/user/packages
image: colcon2deb:0.45.1-amd64
parallel_jobs: 4
packages_skip:
  - test_package
verbose: true
```

Run with config:

```bash
colcon2deb --config config.yaml
```

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success - all packages built |
| 1 | Configuration error |
| 2 | Docker error |
| 3 | Build preparation failed |
| 4 | Dependency installation failed |
| 5 | Compilation failed |
| 6 | Package build failed |
| 127 | Command not found |

## Output Structure

After successful build:

```
output/
├── ros-humble-package1_1.0.0-1_amd64.deb
├── ros-humble-package2_1.0.0-1_amd64.deb
├── build.log                    # Overall build log
├── package-list.txt             # List of built packages
└── logs/                        # Detailed logs (if debug mode)
    ├── build/                   # Colcon build logs
    └── debian/                  # Package build logs
```

## Docker Volume Mounts

The tool creates these volume mounts:

| Container Path | Host Path | Mode | Purpose |
|---------------|-----------|------|---------|
| `/workspace` | `workspace_dir` | Read-only | Source code |
| `/output` | `output_dir` | Read-write | Built packages |
| `/config` | `config/` | Read-only | Package overrides |
| `/helper` | `helper/` | Read-only | Build scripts |

## Performance Tuning

### Memory Limits

Prevent OOM kills:

```bash
# Reduce parallel jobs
colcon2deb --config config.yaml --parallel-jobs 2

# Or in Docker directly
docker run --memory=8g --memory-swap=8g ...
```

### CPU Limits

Control CPU usage:

```bash
# Limit to specific CPUs
docker run --cpuset-cpus="0-3" ...

# Limit CPU shares
docker run --cpu-shares=512 ...
```

### I/O Optimization

Improve disk performance:

```bash
# Use tmpfs for build directory
docker run --mount type=tmpfs,destination=/tmp,size=8g ...

# Use SSD for output
colcon2deb --output /ssd/packages ...
```

## Troubleshooting Commands

### Check Docker

```bash
# Verify Docker is running
docker info

# Check available space
docker system df

# Clean up
docker system prune -a
```

### Validate Configuration

```bash
# Check YAML syntax
python -c "import yaml; yaml.safe_load(open('config.yaml'))"

# Test workspace
ls -la /path/to/workspace
colcon list --base-paths /path/to/workspace
```

### Manual Container Entry

```bash
# Enter container for debugging
docker run -it --rm \
    --entrypoint /bin/bash \
    -v /workspace:/workspace:ro \
    -v /output:/output \
    colcon2deb:0.45.1-amd64
```