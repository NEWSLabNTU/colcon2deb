# Autoware 2025.02 AMD64 Debian Package Build

This example demonstrates how to build Debian packages for Autoware 2025.02 on AMD64 architecture.

## Prerequisites

1. **Install colcon2deb** (from project root):
   ```bash
   cd /path/to/colcon2deb
   uv sync
   ```

2. **Install vcstool**:
   ```bash
   sudo apt install python3-vcstool
   ```

## Prepare Autoware Source

The `source/` directory is a git submodule pointing to the Autoware repository. You need to clone it and import all package dependencies.

### Option 1: Clone Autoware repository

```bash
cd examples/autoware-2025.02-amd64

# Clone Autoware 2025.02 workspace
git clone --branch 2025.02-ws https://github.com/NEWSLabNTU/autoware.git source

# Import all package dependencies using vcs
cd source
vcs import src < autoware.repos
```

### Option 2: Use git submodules

```bash
cd /path/to/colcon2deb

# Initialize and update submodules
git submodule update --init --recursive examples/autoware-2025.02-amd64/source

# Import dependencies
cd examples/autoware-2025.02-amd64/source
vcs import src < autoware.repos
```

## Build Debian Packages

Once the source is prepared with all dependencies imported:

```bash
cd examples/autoware-2025.02-amd64
./build.sh
```

The build process will:
1. Verify that packages are present in `source/src/`
2. Build a Docker image from the remote Dockerfile
3. Run the build inside the Docker container
4. Generate Debian packages in `build/dist/`

## Output

- **Build artifacts**: `build/`
- **Debian packages**: `build/dist/`
- **Build logs**: `build/log/`

## Configuration

The build is configured via `config.yaml`. Key settings:

- **ROS Distribution**: humble
- **Install Prefix**: `/opt/autoware/2025.02`
- **Dockerfile**: Remote URL from autoware-build-images repository
- **Debian Overrides**: `debian-overrides/` directory for package-specific configurations

## Troubleshooting

### No packages found error

If you see "No ROS packages found in source/src", run:

```bash
cd source
vcs import src < autoware.repos
```

### Source directory not found

Clone the Autoware repository first:

```bash
git clone --branch 2025.02-ws https://github.com/NEWSLabNTU/autoware.git source
```
