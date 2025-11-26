# Autoware 2025.02 AMD64 Debian Package Build

This example demonstrates how to build Debian packages for Autoware 2025.02 on AMD64 architecture.

## Prerequisites

Go to the project directory and run `just install`.

## Prepare Autoware Source

The `source/` directory is a git submodule pointing to the Autoware repository. You need to clone it and import all package dependencies.


```bash
cd examples/autoware-2025.02-amd64
git clone --branch 2025.02-ws --recursive https://github.com/NEWSLabNTU/autoware.git source
```

## Build Debian Packages

Once the source is prepared with all dependencies imported:

```bash
cd examples/autoware-2025.02-amd64
just build
```

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
