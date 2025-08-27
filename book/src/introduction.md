# Introduction

## What is colcon2deb?

colcon2deb is a tool that builds Debian packages for ROS 2 and Autoware projects in isolated Docker containers. It bridges the gap between ROS's colcon build system and Debian's packaging infrastructure, enabling reproducible package builds without requiring ROS to be installed on the host system.

## Key Features

- **Isolated Builds**: All builds run in Docker containers, ensuring consistency and preventing system contamination
- **No Host ROS Required**: Only Python and Docker needed on the host system
- **Parallel Processing**: Optimized for multi-core systems with intelligent parallel job distribution
- **Custom Package Overrides**: Support for package-specific Debian configurations
- **Multi-Architecture Support**: Build for amd64, arm64, and Jetpack platforms

## Architecture Overview

The tool implements a two-program architecture:

1. **Host-side CLI** (`colcon2deb.py`): Python orchestrator that manages Docker containers
2. **Container-side Scripts** (`helper/`): Bash scripts that perform the actual build operations

This separation ensures clean interfaces and prevents accidental modifications to the source code.

## Use Cases

- **CI/CD Integration**: Automate Debian package creation for ROS projects
- **Distribution Packaging**: Create installable packages for Ubuntu/Debian systems
- **Reproducible Builds**: Ensure consistent package builds across different environments
- **Cross-compilation**: Build packages for different architectures from a single host

## Prerequisites

Before using colcon2deb, ensure you have:

- Python >= 3.10
- Docker or Docker CE installed and running
- Sufficient disk space for Docker images and build artifacts
- Source code for the ROS/Autoware workspace you want to package

## Quick Start

```bash
# Install colcon2deb
wget https://github.com/autowarefoundation/colcon2deb/releases/download/v0.2.0/colcon2deb_0.2.0-1_all.deb
sudo apt install ./colcon2deb_0.2.0-1_all.deb

# Build packages from an existing workspace
colcon2deb --workspace /path/to/workspace --config config.yaml
```

For detailed installation and usage instructions, see the [Getting Started](./getting-started.md) chapter.