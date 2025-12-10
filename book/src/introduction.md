# Introduction

colcon2deb builds Debian packages from ROS 2 colcon workspaces using Docker containers.

## Features

- **Isolated builds** - All builds run in Docker containers
- **No host ROS required** - Only Python and Docker needed
- **Multi-architecture** - Supports amd64, arm64, Jetson
- **Customizable** - Override Debian metadata per-package

## Prerequisites

- Python >= 3.10
- Docker
- Colcon workspace with ROS 2 packages

## Quick Start

```bash
# Install
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb && just build && just install

# Run
colcon2deb --workspace /path/to/ros_ws --config config.yaml
```

See [Installation](./user/installation.md) for details.
