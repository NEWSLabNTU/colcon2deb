# Docker Configuration

## Base Images

colcon2deb provides Docker images for different Autoware versions and architectures.

## Image Structure

### Directory Layout

```
docker/
├── 0.45.1/
│   ├── amd64/
│   │   └── Dockerfile
│   ├── arm64/
│   │   └── Dockerfile
│   └── jetpack-6.0/
│       └── Dockerfile
└── 2025.02/
    └── amd64/
        └── Dockerfile
```

### Base Image Selection

Images are based on official Ubuntu releases:

| Autoware Version | Ubuntu Version | ROS Version |
|-----------------|---------------|-------------|
| 0.45.1 | 22.04 (Jammy) | Humble |
| 2025.02 | 22.04 (Jammy) | Humble |

## Required Components

### System Packages

Every Docker image must include:

```dockerfile
# Build essentials
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool

# ROS 2 packages
RUN apt-get install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp

# Development tools
RUN apt-get install -y \
    python3-bloom \
    python3-colcon-common-extensions \
    fakeroot \
    dpkg-dev \
    debhelper
```

### Critical Dependencies

These packages must be pre-installed to avoid build failures:

```dockerfile
# Point Cloud Library (required by many Autoware packages)
RUN apt-get install -y libpcl-dev

# OpenCV (computer vision)
RUN apt-get install -y libopencv-dev

# Python XML Schema (for parameter validation)
RUN apt-get install -y python3-xmlschema

# Parallel processing
RUN apt-get install -y parallel

# Git LFS support
RUN apt-get install -y git-lfs && git lfs install
```

### ROS Repository Setup

Configure ROS APT repository:

```dockerfile
# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
```

### Rosdep Initialization

Initialize rosdep for dependency resolution:

```dockerfile
# Initialize rosdep
RUN rosdep init || true
RUN rosdep update --rosdistro humble

# Create rosdep cache directory
RUN mkdir -p /home/ubuntu/.ros/rosdep
```

## Platform-Specific Configurations

### AMD64 Architecture

Standard configuration for x86_64 systems:

```dockerfile
FROM ubuntu:22.04
# Standard installation as shown above
```

### ARM64 Architecture

Optimized for ARM processors:

```dockerfile
FROM arm64v8/ubuntu:22.04

# May need architecture-specific packages
RUN dpkg --add-architecture arm64
```

### NVIDIA Jetpack

Special configuration for Jetson devices:

```dockerfile
FROM nvcr.io/nvidia/l4t-base:r35.3.1

# CUDA and cuDNN are pre-installed
# Add ROS on top of L4T base

ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
```

## Build Optimization

### Layer Caching

Organize Dockerfile for optimal caching:

```dockerfile
# System packages (changes rarely)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake

# ROS packages (changes occasionally)
RUN apt-get install -y \
    ros-humble-desktop

# Project-specific packages (changes frequently)
RUN apt-get install -y \
    libpcl-dev \
    libopencv-dev

# Helper scripts (changes most frequently)
COPY helper/ /helper/
```

### Multi-Stage Builds

For smaller final images:

```dockerfile
# Build stage
FROM ubuntu:22.04 AS builder
RUN apt-get update && apt-get install -y build-essential
# ... build operations ...

# Runtime stage
FROM ubuntu:22.04
COPY --from=builder /output /output
# Only runtime dependencies
```

### Build Arguments

Support customization through build args:

```dockerfile
ARG ROS_DISTRO=humble
ARG UBUNTU_VERSION=22.04

FROM ubuntu:${UBUNTU_VERSION}
RUN apt-get install -y ros-${ROS_DISTRO}-desktop
```

## Entry Point Configuration

### User Setup

Create non-root user for builds:

```dockerfile
# Don't create user in Dockerfile
# Let entry.sh create it with proper UID/GID

COPY helper/entry.sh /entry.sh
ENTRYPOINT ["/entry.sh"]
```

### Working Directory

Set up proper working directory:

```dockerfile
# Create base directories
RUN mkdir -p /workspace /output /workdir

# Don't set WORKDIR - let entry.sh handle it
```

## Environment Variables

### ROS Environment

```dockerfile
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
```

### Build Environment

```dockerfile
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV FORCE_COLOR=1
```

### Locale Settings

```dockerfile
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
```

## Security Considerations

### Package Verification

Always verify package signatures:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    lsb-release
```

### Minimize Attack Surface

Remove unnecessary packages:

```dockerfile
RUN apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
```

### Non-Root Execution

Never run builds as root:

```bash
# In entry.sh
useradd -m -s /bin/bash -u $UID ubuntu
su - ubuntu -c "/helper/main.sh"
```

## Troubleshooting Docker Issues

### Build Failures

If Docker build fails:

1. Check Docker daemon is running
2. Verify network connectivity for package downloads
3. Ensure sufficient disk space
4. Try building with `--no-cache`

### Runtime Issues

Common runtime problems:

- **Permission denied**: Check volume mount permissions
- **Out of memory**: Increase Docker memory limits
- **Network issues**: Check Docker network configuration

### Debugging Containers

Enter container for debugging:

```bash
# Run with interactive shell
docker run -it --rm \
    -v /path/to/workspace:/workspace:ro \
    -v /path/to/output:/output \
    colcon2deb:0.45.1-amd64 \
    /bin/bash

# Debug specific script
docker run -it --rm \
    -v /path/to/workspace:/workspace:ro \
    -v /path/to/output:/output \
    --entrypoint /bin/bash \
    colcon2deb:0.45.1-amd64 \
    -c "/helper/main.sh"
```