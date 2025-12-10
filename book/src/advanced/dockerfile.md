# Custom Dockerfiles

## When to Use

Use a custom Dockerfile when:
- Building for specific Autoware/ROS versions
- Targeting ARM64 or Jetson platforms
- Pre-installing specific dependencies

## Required Components

Your Dockerfile must include:

```dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Build tools
RUN apt-get update && apt-get install -y \
    build-essential cmake git \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    python3-bloom fakeroot dpkg-dev debhelper

# ROS 2 (example for Humble)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y ros-humble-ros-base

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro humble
```

## Platform Examples

### AMD64 (Standard)

```dockerfile
FROM ubuntu:22.04
# Standard installation as above
```

### ARM64

```dockerfile
FROM arm64v8/ubuntu:22.04
# Same packages, ARM-native
```

### NVIDIA Jetson (Jetpack)

```dockerfile
FROM nvcr.io/nvidia/l4t-base:r35.3.1

ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}

# Install ROS on top of L4T
# ...
```

## Configuration

Reference your Dockerfile in config.yaml:

```yaml
docker:
  dockerfile: ./Dockerfile
  image_name: my-builder:latest
```

Or use a remote Dockerfile:

```yaml
docker:
  dockerfile: https://raw.githubusercontent.com/.../Dockerfile
```

## Pre-built Images

See [autoware-build-images](https://github.com/NEWSLabNTU/autoware-build-images) for ready-to-use images.
