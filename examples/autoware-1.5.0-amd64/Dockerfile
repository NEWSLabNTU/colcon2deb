# Use date-pinned tag for reproducible builds
# Format: cuda-YYYYMMDD (based on Autoware 2025.02 release date: 2025-02-05)
FROM ghcr.io/autowarefoundation/autoware-base:cuda-20250205

# Remove the ROS2 repo with expired GPG key
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list || true

# Install curl to download new ROS GPG key
RUN apt update -y && \
    apt install -y curl && \
    rm -rf /var/lib/apt/lists/*

# Fix ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Configure AutonomouStuff APT repository for pacmod packages
RUN apt update -y && \
    apt install -y apt-transport-https && \
    rm -rf /var/lib/apt/lists/*
RUN echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/autonomoustuff-public.list > /dev/null

# Configure the locale and timezone (if not already set)
RUN apt update -y && \
    apt install -y locales && \
    echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen && \
    locale-gen && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN ln -snf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && \
    echo Asia/Taipei > /etc/timezone

# Install required dependencies to build Debian packages
RUN apt update -y && \
    apt install -y \
    parallel \
    fakeroot \
    debhelper \
    dh-python \
    rsync \
    sudo \
    python3-bloom \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# Fix libdrm and Mesa version conflicts by downgrading to Ubuntu 22.04 versions
# The base image includes newer packages from kisak PPA for CUDA support,
# but standard Ubuntu packages require exact version matches
RUN apt update -y && \
    apt install -y --allow-downgrades \
        libdrm2=2.4.113-2~ubuntu0.22.04.1 \
        libdrm-intel1=2.4.113-2~ubuntu0.22.04.1 \
        libdrm-radeon1=2.4.113-2~ubuntu0.22.04.1 \
        libdrm-amdgpu1=2.4.113-2~ubuntu0.22.04.1 \
        libdrm-nouveau2=2.4.113-2~ubuntu0.22.04.1 \
        libdrm-common=2.4.113-2~ubuntu0.22.04.1 \
        libglapi-mesa=23.2.1-1ubuntu3.1~22.04.3 \
        libgl1-mesa-dri=23.2.1-1ubuntu3.1~22.04.3 \
        libglx-mesa0=23.2.1-1ubuntu3.1~22.04.3 \
        libegl-mesa0=23.2.1-1ubuntu3.1~22.04.3 \
        libgbm1=23.2.1-1ubuntu3.1~22.04.3 \
        mesa-vulkan-drivers=23.2.1-1ubuntu3.1~22.04.3 \
    && rm -rf /var/lib/apt/lists/*

# Install cudnn_cmake_module for TensorRT support
RUN apt update -y && \
    apt install -y ros-humble-cudnn-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# Install message_filters - required by nebula_ros but sometimes not detected by rosdep
RUN apt update -y && \
    apt install -y ros-humble-message-filters \
    && rm -rf /var/lib/apt/lists/*

# Pre-install rosdep dependencies to speed up builds
# These packages were extracted from build/log/rosdep_simulate.log
COPY rosdep-packages.txt /tmp/rosdep-packages.txt
RUN apt update -y && \
    grep -v '^#' /tmp/rosdep-packages.txt | grep -v '^$' | xargs apt-get install -y --no-install-recommends || true && \
    rm -rf /var/lib/apt/lists/* /tmp/rosdep-packages.txt

# Set default process to bash
CMD ["/bin/bash"]
