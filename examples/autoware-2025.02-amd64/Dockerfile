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
    && rm -rf /var/lib/apt/lists/*

# Set default process to bash
CMD ["/bin/bash"]
