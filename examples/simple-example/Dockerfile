FROM ros:humble

# Install build dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    fakeroot \
    debhelper \
    dh-python \
    rsync \
    parallel \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment sourcing for colcon2deb
RUN echo 'source /opt/ros/humble/setup.bash' > /colcon2deb-setup.sh
