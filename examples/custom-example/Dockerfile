FROM ros:humble

# Install python3-pip (required for rosdeb-bloom installation)
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*

# Set up ROS environment sourcing for colcon2deb
RUN echo 'source /opt/ros/humble/setup.bash' > /colcon2deb-setup.sh
