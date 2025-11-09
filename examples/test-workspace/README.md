# Test Workspace for colcon2deb

This is a minimal test workspace for validating the Python-based colcon2deb build system.

## Overview

This workspace contains two simple ROS 2 packages for testing:

- **test_cpp_pkg**: A minimal C++ package with a publisher node
- **test_py_pkg**: A minimal Python package with a subscriber node

The workspace is pre-configured and ready to build - no external dependencies needed.

## Prerequisites

1. **Install colcon2deb** (from project root):
   ```bash
   cd /path/to/colcon2deb
   uv sync
   ```

2. **Docker**: Ensure Docker is installed and running

## Build Debian Packages

The test workspace is already prepared with minimal packages. Simply run:

```bash
cd examples/test-workspace
./build.sh
```

The build process will:
1. Verify that test packages are present
2. Download and build a Docker image (or use cached image)
3. Run the Python builder inside the Docker container
4. Generate Debian packages in `build/dist/`

## Output

After a successful build, you'll find:

- **Build artifacts**: `build/`
- **Debian packages**: `build/dist/`
  - `ros-humble-test-cpp-pkg_1.0.0-*.deb`
  - `ros-humble-test-py-pkg_1.0.0-*.deb`
- **Build logs**: `build/log/`

## Testing the Packages

Install the generated packages:

```bash
# Install both packages
sudo dpkg -i build/dist/ros-humble-test-*.deb

# Run the publisher (in one terminal)
ros2 run test_cpp_pkg test_node

# Run the subscriber (in another terminal)
ros2 run test_py_pkg test_subscriber
```

The publisher should send messages, and the subscriber should receive them.

## Configuration

The build is configured via `config.yaml`. Key settings:

- **ROS Distribution**: humble
- **Install Prefix**: `/opt/ros/humble` (standard ROS location)
- **Dockerfile**: Remote URL from autoware-build-images repository
- **Debian Overrides**: `debian-overrides/` directory (initially empty)

## Use Cases

This test workspace is useful for:

1. **Quick validation** of colcon2deb functionality
2. **Testing Python builder** without large Autoware workspace
3. **Development iteration** when working on colcon2deb itself
4. **CI/CD integration** testing

## Differences from Autoware Examples

Unlike the Autoware examples:

- **No git submodule**: Source is checked directly into the repository
- **No vcs/repos file**: Packages are self-contained
- **Minimal dependencies**: Only standard ROS 2 dependencies (rclcpp, rclpy, std_msgs)
- **Fast builds**: Takes seconds instead of hours

## Extending the Test Workspace

To add more test packages:

1. Create a new package directory in `source/src/`
2. Add a `package.xml` file
3. Add build configuration (`CMakeLists.txt` or `setup.py`)
4. Run `./build.sh` again

## Troubleshooting

### Docker build fails

If the Docker image fails to build, try:

```bash
# Clear Docker cache
docker system prune -a

# Rebuild
./build.sh
```

### Package build fails

Check the build logs:

```bash
cat build/log/colcon_build.log
```

### Missing dependencies

The test packages only depend on standard ROS 2 packages (rclcpp, rclpy, std_msgs) which are included in the base Docker image.
