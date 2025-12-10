# Simple Example

Minimal example with two ROS 2 packages:
- **test_cpp_pkg**: C++ publisher node
- **test_py_pkg**: Python subscriber node

## Build

```bash
# From project root, ensure venv is set up
uv sync

# Build packages
cd examples/simple-example
just build
```

## Output

Debian packages in `build/dist/`:
- `ros-humble-test-cpp-pkg_*.deb`
- `ros-humble-test-py-pkg_*.deb`

## Test

```bash
sudo dpkg -i build/dist/ros-humble-test-*.deb
ros2 run test_cpp_pkg test_node      # terminal 1
ros2 run test_py_pkg test_subscriber  # terminal 2
```
