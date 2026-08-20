# Custom Example

Demonstrates two advanced options:

- **`build.install_prefix: /opt/myproject`** — packages install under a custom
  prefix instead of `/opt/ros/humble`
- **`build.package_suffix: "1.0.0"`** — package names get a version suffix,
  e.g. `ros-humble-test-cpp-pkg-1.0.0` instead of `ros-humble-test-cpp-pkg`

## Build

```bash
# From project root, ensure venv is set up
uv sync

# Prepare the workspace (copies the test packages from tests/fixtures)
cd examples/custom-example
mkdir -p source
cp -r ../../tests/fixtures/workspace/src source/

# Build packages
just build
```

## Output

Debian packages in `build/debs/`:

- `ros-humble-test-cpp-pkg-1.0.0_*.deb`
- `ros-humble-test-py-pkg-1.0.0_*.deb`

Files inside the packages install under `/opt/myproject`.
