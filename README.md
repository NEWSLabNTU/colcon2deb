# colcon2deb

Build Debian packages from ROS 2 colcon workspaces using Docker containers.

## Installation

```bash
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb
just build
just install
```

## Quick Start

1. Create a configuration file:

```yaml
# config.yaml
version: 1

docker:
  dockerfile: ./Dockerfile
  image_name: my-builder

output:
  directory: ./build

build:
  ros_distro: humble
```

2. Run the build:

```bash
colcon2deb --workspace /path/to/ros_ws --config config.yaml
```

3. Find packages in `./build/dist/`.

## Examples

See `examples/` for pre-configured Autoware builds:

```bash
cd examples/autoware-2025.02-amd64
# Prepare workspace (see example README)
just build
```

## Documentation

Full documentation: [book/](book/src/SUMMARY.md)

- [Installation](book/src/user/installation.md)
- [Usage](book/src/user/usage.md)
- [Custom Dockerfiles](book/src/advanced/dockerfile.md)
- [Debian Overrides](book/src/advanced/debian-overrides.md)
- [Architecture](book/src/developer/architecture.md)

## Related Projects

- [autoware-localrepo](https://github.com/NEWSLabNTU/autoware-localrepo) - Complete Autoware packaging with APT repository

## License

Apache License 2.0
