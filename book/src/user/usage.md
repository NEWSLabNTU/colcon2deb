# Usage

## Configuration File

Create a `config.yaml`:

```yaml
version: 1

docker:
  dockerfile: ./Dockerfile    # or use 'image: my-image:tag'
  image_name: my-builder

output:
  directory: ./build

packages:
  directory: ./debian-overrides

build:
  ros_distro: humble
```

## Running a Build

```bash
colcon2deb --workspace /path/to/ros_ws --config config.yaml
```

## Using Examples

Pre-configured examples are available:

```bash
cd examples/simple-example

# Prepare workspace (see the example's README)
# Build
just build
```

## Output

After a successful build:

```
build/
├── debs/
│   ├── ros-humble-package1_1.0.0-0jammy_amd64.deb
│   ├── ros-humble-package2_1.0.0-0jammy_amd64.deb
│   └── ...
└── logs/
    └── latest/
        ├── phases/     # one log per build phase
        ├── packages/   # per-package logs
        └── reports/    # summary.txt, successful.txt, failed.txt, skipped.txt
```

Install packages with:

```bash
sudo dpkg -i build/debs/*.deb
```

## Common Options

| Option | Description |
|--------|-------------|
| `--workspace` | Path to colcon workspace |
| `--config` | Path to config.yaml |

Run `colcon2deb --help` for all options.
