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
cd examples/autoware-2025.02-amd64

# Prepare workspace
git clone --branch 2025.02-ws https://github.com/NEWSLabNTU/autoware.git source
cd source && vcs import src < autoware.repos && cd ..

# Build
just build
```

## Output

After a successful build:

```
build/
├── dist/
│   ├── ros-humble-package1_1.0.0-0noble_amd64.deb
│   ├── ros-humble-package2_1.0.0-0noble_amd64.deb
│   └── ...
└── log/
    └── build.log
```

Install packages with:

```bash
sudo dpkg -i build/dist/*.deb
```

## Common Options

| Option | Description |
|--------|-------------|
| `--workspace` | Path to colcon workspace |
| `--config` | Path to config.yaml |
| `--packages-select` | Build specific packages only |

Run `colcon2deb --help` for all options.
