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
        └── reports/    # summary.txt, successful.txt, failed.txt, skipped.txt,
                        # link_check.txt and link_check.json
```

## Relocation and the link check

Each package's dh build resolves its dependencies from the colcon workspace's install
(`/output/workspace/install/<pkg>` inside the container), so its CMake export files,
pkg-config files and `.dsv` environment hooks can record those paths. After each `.deb` is
built, colcon2deb rewrites every `/output/workspace/install/<pkg>` in its `*.cmake`, `*.pc`
and `*.dsv` files to `build.install_prefix` and logs the files it changed in the package's
`build_deb.log`. Without this, a package built on the target machine against these debs
fails at CMake generate ("includes non-existent path").

Phase 9 then unpacks every `.deb` of the output into one tree and checks it:

- `ldd -r` on every ELF file. An undefined data symbol (an object, or a C++ vtable,
  typeinfo or guard variable) makes `dlopen` fail and is an **error**; an undefined
  function fails only when called and is a **warning**. A library that uses another
  package's class without linking it (a dependency used only through its headers) shows up
  here.
- `dpkg-shlibdeps` warnings in the package logs ("contains an unresolvable reference to
  symbol"), classified the same way.
- Any text file under `share/` that still names the build container's directories is an
  error.

```yaml
build:
  link_check: error              # error | warn (default) | off
  link_check_allow: link-check-allow.yaml   # optional, relative to config.yaml
```

The allowlist is a list of entries, each with a reason:

```yaml
- package: ros-humble-nebula-*       # glob on the .deb's package name (default: any)
  symbol: '^nebula::ros::.*Offline'  # regex on the raw or demangled symbol
  reason: examples declare but never define these; not loaded by any launch
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
