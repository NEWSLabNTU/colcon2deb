# Build Process Overview

## Build Stages

The colcon2deb build process consists of eight sequential stages, each handled by a dedicated script.

## Stage 1: Preparation

**Script**: `prepare.sh`

Initializes the working environment:
- Creates directory structure
- Sets up logging
- Validates prerequisites
- Initializes package lists

```bash
/workdir/
├── build/         # Colcon build output
├── install/       # Colcon install output
├── rosdep/        # Custom rosdep definitions
├── packages/      # Package build directories
└── logs/          # Build logs
```

## Stage 2: Source Copy

**Script**: `copy-src.sh`

Copies source code from read-only mount:
- Preserves directory structure
- Excludes build artifacts
- Maintains file permissions
- Validates copy integrity

## Stage 3: Dependency Installation

**Script**: `install-deps.sh`

Installs required dependencies:
- Generates rosdep commands
- Installs system packages
- Handles missing dependencies
- Creates dependency cache

## Stage 4: Source Build

**Script**: `build-src.sh`

Compiles source with colcon:
- Parallel compilation
- CMake configuration
- Package isolation
- Error collection

## Stage 5: Rosdep List Creation

**Script**: `create-rosdep-list.sh`

Generates custom rosdep mappings:
- Scans built packages
- Creates YAML definitions
- Maps ROS names to Debian names
- Handles version constraints

## Stage 6: Package List Generation

**Script**: `create-package-list.sh`

Identifies packages to build:
- Parses package.xml files
- Filters build types
- Orders by dependencies
- Excludes meta packages

## Stage 7: Debian Metadata Generation

**Script**: `generate-debian-dir.sh`

Creates Debian packaging files:
- Uses bloom-generate
- Applies custom overrides
- Sets version numbers
- Configures dependencies

## Stage 8: Package Building

**Script**: `build-deb.sh`

Builds Debian packages:
- Parallel package builds
- dpkg-buildpackage execution
- Binary package generation
- Output collection

## Process Flow Diagram

```
┌─────────────┐
│  prepare.sh │ Initialize directories
└──────┬──────┘
       │
┌──────▼──────┐
│ copy-src.sh │ Copy source to build area
└──────┬──────┘
       │
┌──────▼──────────┐
│ install-deps.sh │ Install dependencies
└──────┬──────────┘
       │
┌──────▼────────┐
│ build-src.sh  │ Compile with colcon
└──────┬────────┘
       │
┌──────▼────────────────┐
│ create-rosdep-list.sh │ Generate rosdep mappings
└──────┬────────────────┘
       │
┌──────▼─────────────────┐
│ create-package-list.sh │ List packages to build
└──────┬─────────────────┘
       │
┌──────▼─────────────────┐
│ generate-debian-dir.sh │ Create Debian metadata
└──────┬─────────────────┘
       │
┌──────▼────────┐
│ build-deb.sh  │ Build .deb packages
└──────┬────────┘
       │
   ┌───▼───┐
   │ Output│ .deb files in /output
   └───────┘
```

## Error Handling

Each stage implements error handling:

### Fail-Fast Behavior

Any stage failure stops the entire build:

```bash
# In main.sh
prepare || exit 1
copy_src || exit 1
install_deps || exit 1
# ... etc
```

### Error Reporting

Failures are logged with context:

```bash
[ERROR] Stage failed: build-src
[ERROR] Package: autoware_launcher
[ERROR] Log: /workdir/logs/build/autoware_launcher.log
```

### Recovery Options

On failure, you can:
1. Check logs in output directory
2. Fix issues in source workspace
3. Re-run build (some artifacts may be reused)

## Performance Considerations

### Parallel Execution

Different stages use different parallelism:

| Stage | Parallel | Limiting Factor |
|-------|----------|-----------------|
| prepare | No | Sequential setup |
| copy-src | No | I/O bound |
| install-deps | Limited (4) | Network I/O |
| build-src | Full (nproc) | CPU bound |
| create-rosdep | No | Sequential scan |
| create-package | No | Quick scan |
| generate-debian | Yes | Independent packages |
| build-deb | Limited (nproc/4) | Memory bound |

### Resource Usage

Typical resource consumption:

- **CPU**: 100% during compilation
- **Memory**: 2-4 GB per build job
- **Disk**: 10-20 GB for full Autoware
- **Network**: Initial dependency download only

### Optimization Tips

1. **Use SSD storage** for workspace and output
2. **Allocate sufficient RAM** (16 GB minimum)
3. **Enable Docker BuildKit** for better caching
4. **Limit parallel jobs** if running out of memory

## Build Artifacts

### Intermediate Files

Created during build:

```
/workdir/
├── build/          # CMake build files
├── install/        # Installed headers/libraries
├── packages/       # Debian package sources
│   └── <package>/
│       ├── debian/ # Packaging metadata
│       └── obj-*/  # Compilation objects
└── logs/           # Build logs
    ├── build/      # Colcon build logs
    └── debian/     # Package build logs
```

### Final Output

Delivered to output directory:

```
/output/
├── ros-humble-<package>_<version>_amd64.deb
├── build.log
└── package-list.txt
```

## Customization Points

### Build Flags

Configure through environment:

```bash
export COLCON_BUILD_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Release"
export DEB_BUILD_OPTIONS="parallel=4 nocheck"
```

### Package Overrides

Add custom Debian files in `config/<package>/debian/`:
- `control`: Package metadata
- `rules`: Build rules
- `changelog`: Version history
- `patches/`: Source patches

### Script Hooks

Add custom logic:

```bash
# In main.sh
if [ -f /config/pre-build.sh ]; then
    source /config/pre-build.sh
fi
```