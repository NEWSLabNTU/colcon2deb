# API Reference

## Helper Scripts API

This section documents the interface and behavior of each helper script used in the build process.

## entry.sh

**Purpose**: Docker container entry point that sets up the build environment.

**Environment Variables**:
- `UID`: User ID for the ubuntu user (default: 1000)
- `GID`: Group ID for the ubuntu user (default: 1000)

**Behavior**:
1. Creates ubuntu user with specified UID/GID
2. Sets up home directory
3. Fixes permissions on mounted volumes
4. Executes main.sh as ubuntu user
5. Propagates exit code

**Exit Codes**:
- 0: Success
- 1-255: Propagated from main.sh

## main.sh

**Purpose**: Orchestrates the entire build process.

**Environment Variables**:
- `WORKSPACE`: Path to source workspace
- `OUTPUT_DIR`: Path for output packages
- `PARALLEL_JOBS`: Number of parallel jobs
- `VERBOSE`: Enable verbose output (0/1)

**Functions Called**:
```bash
prepare
copy_src
install_deps
build_src
create_rosdep_list
create_package_list
generate_debian_dirs
build_debs
```

**Exit Codes**:
- 0: All stages completed successfully
- 1: Stage failure (with error message)

## prepare.sh

**Purpose**: Initialize the build environment.

**Creates Directories**:
```
/workdir/
├── build/
├── install/
├── rosdep/
├── packages/
└── logs/
    ├── build/
    └── debian/
```

**Environment Exports**:
- `BUILD_DIR`: /workdir/build
- `INSTALL_DIR`: /workdir/install
- `ROSDEP_DIR`: /workdir/rosdep
- `PACKAGE_DIR`: /workdir/packages

**Exit Codes**:
- 0: Environment prepared
- 1: Failed to create directories

## copy-src.sh

**Purpose**: Copy source code from read-only mount.

**Parameters**: None (uses environment)

**Behavior**:
- Copies from `$WORKSPACE` to `$WORKDIR/src`
- Preserves permissions and timestamps
- Excludes build artifacts

**Exit Codes**:
- 0: Source copied successfully
- 1: Copy failed or source not found

## install-deps.sh

**Purpose**: Install system dependencies using rosdep.

**Dependencies**:
- Requires source in `$WORKDIR/src`
- Requires rosdep initialized

**Process**:
1. Generate rosdep commands
2. Execute installation in parallel
3. Verify all dependencies installed

**Exit Codes**:
- 0: Dependencies installed
- 1: Installation failed
- 2: Rosdep not initialized

## build-src.sh

**Purpose**: Compile source code with colcon.

**Build Command**:
```bash
colcon build \
    --base-paths $WORKDIR/src \
    --build-base $BUILD_DIR \
    --install-base $INSTALL_DIR \
    --parallel-workers $PARALLEL_JOBS
```

**Environment Setup**:
- Sources `/opt/ros/humble/setup.bash`
- Sets `ROS_DISTRO=humble`

**Exit Codes**:
- 0: Build completed
- 1: Compilation failed

## create-rosdep-list.sh

**Purpose**: Generate custom rosdep YAML for built packages.

**Output File**: `$ROSDEP_DIR/50-built-packages.yaml`

**Format**:
```yaml
package_name:
  ubuntu: [ros-humble-package-name]
```

**Exit Codes**:
- 0: List created
- 1: Failed to generate

## create-package-list.sh

**Purpose**: Generate list of packages to build.

**Output File**: `$WORKDIR/package-list.txt`

**Filters**:
- Includes only packages with build_type
- Excludes metapackages
- Orders by dependencies

**Exit Codes**:
- 0: List created
- 1: No packages found

## generate-debian-dir.sh

**Purpose**: Create Debian packaging metadata for each package.

**Per Package**:
1. Run bloom-generate debian
2. Apply custom overrides from `/config`
3. Fix version numbers
4. Update dependencies

**Generated Files**:
```
$PACKAGE_DIR/<package>/debian/
├── control
├── rules
├── changelog
├── compat
└── source/
    └── format
```

**Exit Codes**:
- 0: All packages processed
- 1: Bloom generation failed

## build-deb.sh

**Purpose**: Build Debian packages from prepared sources.

**Build Command** (per package):
```bash
dpkg-buildpackage -b -uc -us
```

**Parallel Execution**:
- Uses GNU parallel
- Limits to `$PARALLEL_JOBS / 4` concurrent builds
- Logs to `$WORKDIR/logs/debian/<package>.log`

**Output**:
- `.deb` files copied to `$OUTPUT_DIR`
- Build logs preserved

**Exit Codes**:
- 0: All packages built
- 1: One or more packages failed

## Environment Variables

### Global Variables

Set by entry.sh and main.sh:

| Variable | Description | Example |
|----------|-------------|---------|
| `WORKSPACE` | Source workspace mount | `/workspace` |
| `OUTPUT_DIR` | Output directory mount | `/output` |
| `WORKDIR` | Working directory | `/workdir` |
| `BUILD_DIR` | Colcon build directory | `/workdir/build` |
| `INSTALL_DIR` | Colcon install directory | `/workdir/install` |
| `ROSDEP_DIR` | Custom rosdep lists | `/workdir/rosdep` |
| `PACKAGE_DIR` | Package build area | `/workdir/packages` |

### Configuration Variables

Passed from host:

| Variable | Description | Default |
|----------|-------------|---------|
| `PARALLEL_JOBS` | Build parallelism | `nproc` |
| `VERBOSE` | Verbose output | `0` |
| `DEBUG` | Debug mode | `0` |
| `ROS_DISTRO` | ROS distribution | `humble` |

## File Locations

### Input Files

| Path | Description | Required |
|------|-------------|----------|
| `/workspace/` | Source code | Yes |
| `/config/*/debian/` | Package overrides | No |
| `/helper/*.sh` | Build scripts | Yes |

### Output Files

| Path | Description |
|------|-------------|
| `/output/*.deb` | Built packages |
| `/output/build.log` | Build log |
| `/output/package-list.txt` | Package list |
| `/workdir/logs/` | Detailed logs |

## Customization Hooks

### Package Overrides

Add files to `/config/<package>/debian/`:

```bash
control       # Package metadata
rules        # Build rules  
changelog    # Version history
patches/     # Source patches
shlibs.local # Dependency mappings
```

### Script Hooks

Add custom scripts:

```bash
/config/pre-build.sh   # Before build
/config/post-build.sh  # After build
/config/pre-package.sh # Before packaging
```

### Environment Hooks

Export in entry.sh or main.sh:

```bash
export CUSTOM_CMAKE_ARGS="-DBUILD_TESTING=OFF"
export DEB_BUILD_OPTIONS="nocheck parallel=4"
```

## Error Handling

### Error Propagation

```
Script fails with exit 1
    ↓
main.sh catches error
    ↓
Logs error message
    ↓
Exits with same code
    ↓
entry.sh propagates
    ↓
Container exits
    ↓
colcon2deb.py reports
```

### Error Messages

Standard format:

```
[ERROR] Stage failed: <script_name>
[ERROR] Package: <package_name>
[ERROR] Log: <log_file_path>
```

### Recovery

On failure:
1. Check logs in output directory
2. Fix issues in source
3. Re-run (some artifacts preserved)

## Performance Considerations

### Parallelism Strategy

| Operation | Parallel Factor | Semaphore |
|-----------|----------------|-----------|
| rosdep install | 4 | install_deps_io |
| colcon build | nproc | build_cpu |
| bloom generate | nproc | generate_debian |
| dpkg-buildpackage | nproc/4 | build_deb |

### Resource Usage

Typical per package:
- CPU: 100% of one core during compilation
- Memory: 2-4 GB during C++ compilation
- Disk: 100-500 MB per package
- I/O: Heavy during package creation

### Optimization Tips

1. Use SSD for workdir
2. Increase Docker memory limit
3. Reduce parallel jobs if OOM
4. Enable ccache in Docker image