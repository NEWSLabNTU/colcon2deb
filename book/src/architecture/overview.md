# Architecture Overview

## Design Philosophy

colcon2deb follows several key design principles:

1. **Isolation**: All builds run in Docker containers to prevent system contamination
2. **Read-only Sources**: Source code is mounted read-only to prevent accidental modifications
3. **Clean Interfaces**: Communication happens through volumes, environment variables, and exit codes
4. **No Host ROS**: The host system doesn't need ROS installed, only Python and Docker

## Two-Program Architecture

The system is split into two distinct components:

### Host-Side CLI (`colcon2deb.py`)

The Python orchestrator that runs on the host system:

- Parses configuration files
- Validates input parameters
- Manages Docker containers
- Handles volume mounting
- Reports build status

Key responsibilities:
- Docker container lifecycle management
- Configuration validation
- Output directory management
- Error reporting and logging

### Container-Side Scripts (`helper/`)

Bash scripts that execute inside the Docker container:

- `entry.sh`: Container entry point, sets up the ubuntu user
- `main.sh`: Build orchestrator that coordinates all steps
- `prepare.sh`: Initializes working directories
- `copy-src.sh`: Copies source to build area
- `install-deps.sh`: Installs dependencies via rosdep
- `build-src.sh`: Compiles with colcon
- `create-rosdep-list.sh`: Generates custom rosdep mappings
- `create-package-list.sh`: Lists packages to build
- `generate-debian-dir.sh`: Creates Debian metadata with bloom
- `build-deb.sh`: Builds .deb packages in parallel

## Data Flow

```
Host System                    Docker Container
-----------                    ----------------
colcon2deb.py
    |
    ├── Read config.yaml
    ├── Validate paths
    ├── Launch container ──────> entry.sh
    |                              |
    |                              ├── Create ubuntu user
    |                              └── Execute main.sh
    |                                     |
    |                                     ├── prepare.sh
    |                                     ├── copy-src.sh
    |                                     ├── install-deps.sh
    |                                     ├── build-src.sh
    |                                     ├── create-rosdep-list.sh
    |                                     ├── create-package-list.sh
    |                                     ├── generate-debian-dir.sh
    |                                     └── build-deb.sh
    |                                           |
    └── Collect outputs <──────────────────── Write .deb files
```

## Volume Mounts

The tool uses three primary volume mounts:

1. **Source Mount** (`/workspace`): Read-only mount of the ROS workspace
2. **Output Mount** (`/output`): Read-write mount for generated packages
3. **Config Mount** (`/config`): Read-only mount of package overrides

## Environment Variables

Key environment variables passed to the container:

- `WORKSPACE`: Path to the source workspace
- `OUTPUT_DIR`: Path to output directory
- `PARALLEL_JOBS`: Number of parallel build jobs
- `BUILD_FLAGS`: Additional colcon build flags

## Security Considerations

- Source code mounted read-only to prevent modifications
- Container runs as non-root user (ubuntu)
- No network access during build (can be configured)
- Isolated build environment prevents system contamination

## Performance Optimizations

### Parallel Processing

The build system uses GNU parallel for optimal performance:

- I/O-bound operations: Limited parallelism
- CPU-bound operations: Full parallelism
- Semaphore-based job control

### Caching Strategy

- Docker layer caching for base images
- ccache integration for C++ compilation
- Persistent rosdep cache between builds

## Error Handling

The system implements comprehensive error handling:

- Exit codes propagate from container to host
- Each script validates prerequisites
- Build failures are logged per package
- Partial builds can be resumed

## Extension Points

The architecture supports several extension mechanisms:

- Custom Dockerfiles for different platforms
- Package-specific Debian overrides in `config/`
- Additional helper scripts can be added
- Configuration file supports custom parameters