# Script Execution Model

## Execution Flow

The container-side build process follows a strict execution model to ensure reliability and proper error handling.

## Script Types

### Entry Point Script

`entry.sh` is the Docker container's entry point:
- Creates the ubuntu user with proper permissions
- Sets up the environment
- Executes main.sh as the ubuntu user
- Handles signal forwarding

### Orchestrator Script

`main.sh` coordinates the entire build process:
- Sources all helper scripts
- Manages execution order
- Handles error aggregation
- Controls parallel execution

### Helper Scripts

Individual scripts for specific tasks:
- Executed as functions from main.sh
- Use `exit` for error handling (not `return`)
- Share environment through sourcing
- Write to specific log files

## Error Handling Strategy

### Exit vs Return

All scripts use `exit 1` for error conditions:

```bash
# Correct - propagates error to Docker
if [ ! -d "$WORKSPACE" ]; then
    echo "Error: Workspace not found"
    exit 1
fi

# Incorrect - only returns from function
if [ ! -d "$WORKSPACE" ]; then
    echo "Error: Workspace not found"
    return 1  # Won't stop the build!
fi
```

### Error Propagation

Errors propagate through the execution chain:

1. Helper script encounters error → `exit 1`
2. main.sh catches non-zero exit → logs error
3. entry.sh propagates exit code → container exits
4. colcon2deb.py receives exit code → reports failure

## Environment Management

### Shared Variables

All scripts share these environment variables:

```bash
# Set by entry.sh
WORKSPACE=/workspace
OUTPUT_DIR=/output
WORKDIR=/workdir

# Set by main.sh
BUILD_DIR=$WORKDIR/build
INSTALL_DIR=$WORKDIR/install
ROSDEP_DIR=$WORKDIR/rosdep
```

### ROS Environment

Special handling for ROS environment:

```bash
# Disable unbound variable checking for ROS scripts
set +u
source /opt/ros/humble/setup.bash
source $INSTALL_DIR/setup.bash
set -u
```

## Parallel Execution

### Semaphore Management

Different operations use different semaphore IDs:

```bash
# I/O-bound operations (limited parallelism)
--semaphore-id install_deps_io
--jobs 4

# CPU-bound operations (full parallelism)  
--semaphore-id build_cpu
--jobs $(nproc)

# Package builds (memory-limited)
--semaphore-id build_deb
--jobs $(($(nproc) / 4))
```

### Job Distribution

Intelligent job allocation based on operation type:

| Operation | Type | Parallel Jobs | Semaphore ID |
|-----------|------|--------------|--------------|
| rosdep install | I/O | 4 | install_deps_io |
| colcon build | CPU | nproc | build_cpu |
| dpkg-buildpackage | Mixed | nproc/4 | build_deb |
| File operations | I/O | 1 | N/A |

## Logging Strategy

### Log Files

Each component writes to specific log files:

```bash
# Main process logs
$WORKDIR/logs/main.log

# Package-specific logs
$WORKDIR/logs/build/${package}.log
$WORKDIR/logs/debian/${package}.log
```

### Output Redirection

Consistent output handling:

```bash
# Redirect both stdout and stderr
command > $logfile 2>&1

# Tee for real-time monitoring
command 2>&1 | tee $logfile

# Append to existing logs
command >> $logfile 2>&1
```

## Signal Handling

### Trap Configuration

Proper cleanup on interruption:

```bash
# Set up cleanup trap
cleanup() {
    echo "Cleaning up..."
    # Kill background processes
    jobs -p | xargs -r kill
    exit 1
}

trap cleanup INT TERM EXIT
```

### Process Group Management

Ensure all child processes are terminated:

```bash
# Run in new process group
setsid command &

# Kill entire process group on cleanup
kill -TERM -$$
```

## Best Practices

### Script Headers

All scripts should start with:

```bash
#!/usr/bin/env bash
set -e  # Exit on error
set -u  # Error on undefined variables
set -o pipefail  # Propagate pipe failures
```

### Variable Validation

Always validate critical variables:

```bash
# Check required variables
: ${WORKSPACE:?Error: WORKSPACE not set}
: ${OUTPUT_DIR:?Error: OUTPUT_DIR not set}

# Check directory existence
[ -d "$WORKSPACE" ] || exit 1
```

### Function Organization

Keep functions focused and testable:

```bash
# Good - single responsibility
copy_source() {
    local src=$1
    local dst=$2
    cp -r "$src" "$dst"
}

# Bad - multiple responsibilities
do_everything() {
    copy_source
    build_source
    create_packages
    # Too much in one function!
}
```

## Common Pitfalls

### Unbound Variables in ROS

ROS setup scripts don't work with `set -u`:

```bash
# Temporarily disable for ROS
set +u
source /opt/ros/humble/setup.bash
set -u
```

### Background Process Management

Always wait for background processes:

```bash
# Start background jobs
for pkg in $packages; do
    build_package $pkg &
done

# Wait for all to complete
wait
```

### Path Handling

Use absolute paths and quote variables:

```bash
# Good
cd "$WORKSPACE"
cp -r "$src_dir"/* "$dst_dir"/

# Bad
cd $WORKSPACE  # Breaks with spaces
cp -r $src_dir/* $dst_dir/  # Glob expansion issues
```