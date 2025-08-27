# Common Issues

## Build Failures

### Missing Dependencies

**Problem**: Build fails with "Could not find required package"

**Solution**:
1. Check if package is available in Ubuntu repositories:
   ```bash
   apt-cache search <package-name>
   ```
2. Add missing repository or PPA if needed
3. Update Docker image to include the package
4. For ROS packages, ensure rosdep is properly initialized

### PCL (Point Cloud Library) Errors

**Problem**: Compilation fails with PCL-related errors

**Solution**:
```dockerfile
# Add to Dockerfile
RUN apt-get install -y libpcl-dev
```

**Note**: PCL must be installed in the Docker image, not through rosdep.

### OpenCV Version Conflicts

**Problem**: Multiple OpenCV versions causing conflicts

**Solution**:
1. Use system OpenCV:
   ```dockerfile
   RUN apt-get install -y libopencv-dev
   ```
2. Remove OpenCV from rosdep dependencies
3. Set CMake to use system version

### Python Module Import Errors

**Problem**: "ModuleNotFoundError: No module named 'xmlschema'"

**Solution**:
```dockerfile
# Add missing Python packages
RUN apt-get install -y python3-xmlschema python3-lark
```

## Docker Issues

### Permission Denied

**Problem**: Docker commands fail with permission errors

**Solution**:
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

### Out of Space

**Problem**: "No space left on device" during build

**Solution**:
```bash
# Clean up Docker
docker system prune -a --volumes

# Check disk usage
df -h /var/lib/docker

# Move Docker root to larger partition if needed
```

### Container Can't Access Files

**Problem**: "No such file or directory" for mounted volumes

**Solution**:
1. Use absolute paths in configuration
2. Check file permissions
3. Verify paths exist on host
4. On SELinux systems, add `:Z` to volume mounts

## Dependency Resolution

### Rosdep Failures

**Problem**: "ERROR: the following packages/stacks could not have their rosdep keys resolved"

**Solution**:
1. Update rosdep:
   ```bash
   rosdep update
   ```
2. Check custom rosdep list is being generated
3. Verify package names in package.xml
4. Add missing mappings to rosdep YAML

### Circular Dependencies

**Problem**: "Circular dependency detected"

**Solution**:
1. Check package.xml for incorrect dependencies
2. Separate build and runtime dependencies
3. Use `<build_export_depend>` appropriately

### Version Conflicts

**Problem**: "Version conflict for package X"

**Solution**:
1. Pin specific versions in rosdep
2. Use consistent ROS distro throughout
3. Check for ABI compatibility

## Compilation Errors

### Out of Memory

**Problem**: "c++: fatal error: Killed signal terminated program cc1plus"

**Solution**:
```bash
# Reduce parallel jobs
export PARALLEL_JOBS=$(($(nproc) / 2))

# Or increase swap
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Undefined References

**Problem**: "undefined reference to 'symbol'"

**Solution**:
1. Check library link order
2. Verify all dependencies are installed
3. Ensure proper CMake target linking
4. Check for ABI mismatches

### Header Not Found

**Problem**: "fatal error: header.h: No such file or directory"

**Solution**:
1. Verify include paths in CMakeLists.txt
2. Check if dependency is properly installed
3. Ensure ament_target_dependencies is used
4. Verify package.xml dependencies

## Package Building

### Bloom Generation Failures

**Problem**: "bloom-generate debian failed"

**Solution**:
1. Check package.xml is valid:
   ```bash
   xmllint --noout package.xml
   ```
2. Ensure version number is valid
3. Verify maintainer email is set
4. Check for special characters in descriptions

### dpkg-buildpackage Errors

**Problem**: "dpkg-buildpackage: error: debian/rules build subprocess returned exit status 2"

**Solution**:
1. Check debian/rules file permissions
2. Review build logs in /workdir/logs/debian/
3. Verify all build dependencies are installed
4. Check for hardcoded paths

### Invalid Package Names

**Problem**: "Package name contains invalid characters"

**Solution**:
1. ROS package names use underscores
2. Debian package names use hyphens
3. Ensure proper conversion in bloom

## Performance Issues

### Slow Builds

**Problem**: Builds take excessive time

**Solution**:
1. Use SSD storage
2. Increase Docker CPU/memory limits
3. Enable ccache:
   ```dockerfile
   RUN apt-get install -y ccache
   ENV PATH="/usr/lib/ccache:$PATH"
   ```
4. Use local Docker registry for base images

### High Memory Usage

**Problem**: System becomes unresponsive during builds

**Solution**:
```bash
# Limit parallel jobs
export PARALLEL_JOBS=2

# Set memory limits in Docker
docker run --memory="8g" --memory-swap="8g" ...
```

### Network Timeouts

**Problem**: Package downloads timeout

**Solution**:
1. Use local APT mirror
2. Configure Docker to use host network:
   ```bash
   docker run --network=host ...
   ```
3. Increase timeout values
4. Use package caching proxy

## Debug Techniques

### Entering Failed Container

```bash
# Keep container running after failure
docker run -it --entrypoint /bin/bash \
    -v /path/to/workspace:/workspace:ro \
    -v /path/to/output:/output \
    colcon2deb:0.45.1-amd64

# Run build manually
su - ubuntu
cd /workdir
/helper/main.sh
```

### Verbose Output

Enable detailed logging:
```bash
# In config.yaml
debug: true
verbose: true

# Or environment variable
export VERBOSE=1
```

### Step-by-Step Execution

Run individual stages:
```bash
# Inside container
/helper/prepare.sh
/helper/copy-src.sh
/helper/install-deps.sh
# Check state after each step
```

### Log Analysis

Check logs in order:
1. `/output/build.log` - Overall build log
2. `/workdir/logs/build/<package>.log` - Colcon build logs
3. `/workdir/logs/debian/<package>.log` - Package build logs
4. `/var/log/apt/` - Dependency installation logs