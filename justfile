# justfile for colcon2deb project

VERSION := "0.2.0"

# Show available recipes
default:
    @echo 'Available recipes:'
    @echo ''
    @echo 'just build'
    @echo '    Build Python wheel package for release.'
    @echo ''
    @echo 'just clean'
    @echo '    Clean up build artifacts.'

# Build Python wheel package for release
build:
    @echo "Building Python wheel package..."
    uv build --wheel
    @echo "Wheel package created in dist/"

# Alias for build
wheel: build

# Clean up build artifacts
clean:
    rm -rf dist/
    rm -rf build/
    rm -f *.tar.gz
    @echo "Cleaned up build artifacts"
