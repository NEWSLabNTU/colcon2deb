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
    @echo ''
    @echo 'just build-workspace WORKSPACE OUTPUT [OPTIONS]'
    @echo '    Build Debian packages from workspace (using Python builder).'
    @echo ''
    @echo 'just build-example EXAMPLE'
    @echo '    Build packages for an example (autoware-2025.02-amd64, etc).'
    @echo ''
    @echo 'just install-dev'
    @echo '    Install package in development mode.'
    @echo ''
    @echo 'just test'
    @echo '    Run tests.'

# Build Python wheel package for release
build:
    @echo "Building Python wheel package..."
    uv build --wheel
    @echo "Wheel package created in dist/"

# Alias for build
wheel: build

# Install package in development mode
install-dev:
    @echo "Installing in development mode..."
    uv sync
    @echo "Installed. Use 'uv run autoware-deb-builder --help' to test."

# Build Debian packages from workspace
build-workspace workspace output *args:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building Debian packages..."
    echo "  Workspace: {{workspace}}"
    echo "  Output: {{output}}"
    uv run python -m autoware_debian_packager.main \
        --workspace {{workspace}} \
        --output {{output}} \
        {{args}}

# Build packages for an example directory
build-example example:
    #!/usr/bin/env bash
    set -euo pipefail
    cd examples/{{example}}
    if [ ! -d source ]; then
        echo "Error: source directory not found in examples/{{example}}"
        echo "Please prepare the workspace first. See README.md"
        exit 1
    fi
    uv run python -m autoware_debian_packager.main \
        --workspace source \
        --output build

# Run tests
test:
    uv run pytest tests/ -v

# Clean up build artifacts
clean:
    rm -rf dist/
    rm -rf build/
    rm -f *.tar.gz
    @echo "Cleaned up build artifacts"

# Clean example build directories
clean-examples:
    #!/usr/bin/env bash
    for dir in examples/*/build; do
        if [ -d "$dir" ]; then
            echo "Cleaning $dir"
            rm -rf "$dir"
        fi
    done
    @echo "Cleaned example build directories"
