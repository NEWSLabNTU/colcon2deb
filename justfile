# justfile for colcon2deb project

# Read version from pyproject.toml (single source of truth)
VERSION := `grep '^version = ' pyproject.toml | head -1 | sed 's/version = "\(.*\)"/\1/'`

# Show available recipes
default:
    @just --list

# Build Python wheel package for release
build:
    @echo "Building Python wheel package..."
    uv build --wheel
    @echo "Wheel package created in dist/"

# Alias for build
wheel: build

# Install the wheel package
install:
    #!/usr/bin/env bash
    set -euo pipefail

    # Build wheel if it doesn't exist
    if [ ! -f dist/colcon2deb-{{VERSION}}-py3-none-any.whl ]; then
        echo "Wheel not found, building..."
        just build
    fi

    echo "Installing wheel package..."
    pip install --force-reinstall dist/colcon2deb-{{VERSION}}-py3-none-any.whl
    echo "Installed successfully. Use 'colcon2deb --help' to test."

# Install package in development mode
install-dev:
    @echo "Installing in development mode..."
    uv sync
    @echo "Installed. Use 'uv run colcon2deb --help' to test."

# Build Debian packages from workspace
build-workspace workspace output *args:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building Debian packages..."
    echo "  Workspace: {{workspace}}"
    echo "  Output: {{output}}"
    uv run python -m colcon2deb.main \
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
    uv run python -m colcon2deb.main \
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

# Bump version number (pyproject.toml is single source of truth)
bump-version new_version:
    #!/usr/bin/env bash
    set -euo pipefail

    OLD_VERSION=$(grep '^version = ' pyproject.toml | head -1 | sed 's/version = "\(.*\)"/\1/')
    echo "Bumping version from $OLD_VERSION to {{new_version}}"

    # Update pyproject.toml (single source of truth)
    sed -i 's/^version = ".*"/version = "{{new_version}}"/' pyproject.toml

    echo "Updated pyproject.toml"
    echo ""
    echo "Version bumped to {{new_version}}"
    echo "Note: colcon2deb/__init__.py reads version dynamically from package metadata"
