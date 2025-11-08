#!/bin/bash
set -e

# Script to build Autoware 0.45.1 Debian packages
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/source"
AUTOWARE_BRANCH="0.45.1-ws"
AUTOWARE_REPO="https://github.com/NEWSLabNTU/autoware.git"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "=== Building Autoware 0.45.1 AMD64 Debian packages ==="

# Check if colcon2deb is available
if command -v colcon2deb &> /dev/null; then
    COLCON2DEB="colcon2deb"
elif [ -f "${PROJECT_ROOT}/.venv/bin/colcon2deb" ]; then
    echo "Using colcon2deb from project venv..."
    COLCON2DEB="${PROJECT_ROOT}/.venv/bin/colcon2deb"
else
    echo "Error: colcon2deb is not available."
    echo "Please install it first:"
    echo "  cd ${PROJECT_ROOT}"
    echo "  uv sync  # Install dependencies"
    echo "  uv run colcon2deb --help  # Verify installation"
    exit 1
fi

# Clone or update the Autoware repository
if [ -d "${SOURCE_DIR}" ]; then
    echo "Source directory exists. Updating..."
    cd "${SOURCE_DIR}"
    git fetch origin
    git checkout "${AUTOWARE_BRANCH}"
    git pull origin "${AUTOWARE_BRANCH}"
else
    echo "Cloning Autoware repository (branch: ${AUTOWARE_BRANCH})..."
    git clone --branch "${AUTOWARE_BRANCH}" "${AUTOWARE_REPO}" "${SOURCE_DIR}"
fi

# Run colcon2deb
echo "Running colcon2deb..."
cd "${SCRIPT_DIR}"
"${COLCON2DEB}" --workspace "${SOURCE_DIR}" --config config.yaml

echo "=== Build complete ==="
echo "Debian packages are available in: ${SCRIPT_DIR}/build/"
