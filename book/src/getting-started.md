# Getting Started

## Installation

### From Debian Package (Recommended)

Download and install the latest release:

```bash
wget https://github.com/autowarefoundation/colcon2deb/releases/download/v0.2.0/colcon2deb_0.2.0-1_all.deb
sudo apt install ./colcon2deb_0.2.0-1_all.deb
```

### From Source

Clone the repository and install dependencies:

```bash
git clone https://github.com/autowarefoundation/colcon2deb.git
cd colcon2deb

# Install uv (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Set up the development environment
uv sync

# Link system packages for apt_pkg
make setup-apt

# Build and install the Debian package
make deb
sudo dpkg -i dist/colcon2deb_0.2.0-1_all.deb
```

## Configuration

### Basic Configuration File

Create a `config.yaml` file specifying your build parameters:

```yaml
# Workspace and output directories
workspace_dir: /path/to/autoware/workspace
output_dir: /path/to/output

# Docker image configuration
image: colcon2deb:0.45.1-amd64
```

### Using Remote Dockerfiles

You can also specify a Dockerfile URL instead of a prebuilt image:

```yaml
workspace_dir: /path/to/workspace
output_dir: /path/to/output
dockerfile: https://raw.githubusercontent.com/autowarefoundation/colcon2deb/main/docker/0.45.1/amd64/Dockerfile
```

## Building Docker Images

### Standard Images

Build images for different Autoware versions:

```bash
# Autoware 0.45.1
docker build -t colcon2deb:0.45.1-amd64 docker/0.45.1/amd64/

# Autoware 2025.02
docker build -t colcon2deb:2025.02-amd64 docker/2025.02/amd64/
```

### ARM64 Support

Build for ARM64 architecture:

```bash
docker build -t colcon2deb:0.45.1-arm64 docker/0.45.1/arm64/
```

### Jetpack Support

For NVIDIA Jetson devices:

```bash
docker build -t colcon2deb:0.45.1-jetpack docker/0.45.1/jetpack-6.0/
```

## Running Your First Build

### Using Example Scripts

The quickest way to get started is using the provided example scripts:

```bash
cd examples/autoware-0.45.1-amd64
./build.sh
```

This script will:
1. Clone the Autoware source code
2. Build the Docker image
3. Run colcon2deb to create Debian packages
4. Output packages to the `output` directory

### Manual Build Process

For more control over the build process:

```bash
# 1. Prepare your workspace
git clone https://github.com/autowarefoundation/autoware.git -b 0.45.1-ws workspace

# 2. Create configuration
cat > config.yaml <<EOF
workspace_dir: $(pwd)/workspace
output_dir: $(pwd)/output
image: colcon2deb:0.45.1-amd64
EOF

# 3. Run the build
colcon2deb --config config.yaml
```

## Understanding the Output

After a successful build, you'll find:

- **Debian packages** (`.deb` files) in the output directory
- **Build logs** for each package
- **Dependency information** used during the build

The packages can be:
- Installed directly with `dpkg -i`
- Added to an APT repository
- Distributed to other systems

## Troubleshooting

### Common Issues

#### Docker Permission Denied

If you get permission errors when running Docker:

```bash
# Add your user to the docker group
sudo usermod -aG docker $USER

# Log out and back in for changes to take effect
```

#### Missing Dependencies

If builds fail due to missing dependencies:

1. Check that your Docker image has all required packages
2. Verify the workspace structure is correct
3. Review logs in the output directory

#### Out of Disk Space

Docker builds can consume significant space. Clean up with:

```bash
# Remove unused Docker images
docker system prune -a

# Clear build artifacts
rm -rf output/*
```

For more detailed troubleshooting, see the [Troubleshooting Guide](./troubleshooting/common-issues.md).