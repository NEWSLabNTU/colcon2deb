# Installation

## Prerequisites

- Python >= 3.10
- Docker
- [uv](https://docs.astral.sh/uv/) (for building from source)

## From Source

```bash
git clone https://github.com/NEWSLabNTU/colcon2deb.git
cd colcon2deb

# Build and install
just build
just install
```

The `colcon2deb` command is now available in your PATH.

## Development Mode

For development or testing without installation:

```bash
# Install uv if needed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Set up development environment
uv sync

# Run directly
uv run colcon2deb --help
```

## Verify Installation

```bash
colcon2deb --help
```
