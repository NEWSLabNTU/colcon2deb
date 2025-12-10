# Architecture

## Two-Program Design

colcon2deb uses a host/container split:

1. **Host CLI** (`colcon2deb/main.py`): Python orchestrator managing Docker
2. **Container Scripts** (`colcon2deb/helper/`): Build logic running inside Docker

This ensures the host needs only Python and Docker - no ROS installation required.

## Code Structure

```
colcon2deb/
├── colcon2deb/
│   ├── main.py              # CLI entry point
│   └── helper/
│       ├── main.py          # Container orchestrator
│       ├── build_deb.py     # Package builder
│       └── generate_debian_dir.py
├── templates/               # Debian templates
├── examples/                # Example configurations
├── tests/                   # Test suite
├── pyproject.toml           # Package config
└── justfile                 # Build automation
```

## Build Procedure

```
Host                           Container
----                           ---------
1. Parse config.yaml
2. Launch Docker container --> 3. Create build user
                               4. Copy source to workdir
                               5. Install deps (rosdep)
                               6. Build with colcon
                               7. Generate debian/ dirs (bloom)
                               8. Build .deb packages
9. Collect output <----------- Write to /output
```

## Key Design Decisions

- **Read-only source mount**: Prevents accidental modifications
- **Non-root container user**: Security best practice
- **Parallel package builds**: Optimized for multi-core systems
- **Package overrides**: Custom debian/ directories override bloom generation

## Development

```bash
# Run tests
uv run pytest tests/

# Build wheel
just build

# Run from source
uv run colcon2deb --help
```

## Code Style

- Python: PEP 8, type hints
- Shell: `set -e`, quoted variables
