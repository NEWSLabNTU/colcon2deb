# Tests

## Layout

- `test_config_validation.py` — config loading/validation (`colcon2deb.config`)
- `test_helper_summary.py` — container-side summary and exit-code logic
- `test_fingerprint.py`, `test_fingerprint_paths.py` — fingerprint hashing and per-stage files
- `test_generate_debian.py` — debian-dir generation and deb-build skip logic (fake bloom)
- `test_ui.py` — host TUI rendering
- `test_events.py` — event stream format
- `test_python_modules.py` — imports and version consistency
- `test_remote_dockerfile.py`, `test_remote_config.py` — remote Dockerfile handling
  (marked `network`; excluded from the default run)
- `test_build_outputs.py`, `test_debian_control.py` — output layout and control-file conventions
- `test_integration.py` — full end-to-end builds in Docker (marked `integration`)
- `fixtures/` — test workspace, configs, and reference files

## Running

```bash
just test        # unit tests (fast; no Docker, no network)
just test-integ  # integration tests (requires Docker; slow on first run)
just test-all    # everything
```

Markers (see `pyproject.toml`): `integration` needs Docker, `network` needs
internet access. The default run excludes both.
