# Phase 1: Package Fingerprinting for Smart Rebuilds

## Goal

Replace the naive "skip if .deb exists" logic with Cargo-style fingerprinting
that detects when inputs have changed and only rebuilds what's necessary.

## Background

Currently `build_deb.py` skips a package if a matching `.deb` file exists in
`debs/`. This misses changes to source code, debian-overrides, config
(install_prefix, package_suffix), and the Docker image. Users must manually
delete `.deb` files to force rebuilds.

## Design

Per-package fingerprint = SHA-256 of:
- Source tree hash (`workspace/src/{pkg}/`)
- Debian overrides hash (`debian-overrides/{pkg}/`)
- Build config: `install_prefix`, `package_suffix`, `ros_distro`
- Toolchain: `colcon2deb` version, Docker image ID

Stored at `packaging/{pkg}/.fingerprint.json`.

Both `generate_debian_dir.py` (phase 7) and `build_deb.py` (phase 8) check
the fingerprint before doing work. If the fingerprint matches, skip. If not,
rebuild and write the new fingerprint.

## Work Items

- [x] Create `colcon2deb/helper/fingerprint.py` module
  - [x] `hash_tree(path) -> str` — deterministic SHA-256 of a directory tree
  - [x] `compute_fingerprint(pkg_name, ...) -> dict` — build the fingerprint dict
  - [x] `read_fingerprint(path) -> dict | None` — read stored fingerprint
  - [x] `write_fingerprint(path, fp)` — write fingerprint to JSON
  - [x] `fingerprint_matches(stored, current) -> bool` — compare
- [x] Pass Docker image ID from host into container via env var
  - [x] Host `main.py`: capture image ID after docker build/pull
  - [x] Pass as `COLCON2DEB_IMAGE_ID` env var to container
- [x] Integrate into `generate_debian_dir.py` (phase 7)
  - [x] Compute fingerprint before generating debian/
  - [x] Skip if fingerprint matches and `packaging/{pkg}/debian/` exists
  - [x] Write fingerprint after successful generation
- [x] Integrate into `build_deb.py` (phase 8)
  - [x] Replace existing `find_deb_file` skip check
  - [x] Skip only if fingerprint matches AND .deb exists
  - [x] Write fingerprint after successful build
- [x] Unit tests for fingerprint module
  - [x] `hash_tree` produces deterministic output
  - [x] `hash_tree` detects file additions, deletions, modifications
  - [x] `hash_tree` handles empty directories and missing directories
  - [x] `compute_fingerprint` changes when any input changes
  - [x] `fingerprint_matches` returns False on mismatch
  - [x] Round-trip: write then read back
- [x] Integration tests
  - [x] First build: all packages built (not skipped)
  - [x] Second build (no changes): all packages skipped
  - [x] Fingerprint JSON files written with expected fields
  - [x] Source change + deb removal triggers rebuild
  - [x] Debian-override change + deb removal triggers rebuild

## Acceptance Criteria

- [x] `just test` passes — 96 unit tests (21 fingerprint + 75 existing)
- [x] `just test-integ` passes — 26 integration tests (5 fingerprint + 21 existing)
- [x] `just check` passes (lint + type check)
- [x] Running colcon2deb twice with no changes shows all packages "cached"
- [x] Modifying a debian-override triggers rebuild of that package

## Verified

- Autoware 1.5.0 (453 packages): second run shows "Built 0 packages (453 cached)"
- Touching `debian-overrides/autoware_lint_common/debian/control` + removing its .deb:
  "Built 1 packages (452 cached)" — only the affected package rebuilt
