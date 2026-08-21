"""Package fingerprinting for smart rebuild detection.

Computes a per-package fingerprint from source files, debian overrides,
build configuration, and toolchain version. If the fingerprint matches
the stored one, the package can be skipped.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path


def hash_tree(path: Path, exclude_top_level: frozenset[str] = frozenset()) -> str:
    """Compute a deterministic SHA-256 hash of a directory tree.

    Hashes (sorted_relative_path, file_content) pairs. Returns a fixed
    hex string even for missing or empty directories. Top-level entries
    whose names match exclude_top_level (or start with an excluded name
    ending in '-', e.g. '.obj-') are skipped.
    """
    h = hashlib.sha256()
    if not path.is_dir():
        return h.hexdigest()

    def excluded(rel_first: str) -> bool:
        for name in exclude_top_level:
            if rel_first == name or (name.endswith("-") and rel_first.startswith(name)):
                return True
        return False

    entries: list[tuple[str, bytes]] = []
    for f in sorted(path.rglob("*")):
        if f.is_file() and not f.is_symlink():
            rel = f.relative_to(path)
            if excluded(rel.parts[0]):
                continue
            entries.append((str(rel), f.read_bytes()))

    for rel_str, content in entries:
        h.update(rel_str.encode())
        h.update(content)

    return h.hexdigest()


# Artifacts our own phase 8 leaves inside the package source copy
# (fakeroot debian/rules runs there): they must not affect the source
# fingerprint, or every run invalidates the previous run's cache.
_PKG_DIR_EXCLUDES = frozenset({"debian", ".obj-"})


def compute_fingerprint(
    *,
    pkg_name: str,
    pkg_dir: Path,
    overrides_dir: Path,
    install_prefix: str,
    package_suffix: str,
    ros_distro: str,
    colcon2deb_version: str,
    docker_image_id: str,
) -> dict[str, str]:
    """Build a fingerprint dict for a package.

    pkg_dir is the package's actual source directory (from colcon list) —
    its basename may differ from pkg_name, so it must be hashed directly.
    The 'fingerprint' key is a SHA-256 digest of all tracked inputs.
    The remaining keys are stored for human-readable debugging.
    """
    source_hash = hash_tree(pkg_dir, exclude_top_level=_PKG_DIR_EXCLUDES)
    overrides_hash = hash_tree(overrides_dir / pkg_name)

    combined = hashlib.sha256()
    for value in [
        source_hash,
        overrides_hash,
        install_prefix,
        package_suffix,
        ros_distro,
        colcon2deb_version,
        docker_image_id,
    ]:
        combined.update(value.encode())

    return {
        "fingerprint": combined.hexdigest(),
        "source_hash": source_hash,
        "overrides_hash": overrides_hash,
        "install_prefix": install_prefix,
        "package_suffix": package_suffix,
        "ros_distro": ros_distro,
        "colcon2deb_version": colcon2deb_version,
        "docker_image_id": docker_image_id,
    }


def fingerprint_path(pkg_work_dir: Path, stage: str) -> Path:
    """Per-stage fingerprint file path.

    Each build stage (e.g. 'debian' for phase 7, 'deb' for phase 8) keeps
    its own fingerprint file. Sharing one file caused phase 8 to see the
    fingerprint phase 7 had just written and skip rebuilding the .deb.
    """
    return pkg_work_dir / f".fingerprint.{stage}.json"


def read_fingerprint(path: Path) -> dict[str, str] | None:
    """Read a stored fingerprint from a JSON file. Returns None if missing or invalid."""
    try:
        data = json.loads(path.read_text())
        if isinstance(data, dict) and "fingerprint" in data:
            return data  # type: ignore[return-value]
        return None
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def write_fingerprint(path: Path, fp: dict[str, str]) -> None:
    """Write a fingerprint dict to a JSON file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(fp, indent=2) + "\n")


def fingerprint_matches(stored: dict[str, str] | None, current: dict[str, str]) -> bool:
    """Check if a stored fingerprint matches the current one."""
    if stored is None:
        return False
    return stored.get("fingerprint") == current.get("fingerprint")


def get_fingerprint_inputs_from_env() -> dict[str, str]:
    """Read fingerprint input values from environment variables.

    Used by generate_debian_dir.py and build_deb.py inside the container.
    """
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    return {
        "install_prefix": os.environ.get("ROS_INSTALL_PREFIX", f"/opt/ros/{ros_distro}"),
        "package_suffix": os.environ.get("ROS_PACKAGE_SUFFIX", ""),
        "ros_distro": ros_distro,
        "colcon2deb_version": os.environ.get("COLCON2DEB_VERSION", "unknown"),
        "docker_image_id": os.environ.get("COLCON2DEB_IMAGE_ID", "unknown"),
    }
