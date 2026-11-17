"""Rewrite the build workspace's install paths in a built .deb to the install prefix.

The dh build of each package resolves its dependencies from the colcon workspace's
isolated install (``COLCON_INSTALL_PATH``, e.g. ``/output/workspace/install``), so CMake
export files, pkg-config files and ament environment hooks can record paths such as
``/output/workspace/install/autoware_point_types/include``. Those paths exist only inside
the build container: on a user's machine a package that finds such a dependency fails at
CMake generate ("includes non-existent path"). Every package of one build installs into the
same merged prefix (``ROS_INSTALL_PREFIX``), so ``<colcon install>/<pkg>`` maps to that
prefix.

The rewrite is done on the finished .deb (unpack, edit, repack) rather than in
``debian/rules``, so that it applies to packages whose ``debian/`` comes from an override
written by an older template as well.
"""

from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

# Text files that carry search paths for other packages' builds and environments.
SUFFIXES = (".cmake", ".pc", ".dsv")


def install_path_pattern(colcon_install_path: str) -> re.Pattern[str]:
    """``<colcon install>/<pkg>`` (one path component), as it appears in a text file."""
    base = re.escape(colcon_install_path.rstrip("/"))
    return re.compile(base + r"/[A-Za-z0-9_.+-]+")


def rewrite_text(text: str, colcon_install_path: str, install_prefix: str) -> str:
    """``text`` with every ``<colcon install>/<pkg>`` replaced by ``install_prefix``."""
    return install_path_pattern(colcon_install_path).sub(install_prefix.rstrip("/"), text)


def relocate_tree(root: Path, colcon_install_path: str, install_prefix: str) -> list[Path]:
    """Rewrite the matching files under ``root`` in place; returns the files changed."""
    changed: list[Path] = []
    needle = colcon_install_path.rstrip("/") + "/"
    for f in sorted(root.rglob("*")):
        if not f.is_file() or f.is_symlink() or f.suffix not in SUFFIXES:
            continue
        try:
            text = f.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        if needle not in text:
            continue
        f.write_text(rewrite_text(text, colcon_install_path, install_prefix))
        changed.append(f)
    return changed


def _write_md5sums(root: Path) -> None:
    """Regenerate ``DEBIAN/md5sums`` for the unpacked package at ``root``."""
    lines: list[str] = []
    for f in sorted(root.rglob("*")):
        rel = f.relative_to(root)
        if rel.parts[0] == "DEBIAN" or not f.is_file() or f.is_symlink():
            continue
        digest = hashlib.md5(f.read_bytes()).hexdigest()  # noqa: S324 (dpkg's format)
        lines.append(f"{digest}  {rel.as_posix()}\n")
    (root / "DEBIAN" / "md5sums").write_text("".join(lines))


def relocate_deb(deb: Path, colcon_install_path: str, install_prefix: str) -> list[str]:
    """Rewrite ``deb`` in place when it holds build paths; returns the files changed.

    The package is unpacked with ``dpkg-deb -R``, the files are rewritten, ``md5sums`` is
    regenerated, and the package is rebuilt with ``dpkg-deb -b`` (as root-owned files, the
    way dh_builddeb builds under fakeroot). An unchanged package is left untouched.
    """
    if not colcon_install_path:
        return []
    with tempfile.TemporaryDirectory(prefix="relocate-") as tmp:
        root = Path(tmp) / "pkg"
        subprocess.run(["dpkg-deb", "-R", str(deb), str(root)], check=True, capture_output=True)
        changed = relocate_tree(root, colcon_install_path, install_prefix)
        if not changed:
            return []
        if (root / "DEBIAN" / "md5sums").exists():
            _write_md5sums(root)
        out = Path(tmp) / deb.name
        subprocess.run(
            ["dpkg-deb", "--root-owner-group", "-b", str(root), str(out)],
            check=True,
            capture_output=True,
        )
        shutil.move(str(out), str(deb))
        return [p.relative_to(root).as_posix() for p in changed]
