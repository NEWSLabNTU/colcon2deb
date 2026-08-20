"""Tests for fingerprint correctness fixes.

Covers two production bugs:
1. compute_fingerprint hashed source_dir/pkg_name, which is wrong whenever
   the package directory basename differs from the colcon package name —
   the hash silently became the empty-tree constant and never changed.
2. Phases 7 and 8 shared one .fingerprint.json file, so phase 7 writing the
   new fingerprint made phase 8 think nothing changed and skip the rebuild.
"""

from __future__ import annotations

from pathlib import Path

from colcon2deb.helper.fingerprint import (
    compute_fingerprint,
    fingerprint_path,
)

FP_DEFAULTS = {
    "install_prefix": "/opt/ros/humble",
    "package_suffix": "",
    "ros_distro": "humble",
    "colcon2deb_version": "0.4.1",
    "docker_image_id": "sha256:abc123",
}


class TestComputeFingerprintUsesPkgDir:
    def test_source_change_detected_when_dir_name_differs_from_pkg_name(
        self, tmp_path: Path
    ) -> None:
        """Package 'foo_bar' living in src/bar/ must still get real source hashes."""
        pkg_dir = tmp_path / "src" / "bar"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "main.cpp").write_text("int main() {}")
        overrides = tmp_path / "overrides"
        overrides.mkdir()

        fp1 = compute_fingerprint(
            pkg_name="foo_bar", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        (pkg_dir / "main.cpp").write_text("int main() { return 1; }")
        fp2 = compute_fingerprint(
            pkg_name="foo_bar", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_source_hash_is_not_empty_tree_constant(self, tmp_path: Path) -> None:
        """A package with sources must not hash as an empty/missing tree."""
        pkg_dir = tmp_path / "src" / "bar"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "main.cpp").write_text("int main() {}")
        overrides = tmp_path / "overrides"
        overrides.mkdir()

        empty = tmp_path / "empty"
        empty.mkdir()

        fp = compute_fingerprint(
            pkg_name="foo_bar", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        fp_empty = compute_fingerprint(
            pkg_name="foo_bar", pkg_dir=empty, overrides_dir=overrides, **FP_DEFAULTS
        )
        assert fp["source_hash"] != fp_empty["source_hash"]


class TestFingerprintPathPerStage:
    def test_stages_get_distinct_files(self, tmp_path: Path) -> None:
        """Phase 7 (debian generation) and phase 8 (deb build) must not share
        a fingerprint file, or phase 7's write makes phase 8 skip."""
        p_debian = fingerprint_path(tmp_path, "debian")
        p_deb = fingerprint_path(tmp_path, "deb")
        assert p_debian != p_deb
        assert p_debian.parent == tmp_path
        assert p_deb.parent == tmp_path
