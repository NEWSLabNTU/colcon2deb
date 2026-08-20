"""Tests for package fingerprinting module.

Tests that the fingerprint system correctly detects changes in source files,
debian overrides, and build configuration.
"""

from __future__ import annotations

from pathlib import Path

from colcon2deb.helper.fingerprint import (
    compute_fingerprint,
    fingerprint_matches,
    hash_tree,
    read_fingerprint,
    write_fingerprint,
)

FP_DEFAULTS = {
    "install_prefix": "/opt/ros/humble",
    "package_suffix": "",
    "ros_distro": "humble",
    "colcon2deb_version": "0.3.3",
    "docker_image_id": "sha256:abc123",
}


class TestHashTree:
    def test_empty_dir(self, tmp_path: Path) -> None:
        d = tmp_path / "empty"
        d.mkdir()
        h = hash_tree(d)
        assert isinstance(h, str)
        assert len(h) == 64  # SHA-256 hex

    def test_missing_dir(self, tmp_path: Path) -> None:
        h = hash_tree(tmp_path / "nonexistent")
        assert len(h) == 64

    def test_deterministic(self, tmp_path: Path) -> None:
        d = tmp_path / "pkg"
        d.mkdir()
        (d / "a.txt").write_text("hello")
        (d / "b.txt").write_text("world")
        assert hash_tree(d) == hash_tree(d)

    def test_detects_modification(self, tmp_path: Path) -> None:
        d = tmp_path / "pkg"
        d.mkdir()
        (d / "file.txt").write_text("original")
        h1 = hash_tree(d)
        (d / "file.txt").write_text("modified")
        h2 = hash_tree(d)
        assert h1 != h2

    def test_detects_addition(self, tmp_path: Path) -> None:
        d = tmp_path / "pkg"
        d.mkdir()
        (d / "a.txt").write_text("hello")
        h1 = hash_tree(d)
        (d / "b.txt").write_text("world")
        h2 = hash_tree(d)
        assert h1 != h2

    def test_detects_deletion(self, tmp_path: Path) -> None:
        d = tmp_path / "pkg"
        d.mkdir()
        (d / "a.txt").write_text("hello")
        (d / "b.txt").write_text("world")
        h1 = hash_tree(d)
        (d / "b.txt").unlink()
        h2 = hash_tree(d)
        assert h1 != h2

    def test_includes_subdirs(self, tmp_path: Path) -> None:
        d = tmp_path / "pkg"
        (d / "sub").mkdir(parents=True)
        (d / "sub" / "file.txt").write_text("nested")
        h1 = hash_tree(d)
        (d / "sub" / "file.txt").write_text("changed")
        h2 = hash_tree(d)
        assert h1 != h2


class TestComputeFingerprint:
    def _make_pkg(self, tmp_path: Path) -> tuple[Path, Path]:
        pkg_dir = tmp_path / "src" / "my_pkg"
        pkg_dir.mkdir(parents=True)
        (pkg_dir / "main.cpp").write_text("int main() {}")
        overrides = tmp_path / "overrides"
        overrides.mkdir()
        return pkg_dir, overrides

    def test_basic(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        assert "fingerprint" in fp
        assert len(fp["fingerprint"]) == 64

    def test_source_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        (pkg_dir / "main.cpp").write_text("int main() { return 1; }")
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_override_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        (overrides / "my_pkg").mkdir()
        (overrides / "my_pkg" / "control").write_text("new override")
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_install_prefix_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        modified = {**FP_DEFAULTS, "install_prefix": "/opt/autoware/1.5.0"}
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **modified
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_suffix_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        modified = {**FP_DEFAULTS, "package_suffix": "1-5-0"}
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **modified
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_docker_image_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        modified = {**FP_DEFAULTS, "docker_image_id": "sha256:different"}
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **modified
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]

    def test_version_change(self, tmp_path: Path) -> None:
        pkg_dir, overrides = self._make_pkg(tmp_path)
        fp1 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **FP_DEFAULTS
        )
        modified = {**FP_DEFAULTS, "colcon2deb_version": "0.4.0"}
        fp2 = compute_fingerprint(
            pkg_name="my_pkg", pkg_dir=pkg_dir, overrides_dir=overrides, **modified
        )
        assert fp1["fingerprint"] != fp2["fingerprint"]


class TestReadWriteFingerprint:
    def test_round_trip(self, tmp_path: Path) -> None:
        fp = {"fingerprint": "abc123", "source_hash": "def456"}
        path = tmp_path / "pkg" / ".fingerprint.json"
        write_fingerprint(path, fp)
        loaded = read_fingerprint(path)
        assert loaded == fp

    def test_read_missing(self, tmp_path: Path) -> None:
        assert read_fingerprint(tmp_path / "nonexistent.json") is None

    def test_read_invalid_json(self, tmp_path: Path) -> None:
        path = tmp_path / "bad.json"
        path.write_text("not json")
        assert read_fingerprint(path) is None

    def test_read_missing_key(self, tmp_path: Path) -> None:
        path = tmp_path / "missing.json"
        path.write_text('{"foo": "bar"}')
        assert read_fingerprint(path) is None


class TestFingerprintMatches:
    def test_matches(self) -> None:
        fp = {"fingerprint": "abc123"}
        assert fingerprint_matches(fp, fp) is True

    def test_mismatch(self) -> None:
        assert fingerprint_matches({"fingerprint": "a"}, {"fingerprint": "b"}) is False

    def test_none_stored(self) -> None:
        assert fingerprint_matches(None, {"fingerprint": "a"}) is False
