"""Tests for build output structure and report files.

Validates the output directory layout, report file formats, and deb naming
conventions based on real Autoware 1.5.0/amd64 builds (453 packages).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


# Deb naming pattern: ros-{distro}-{pkg-name}[-{suffix}]_{version}-{rev}_{arch}.deb
DEB_NAME_RE = re.compile(
    r"^ros-(?P<distro>[a-z]+)-(?P<name>[a-z0-9][-a-z0-9]*?)(?:-(?P<suffix>\d[-\d]+\d))?"
    r"_(?P<version>\d+\.\d+\.\d+)-(?P<revision>\d+[a-z]+)_(?P<arch>[a-z0-9]+)\.deb$"
)
DDEB_NAME_RE = re.compile(
    r"^ros-(?P<distro>[a-z]+)-(?P<name>[a-z0-9][-a-z0-9]*?)(?:-(?P<suffix>\d[-\d]+\d))?-dbgsym"
    r"_(?P<version>\d+\.\d+\.\d+)-(?P<revision>\d+[a-z]+)_(?P<arch>[a-z0-9]+)\.ddeb$"
)


class TestOutputDirectoryStructure:
    """Test the build output directory layout."""

    def test_output_directories_exist(self, tmp_output_dir: Path) -> None:
        assert (tmp_output_dir / "debs").is_dir()
        assert (tmp_output_dir / "packaging").is_dir()
        assert (tmp_output_dir / "workspace" / "src").is_dir()
        assert (tmp_output_dir / "logs").is_dir()

    def test_log_subdirectories(self, tmp_output_dir: Path) -> None:
        latest = tmp_output_dir / "logs" / "latest"
        assert latest.is_symlink()
        assert (latest / "phases").is_dir()
        assert (latest / "packages").is_dir()
        assert (latest / "reports").is_dir()
        assert (latest / "scripts").is_dir()

    def test_latest_symlink_resolves(self, tmp_output_dir: Path) -> None:
        latest = tmp_output_dir / "logs" / "latest"
        assert latest.resolve().name == "2026-03-28_02-09-16"


class TestReportFiles:
    """Test report file formats based on real Autoware output."""

    def test_successful_build_reports(self, successful_build_output: Path) -> None:
        reports = successful_build_output / "logs" / "latest" / "reports"
        successful = (reports / "successful.txt").read_text().strip().split("\n")
        failed = (reports / "failed.txt").read_text()
        skipped = (reports / "skipped.txt").read_text()

        assert len(successful) == 10
        assert failed == ""
        assert skipped == ""

    def test_packages_txt_format(self, successful_build_output: Path) -> None:
        """packages.txt uses ros-distro-pkg-name=version-revision format."""
        reports = successful_build_output / "logs" / "latest" / "reports"
        lines = (reports / "packages.txt").read_text().strip().split("\n")
        for line in lines:
            assert "=" in line, f"Missing '=' separator: {line}"
            name, version = line.split("=", 1)
            assert name.startswith("ros-humble-"), f"Bad prefix: {name}"
            assert "-0jammy" in version, f"Missing revision: {version}"

    def test_package_name_underscore_to_dash(
        self, successful_build_output: Path
    ) -> None:
        """Package names in packages.txt use dashes, not underscores."""
        reports = successful_build_output / "logs" / "latest" / "reports"
        lines = (reports / "packages.txt").read_text().strip().split("\n")
        for line in lines:
            name = line.split("=")[0]
            assert "_" not in name, f"Underscore in deb name: {name}"

    def test_successful_txt_uses_underscores(
        self, successful_build_output: Path
    ) -> None:
        """successful.txt uses original package names with underscores."""
        reports = successful_build_output / "logs" / "latest" / "reports"
        lines = (reports / "successful.txt").read_text().strip().split("\n")
        for line in lines:
            # Original ROS package names use underscores
            assert "-" not in line or line.startswith(
                "agnocast"
            ), f"Unexpected dash in package name: {line}"

    def test_partial_failure_reports(
        self, partial_failure_build_output: Path
    ) -> None:
        reports = partial_failure_build_output / "logs" / "latest" / "reports"
        successful = (reports / "successful.txt").read_text().strip().split("\n")
        failed = (reports / "failed.txt").read_text().strip().split("\n")
        skipped = (reports / "skipped.txt").read_text().strip().split("\n")

        assert len(successful) == 3
        assert len(failed) == 2
        assert len(skipped) == 1
        assert "autoware_ndt_scan_matcher" in failed
        assert "agnocast_e2e_test" in skipped

    def test_version_diversity(self, successful_build_output: Path) -> None:
        """Real Autoware builds have diverse upstream versions."""
        reports = successful_build_output / "logs" / "latest" / "reports"
        lines = (reports / "packages.txt").read_text().strip().split("\n")
        versions = {line.split("=")[1].split("-")[0] for line in lines}
        # Should have multiple different upstream versions
        assert len(versions) > 1


class TestDebNaming:
    """Test .deb file naming patterns from real Autoware builds."""

    def test_deb_files_exist(self, successful_build_output: Path) -> None:
        debs = list((successful_build_output / "debs").glob("*.deb"))
        assert len(debs) == 10

    def test_ddeb_files_exist(self, successful_build_output: Path) -> None:
        ddebs = list((successful_build_output / "debs").glob("*.ddeb"))
        assert len(ddebs) == 10

    def test_deb_naming_pattern(self, successful_build_output: Path) -> None:
        """All .deb files follow the ros-{distro}-{name}[-{suffix}]_{ver}-{rev}_{arch}.deb pattern."""
        for deb in (successful_build_output / "debs").glob("*.deb"):
            match = DEB_NAME_RE.match(deb.name)
            assert match is not None, f"Bad deb name: {deb.name}"
            assert match.group("distro") == "humble"
            assert match.group("suffix") == "1-5-0"
            assert match.group("arch") == "amd64"

    def test_ddeb_naming_pattern(self, successful_build_output: Path) -> None:
        """All .ddeb files follow the standard -dbgsym naming pattern."""
        for ddeb in (successful_build_output / "debs").glob("*.ddeb"):
            match = DDEB_NAME_RE.match(ddeb.name)
            assert match is not None, f"Bad ddeb name: {ddeb.name}"
            assert match.group("distro") == "humble"

    def test_deb_and_ddeb_pairs(self, successful_build_output: Path) -> None:
        """Every .deb should have a matching -dbgsym .ddeb."""
        debs_dir = successful_build_output / "debs"
        debs = {f.name for f in debs_dir.glob("*.deb")}
        ddebs = {f.name for f in debs_dir.glob("*.ddeb")}
        for deb_name in debs:
            # ros-humble-foo-1-5-0_1.0.0-0jammy_amd64.deb
            # -> ros-humble-foo-1-5-0-dbgsym_1.0.0-0jammy_amd64.ddeb
            expected_ddeb = deb_name.replace(".deb", ".ddeb").replace(
                "_", "-dbgsym_", 1
            )
            assert expected_ddeb in ddebs, f"Missing ddeb for {deb_name}"

    def test_long_package_names(self) -> None:
        """Real Autoware has extremely long package names (100+ chars in .deb name)."""
        # From real build: 102 chars
        long_name = "ros-humble-autoware-motion-velocity-boundary-departure-prevention-module-1-5-0_0.48.0-0jammy_amd64.deb"
        match = DEB_NAME_RE.match(long_name)
        assert match is not None
        assert match.group("name") == "autoware-motion-velocity-boundary-departure-prevention-module"

    def test_various_upstream_versions(self) -> None:
        """Real builds have diverse version numbers."""
        test_cases = [
            ("ros-humble-agnocast-e2e-test-1-5-0_2.1.2-0jammy_amd64.deb", "2.1.2"),
            ("ros-humble-autoware-adapi-specs-1-5-0_1.5.0-0jammy_amd64.deb", "1.5.0"),
            ("ros-humble-autoware-auto-common-1-5-0_0.48.0-0jammy_amd64.deb", "0.48.0"),
            ("ros-humble-autoware-adapi-v1-msgs-1-5-0_1.9.1-0jammy_amd64.deb", "1.9.1"),
        ]
        for deb_name, expected_version in test_cases:
            match = DEB_NAME_RE.match(deb_name)
            assert match is not None, f"Failed to parse: {deb_name}"
            assert match.group("version") == expected_version

    def test_no_suffix_deb_name(self) -> None:
        """Packages without a suffix (default /opt/ros/humble prefix)."""
        name = "ros-humble-rclcpp_21.0.5-0jammy_amd64.deb"
        match = DEB_NAME_RE.match(name)
        assert match is not None
        assert match.group("name") == "rclcpp"
        assert match.group("suffix") is None
