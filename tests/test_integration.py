"""Integration tests that build real ROS workspaces using Docker.

These tests exercise the full colcon2deb pipeline end-to-end:
  Host -> Docker container -> colcon build -> debian generation -> .deb output

Test cases are derived from real Autoware 1.5.0/amd64 production builds.

Requires: Docker daemon running, ros:humble image pullable.
Run with: just test-integ  (or just test-all)
"""

from __future__ import annotations

import json
import re
import subprocess
import tempfile
from pathlib import Path

import pytest

FIXTURES_DIR = Path(__file__).parent / "fixtures"
WORKSPACE_DIR = FIXTURES_DIR / "workspace"
CONFIGS_DIR = FIXTURES_DIR / "configs"

# Deb naming: ros-{distro}-{name}[-{suffix}]_{ver}-{rev}_{arch}.deb
DEB_RE = re.compile(
    r"^ros-(?P<distro>[a-z]+)-(?P<name>[a-z0-9][-a-z0-9]*?)"
    r"(?:-(?P<suffix>\d[-\d]+\d))?"
    r"_(?P<version>\d+\.\d+\.\d+)-(?P<revision>\d+[a-z]+)_(?P<arch>[a-z0-9]+)\.deb$"
)


def _docker_available() -> bool:
    """Check if Docker daemon is accessible."""
    try:
        r = subprocess.run(["docker", "info"], capture_output=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not _docker_available(), reason="Docker not available"),
]


def _prepare_config(config_name: str, output_dir: Path) -> Path:
    """Rewrite a fixture config to use absolute paths and the given output dir."""
    config_src = CONFIGS_DIR / config_name
    config_dst = output_dir / "config.yaml"

    config_text = config_src.read_text()
    config_text = config_text.replace(
        "dockerfile: ../Dockerfile",
        f"dockerfile: {FIXTURES_DIR / 'Dockerfile'}",
    )
    config_text = config_text.replace(
        "directory: ./build",
        f"directory: {output_dir / 'build'}",
    )
    config_text = config_text.replace(
        "directory: ./debian-overrides",
        f"directory: {output_dir / 'debian-overrides'}",
    )
    config_dst.write_text(config_text)
    (output_dir / "debian-overrides").mkdir(exist_ok=True)
    return config_dst


def _run_build(config_name: str, tmp_dir: Path | None = None) -> Path:
    """Run colcon2deb with a fixture config and return the build output dir.

    If tmp_dir is provided, reuse it (for multi-run fingerprint tests).
    Otherwise create a fresh temp dir.
    """
    tmp = tmp_dir or Path(tempfile.mkdtemp(prefix="colcon2deb-integ-"))
    config = _prepare_config(config_name, tmp)

    cmd = [
        "uv", "run", "colcon2deb",
        "--workspace", str(WORKSPACE_DIR),
        "--config", str(config),
    ]
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=600,
        cwd=Path(__file__).parent.parent,
    )
    if result.returncode != 0:
        print("STDOUT:", result.stdout[-3000:] if result.stdout else "")
        print("STDERR:", result.stderr[-2000:] if result.stderr else "")
        pytest.fail(f"colcon2deb failed with exit code {result.returncode}")
    return tmp / "build"


def _parse_build_counts(build_dir: Path) -> tuple[int, int]:
    """Parse successful and skipped counts from the latest reports.

    Returns (successful_count, skipped_count).
    """
    reports = build_dir / "logs" / "latest" / "reports"
    successful = (reports / "successful.txt").read_text().strip()
    skipped = (reports / "skipped.txt").read_text().strip()
    n_successful = len(successful.splitlines()) if successful else 0
    n_skipped = len(skipped.splitlines()) if skipped else 0
    return n_successful, n_skipped


# ---------------------------------------------------------------------------
# Fixtures: one build per test class (expensive, runs Docker)
# ---------------------------------------------------------------------------
@pytest.fixture(scope="class")
def default_build() -> Path:
    """Build with default /opt/ros/humble prefix, no suffix. Runs once per class."""
    return _run_build("integ_default_prefix.yaml")


@pytest.fixture(scope="class")
def custom_build() -> Path:
    """Build with /opt/testproject/1.0 prefix and 1-0-0 suffix. Runs once per class."""
    return _run_build("integ_custom_prefix.yaml")


# ---------------------------------------------------------------------------
# Test case 1: Default prefix (/opt/ros/humble), no suffix
# Mirrors the simplest real-world usage
# ---------------------------------------------------------------------------
class TestDefaultPrefixBuild:
    """Build with default /opt/ros/humble prefix, no package suffix."""

    def test_output_directory_structure(self, default_build: Path) -> None:
        assert (default_build / "debs").is_dir()
        assert (default_build / "packaging").is_dir()
        assert (default_build / "workspace").is_dir()
        assert (default_build / "logs").is_dir()

    def test_debs_produced(self, default_build: Path) -> None:
        """Both test_cpp_pkg and test_py_pkg should produce .deb files."""
        debs = list((default_build / "debs").glob("*.deb"))
        assert len(debs) == 2, f"Expected 2 .deb files, got: {[d.name for d in debs]}"

    def test_cpp_deb_naming(self, default_build: Path) -> None:
        debs = list((default_build / "debs").glob("ros-humble-test-cpp-pkg_*.deb"))
        assert len(debs) == 1, f"Expected 1 C++ .deb, got: {[d.name for d in debs]}"
        m = DEB_RE.match(debs[0].name)
        assert m is not None, f"Bad name: {debs[0].name}"
        assert m.group("distro") == "humble"
        assert m.group("name") == "test-cpp-pkg"
        assert m.group("version") == "1.0.0"
        assert m.group("suffix") is None

    def test_py_deb_naming(self, default_build: Path) -> None:
        debs = list((default_build / "debs").glob("ros-humble-test-py-pkg_*.deb"))
        assert len(debs) == 1
        m = DEB_RE.match(debs[0].name)
        assert m is not None, f"Bad name: {debs[0].name}"
        assert m.group("name") == "test-py-pkg"
        assert m.group("suffix") is None

    def test_deb_not_empty(self, default_build: Path) -> None:
        for deb in (default_build / "debs").glob("*.deb"):
            assert deb.stat().st_size > 1000, f"{deb.name} is suspiciously small"

    def test_log_reports(self, default_build: Path) -> None:
        latest = default_build / "logs" / "latest"
        assert latest.exists(), "logs/latest symlink missing"
        reports = latest / "reports"
        assert (reports / "successful.txt").exists()
        assert (reports / "failed.txt").exists()
        assert (reports / "packages.txt").exists()

    def test_log_directory_structure(self, default_build: Path) -> None:
        """New log layout: phases/, packages/, reports/, scripts/."""
        latest = default_build / "logs" / "latest"
        assert (latest / "phases").is_dir()
        assert (latest / "packages").is_dir()
        assert (latest / "reports").is_dir()
        assert (latest / "scripts").is_dir()

    def test_phase_logs_exist(self, default_build: Path) -> None:
        """Every phase should have a log file with a predictable name (no dates)."""
        phases = default_build / "logs" / "latest" / "phases"
        # Check the 8 main phase logs exist (there may be extra detail logs like rosdep_simulate)
        expected = [
            "phase1_prepare.log",
            "phase2_copy_src.log",
            "phase3_install_deps.log",
            "phase4_build_src.log",
            "phase5_create_rosdep_list.log",
            "phase6_create_package_list.log",
            "phase7_generate_debian_dir.log",
            "phase8_build_deb.log",
        ]
        for name in expected:
            assert (phases / name).exists(), f"Missing phase log: {name}"

    def test_per_package_logs(self, default_build: Path) -> None:
        """Per-package logs should be in packages/ with predictable names."""
        packages = default_build / "logs" / "latest" / "packages"
        for pkg_name in ["test_cpp_pkg", "test_py_pkg"]:
            pkg_log_dir = packages / pkg_name
            assert pkg_log_dir.is_dir(), f"Missing package log dir: {pkg_name}"
            gen_log = pkg_log_dir / "generate_debian.log"
            build_log = pkg_log_dir / "build_deb.log"
            assert gen_log.exists(), f"Missing {pkg_name}/generate_debian.log"
            assert build_log.exists(), f"Missing {pkg_name}/build_deb.log"

    def test_all_packages_successful(self, default_build: Path) -> None:
        reports = default_build / "logs" / "latest" / "reports"
        successful = (reports / "successful.txt").read_text().strip().split("\n")
        failed = (reports / "failed.txt").read_text().strip()
        assert "test_cpp_pkg" in successful
        assert "test_py_pkg" in successful
        assert failed == ""

    def test_packaging_dirs(self, default_build: Path) -> None:
        pkg = default_build / "packaging"
        assert (pkg / "test_cpp_pkg" / "debian").is_dir()
        assert (pkg / "test_py_pkg" / "debian").is_dir()

    def test_debian_control_no_suffix(self, default_build: Path) -> None:
        control = (
            default_build / "packaging" / "test_cpp_pkg" / "debian" / "control"
        ).read_text()
        assert "Source: ros-humble-test-cpp-pkg" in control
        assert "Package: ros-humble-test-cpp-pkg" in control
        assert "ros-humble-test-cpp-pkg-1-0-0" not in control

    def test_debian_rules_default_prefix(self, default_build: Path) -> None:
        rules = (
            default_build / "packaging" / "test_cpp_pkg" / "debian" / "rules"
        ).read_text()
        assert "/opt/ros/humble" in rules

    def test_events_lifecycle(self, default_build: Path) -> None:
        events_file = default_build / ".events.jsonl"
        assert events_file.exists()
        events = [json.loads(line) for line in events_file.read_text().strip().split("\n")]
        types = [e["type"] for e in events]
        assert "build_start" in types
        assert "build_complete" in types
        assert events[-1]["success"] is True


# ---------------------------------------------------------------------------
# Test case 2: Custom prefix + suffix
# Mirrors Autoware 1.5.0: install_prefix=/opt/autoware/1.5.0, suffix=1-5-0
# ---------------------------------------------------------------------------
class TestCustomPrefixBuild:
    """Build with custom prefix /opt/testproject/1.0 and suffix 1-0-0."""

    def test_debs_produced(self, custom_build: Path) -> None:
        debs = list((custom_build / "debs").glob("*.deb"))
        assert len(debs) == 2

    def test_cpp_deb_has_suffix(self, custom_build: Path) -> None:
        debs = list((custom_build / "debs").glob("ros-humble-test-cpp-pkg-1-0-0_*.deb"))
        assert len(debs) == 1, (
            f"Expected suffixed .deb, got: {[d.name for d in (custom_build / 'debs').glob('*.deb')]}"
        )
        m = DEB_RE.match(debs[0].name)
        assert m is not None
        assert m.group("suffix") == "1-0-0"

    def test_py_deb_has_suffix(self, custom_build: Path) -> None:
        debs = list((custom_build / "debs").glob("ros-humble-test-py-pkg-1-0-0_*.deb"))
        assert len(debs) == 1
        m = DEB_RE.match(debs[0].name)
        assert m is not None
        assert m.group("suffix") == "1-0-0"

    def test_debian_control_has_suffix(self, custom_build: Path) -> None:
        control = (
            custom_build / "packaging" / "test_cpp_pkg" / "debian" / "control"
        ).read_text()
        assert "Source: ros-humble-test-cpp-pkg-1-0-0" in control
        assert "Package: ros-humble-test-cpp-pkg-1-0-0" in control

    def test_debian_rules_custom_prefix(self, custom_build: Path) -> None:
        rules = (
            custom_build / "packaging" / "test_cpp_pkg" / "debian" / "rules"
        ).read_text()
        assert "/opt/testproject/1.0" in rules
        assert 'CMAKE_INSTALL_PREFIX="/opt/ros/humble"' not in rules

    def test_py_rules_custom_prefix(self, custom_build: Path) -> None:
        rules = (
            custom_build / "packaging" / "test_py_pkg" / "debian" / "rules"
        ).read_text()
        assert "/opt/testproject/1.0" in rules

    def test_all_packages_successful(self, custom_build: Path) -> None:
        reports = custom_build / "logs" / "latest" / "reports"
        successful = (reports / "successful.txt").read_text().strip().split("\n")
        failed = (reports / "failed.txt").read_text().strip()
        assert len(successful) == 2
        assert failed == ""


# ---------------------------------------------------------------------------
# Test case 3: Fingerprint-based rebuild detection
# Verifies Cargo-like behaviour: skip if inputs unchanged, rebuild if changed
# ---------------------------------------------------------------------------
@pytest.fixture(scope="class")
def fingerprint_env() -> Path:
    """Persistent temp dir shared across all fingerprint tests (one class)."""
    return Path(tempfile.mkdtemp(prefix="colcon2deb-fp-"))


class TestFingerprinting:
    """Fingerprint-based rebuild detection across multiple builds."""

    def test_01_first_build(self, fingerprint_env: Path) -> None:
        """First build: both packages should be built (not cached)."""
        build_dir = _run_build("integ_default_prefix.yaml", fingerprint_env)
        successful, skipped = _parse_build_counts(build_dir)
        assert successful == 2, f"Expected 2 built, got {successful} built + {skipped} cached"
        assert skipped == 0

    def test_02_fingerprints_written(self, fingerprint_env: Path) -> None:
        """After first build, each package should have a .fingerprint.json."""
        packaging = fingerprint_env / "build" / "packaging"
        for pkg in ["test_cpp_pkg", "test_py_pkg"]:
            fp_file = packaging / pkg / ".fingerprint.json"
            assert fp_file.exists(), f"Missing fingerprint for {pkg}"
            data = json.loads(fp_file.read_text())
            assert "fingerprint" in data
            assert "source_hash" in data
            assert "docker_image_id" in data

    def test_03_second_build_all_cached(self, fingerprint_env: Path) -> None:
        """Second build with no changes: all packages should be cached."""
        build_dir = _run_build("integ_default_prefix.yaml", fingerprint_env)
        successful, skipped = _parse_build_counts(build_dir)
        assert successful == 0, f"Expected 0 built, got {successful}"
        assert skipped == 2, f"Expected 2 cached, got {skipped}"

    def test_04_source_change_triggers_rebuild(self, fingerprint_env: Path) -> None:
        """Modifying source code should trigger rebuild of that package."""
        # Modify a source file in the workspace copy (inside the build dir)
        workspace_src = fingerprint_env / "build" / "workspace" / "src"
        cpp_main = workspace_src / "test_cpp_pkg" / "src" / "test_node.cpp"
        if cpp_main.exists():
            cpp_main.write_text(
                cpp_main.read_text() + "\n// fingerprint test modification\n"
            )

        # Delete the .deb so it can be rebuilt
        debs = fingerprint_env / "build" / "debs"
        for deb in debs.glob("ros-humble-test-cpp-pkg_*"):
            deb.unlink()

        build_dir = _run_build("integ_default_prefix.yaml", fingerprint_env)
        successful, skipped = _parse_build_counts(build_dir)
        # test_cpp_pkg rebuilt, test_py_pkg cached
        assert successful >= 1, f"Expected at least 1 rebuilt, got {successful}"

    def test_05_override_change_triggers_rebuild(self, fingerprint_env: Path) -> None:
        """Modifying a debian-override should trigger rebuild."""
        # Create a debian-override for test_py_pkg
        override_dir = fingerprint_env / "debian-overrides" / "test_py_pkg"
        override_dir.mkdir(parents=True, exist_ok=True)
        (override_dir / "marker.txt").write_text("trigger rebuild\n")

        # Delete the .deb so it can be rebuilt
        debs = fingerprint_env / "build" / "debs"
        for deb in debs.glob("ros-humble-test-py-pkg_*"):
            deb.unlink()

        build_dir = _run_build("integ_default_prefix.yaml", fingerprint_env)
        successful, skipped = _parse_build_counts(build_dir)
        assert successful >= 1, f"Expected at least 1 rebuilt, got {successful}"
