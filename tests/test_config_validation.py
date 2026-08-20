"""Tests for config loading and validation (colcon2deb.config).

These exercise the real production validation — the previous
test_config_loading.py tested a private copy of the loader and let
empty-file/non-dict/typo configs crash main() with raw tracebacks.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon2deb.config import BuildConfig, ConfigError, load_config_file, validate_config

MINIMAL = {
    "version": 1,
    "docker": {"image": "my-image:latest"},
    "output": {"directory": "./build"},
    "packages": {"directory": "./overrides"},
}


def _validate(config: object, base: Path = Path("/base")) -> BuildConfig:
    return validate_config(config, base)


class TestLoadConfigFile:
    def test_missing_file(self, tmp_path: Path) -> None:
        with pytest.raises(ConfigError, match="not found"):
            load_config_file(tmp_path / "nope.yaml")

    def test_invalid_yaml(self, tmp_path: Path) -> None:
        f = tmp_path / "bad.yaml"
        f.write_text("docker: [unclosed")
        with pytest.raises(ConfigError):
            load_config_file(f)

    def test_valid_yaml(self, tmp_path: Path) -> None:
        f = tmp_path / "ok.yaml"
        f.write_text("version: 1\n")
        assert load_config_file(f) == {"version": 1}


class TestTopLevelShape:
    def test_empty_config_rejected_cleanly(self) -> None:
        """yaml.safe_load of an empty file returns None — must not traceback."""
        with pytest.raises(ConfigError):
            _validate(None)

    def test_non_mapping_config_rejected(self) -> None:
        with pytest.raises(ConfigError):
            _validate(["a", "b"])

    def test_minimal_valid(self) -> None:
        cfg = _validate(dict(MINIMAL))
        assert cfg.image == "my-image:latest"
        assert cfg.dockerfile is None


class TestVersion:
    def test_missing_version_names_expectation(self) -> None:
        c = {k: v for k, v in MINIMAL.items() if k != "version"}
        with pytest.raises(ConfigError, match="version: 1"):
            _validate(c)

    def test_string_version_accepted(self) -> None:
        """'version: \"1\"' must not be rejected with 'Unsupported version: 1'."""
        cfg = _validate({**MINIMAL, "version": "1"})
        assert isinstance(cfg, BuildConfig)

    def test_unsupported_version_says_supported(self) -> None:
        with pytest.raises(ConfigError, match="[Ss]upported"):
            _validate({**MINIMAL, "version": 2})


class TestDockerSection:
    def test_non_mapping_docker_rejected_cleanly(self) -> None:
        """'docker: my-image' used to TypeError via substring containment."""
        with pytest.raises(ConfigError, match="docker"):
            _validate({**MINIMAL, "docker": "my-image"})

    def test_both_image_and_dockerfile_rejected(self) -> None:
        with pytest.raises(ConfigError, match="both"):
            _validate({**MINIMAL, "docker": {"image": "a", "dockerfile": "b"}})

    def test_neither_image_nor_dockerfile_rejected(self) -> None:
        with pytest.raises(ConfigError):
            _validate({**MINIMAL, "docker": {}})

    def test_dockerfile_variant(self) -> None:
        cfg = _validate({**MINIMAL, "docker": {"dockerfile": "./Dockerfile"}})
        assert cfg.dockerfile == "./Dockerfile"
        assert cfg.image is None


class TestRequiredSections:
    def test_missing_output_directory(self) -> None:
        c = {**MINIMAL, "output": {}}
        with pytest.raises(ConfigError, match="output.directory"):
            _validate(c)

    def test_missing_packages_directory(self) -> None:
        c = {**MINIMAL, "packages": {}}
        with pytest.raises(ConfigError, match="packages.directory"):
            _validate(c)

    def test_non_mapping_output_rejected_cleanly(self) -> None:
        with pytest.raises(ConfigError, match="output"):
            _validate({**MINIMAL, "output": "./build"})

    def test_relative_paths_resolved_against_config_dir(self, tmp_path: Path) -> None:
        cfg = _validate(dict(MINIMAL), base=tmp_path)
        assert cfg.output_dir == tmp_path / "build"
        assert cfg.packages_dir == tmp_path / "overrides"

    def test_absolute_paths_kept(self, tmp_path: Path) -> None:
        c = {**MINIMAL, "output": {"directory": str(tmp_path / "out")}}
        cfg = _validate(c, base=Path("/elsewhere"))
        assert cfg.output_dir == tmp_path / "out"


class TestBuildSection:
    def test_defaults(self) -> None:
        cfg = _validate(dict(MINIMAL))
        assert cfg.ros_distro == "humble"
        assert cfg.install_prefix == "/opt/ros/humble"
        assert cfg.package_suffix is None
        assert cfg.parallel_jobs == 0
        assert cfg.use_nvidia_runtime is False

    def test_parallel_jobs_must_be_int(self) -> None:
        """'parallel_jobs: four' must fail at config time, not deep in the container."""
        c = {**MINIMAL, "build": {"parallel_jobs": "four"}}
        with pytest.raises(ConfigError, match="parallel_jobs"):
            _validate(c)

    def test_negative_parallel_jobs_rejected(self) -> None:
        c = {**MINIMAL, "build": {"parallel_jobs": -2}}
        with pytest.raises(ConfigError, match="parallel_jobs"):
            _validate(c)

    def test_install_prefix_must_be_absolute(self) -> None:
        c = {**MINIMAL, "build": {"install_prefix": "opt/ros"}}
        with pytest.raises(ConfigError, match="install_prefix"):
            _validate(c)

    def test_skip_tests_defaults_false(self) -> None:
        cfg = _validate(dict(MINIMAL))
        assert cfg.skip_tests is False

    def test_skip_tests_true_accepted_without_warning(self) -> None:
        """skip_tests is a real setting now (BUILD_TESTING=OFF + nocheck),
        not a dead key."""
        cfg = _validate({**MINIMAL, "build": {"skip_tests": True}})
        assert cfg.skip_tests is True
        assert cfg.warnings == []

    def test_skip_tests_must_be_bool(self) -> None:
        with pytest.raises(ConfigError, match="skip_tests"):
            _validate({**MINIMAL, "build": {"skip_tests": "yes"}})

    def test_pipeline_defaults_true(self) -> None:
        cfg = _validate(dict(MINIMAL))
        assert cfg.pipeline is True

    def test_pipeline_false_accepted(self) -> None:
        cfg = _validate({**MINIMAL, "build": {"pipeline": False}})
        assert cfg.pipeline is False

    def test_pipeline_must_be_bool(self) -> None:
        with pytest.raises(ConfigError, match="pipeline"):
            _validate({**MINIMAL, "build": {"pipeline": "yes"}})

    def test_values_pass_through(self) -> None:
        c = {
            **MINIMAL,
            "build": {
                "ros_distro": "jazzy",
                "install_prefix": "/opt/autoware",
                "package_suffix": "1.5.0",
                "parallel_jobs": 4,
                "use_nvidia_runtime": True,
            },
        }
        cfg = _validate(c)
        assert cfg.ros_distro == "jazzy"
        assert cfg.install_prefix == "/opt/autoware"
        assert cfg.package_suffix == "1.5.0"
        assert cfg.parallel_jobs == 4
        assert cfg.use_nvidia_runtime is True


class TestUnknownKeys:
    def test_unknown_top_level_keys_warn(self) -> None:
        """Silently ignored keys (e.g. workspace_dir from old docs) mislead users."""
        cfg = _validate({**MINIMAL, "workspace_dir": "./source"})
        assert any("workspace_dir" in w for w in cfg.warnings)

    def test_unknown_build_keys_warn(self) -> None:
        cfg = _validate({**MINIMAL, "build": {"made_up_key": True}})
        assert any("made_up_key" in w for w in cfg.warnings)

    def test_known_keys_do_not_warn(self) -> None:
        cfg = _validate(dict(MINIMAL))
        assert cfg.warnings == []


class TestDockerPrereqCheck:
    """check_docker_prereqs must catch missing binary and dead daemon
    before any expensive work."""

    def _with_fake_docker(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, exit_code: int) -> None:
        bin_dir = tmp_path / "bin"
        bin_dir.mkdir()
        fake = bin_dir / "docker"
        fake.write_text(f"#!/bin/sh\nexit {exit_code}\n")
        fake.chmod(0o755)
        monkeypatch.setenv("PATH", str(bin_dir))

    def test_missing_docker_binary(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        from colcon2deb.main import check_docker_prereqs

        monkeypatch.setenv("PATH", str(tmp_path))  # empty dir on PATH
        err = check_docker_prereqs()
        assert err is not None and "not found" in err

    def test_daemon_unreachable(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        from colcon2deb.main import check_docker_prereqs

        self._with_fake_docker(tmp_path, monkeypatch, exit_code=1)
        err = check_docker_prereqs()
        assert err is not None and "daemon" in err.lower()

    def test_docker_ok(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        from colcon2deb.main import check_docker_prereqs

        self._with_fake_docker(tmp_path, monkeypatch, exit_code=0)
        assert check_docker_prereqs() is None
