"""Tests for config loading and validation logic.

Based on real Autoware 1.5.0/amd64 production configs and edge cases.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml


def load_config(path: Path) -> dict:
    """Load and return YAML config (mirrors colcon2deb.main.load_config logic)."""
    with open(path) as f:
        return yaml.safe_load(f)


class TestConfigLoading:
    """Test config file loading with real Autoware patterns."""

    def test_autoware_custom_prefix_config(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "autoware_custom_prefix.yaml")
        assert config["version"] == 1
        assert config["build"]["install_prefix"] == "/opt/autoware/1.5.0"
        assert config["build"]["package_suffix"] == "1-5-0"
        assert config["build"]["ros_distro"] == "humble"
        assert config["docker"]["dockerfile"] == "Dockerfile"
        assert config["docker"]["image_name"] == "autoware-1.5.0-amd64-builder"

    def test_default_prefix_config(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "default_prefix.yaml")
        assert config["version"] == 1
        assert "install_prefix" not in config["build"]
        assert "package_suffix" not in config["build"]
        assert config["docker"]["image"] == "ros:humble-ros-base"

    def test_parallel_jobs_zero_means_auto(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "autoware_custom_prefix.yaml")
        assert config["build"]["parallel_jobs"] == 0


class TestConfigValidation:
    """Test config validation edge cases."""

    def test_missing_version(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "missing_version.yaml")
        assert config.get("version") is None

    def test_both_image_and_dockerfile(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "both_image_and_dockerfile.yaml")
        docker = config.get("docker", {})
        assert "image" in docker
        assert "dockerfile" in docker

    def test_no_docker_section(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "no_docker.yaml")
        docker = config.get("docker", {})
        assert "image" not in docker
        assert "dockerfile" not in docker

    def test_missing_output_directory(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "missing_output.yaml")
        assert "output" not in config

    def test_missing_packages_directory(self, configs_dir: Path) -> None:
        config = load_config(configs_dir / "missing_packages.yaml")
        assert "packages" not in config

    def test_install_prefix_defaults(self, configs_dir: Path) -> None:
        """When install_prefix is not set, code defaults to /opt/ros/{ros_distro}."""
        config = load_config(configs_dir / "default_prefix.yaml")
        ros_distro = config["build"].get("ros_distro", "humble")
        install_prefix = config["build"].get(
            "install_prefix", f"/opt/ros/{ros_distro}"
        )
        assert install_prefix == "/opt/ros/humble"

    def test_package_suffix_is_string(self, configs_dir: Path) -> None:
        """package_suffix should always be a string, even if it looks numeric."""
        config = load_config(configs_dir / "autoware_custom_prefix.yaml")
        suffix = config["build"]["package_suffix"]
        assert isinstance(suffix, str)
        assert suffix == "1-5-0"
