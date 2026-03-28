"""Test that the remote Dockerfile configuration can be loaded and validated."""

from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SCRIPT_DIR / "configs" / "remote-dockerfile.yaml"


def test_config():
    """Test loading and validating the remote Dockerfile config."""
    assert CONFIG_PATH.exists(), f"Config file not found: {CONFIG_PATH}"

    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f)

    # Check Docker configuration has a remote URL
    docker_config = config.get("docker", {})
    assert "dockerfile" in docker_config, "No dockerfile in config"
    dockerfile_url = docker_config["dockerfile"]
    assert dockerfile_url.startswith(("http://", "https://")), (
        f"Dockerfile is not a URL: {dockerfile_url}"
    )

    # Check other required fields
    assert "output" in config and "directory" in config["output"], (
        "Output directory not configured"
    )
    assert "packages" in config and "directory" in config["packages"], (
        "Packages directory not configured"
    )
