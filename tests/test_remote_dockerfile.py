"""Test remote Dockerfile download functionality."""

import urllib.request
from pathlib import Path

import pytest

from colcon2deb.main import download_dockerfile


def _network_available() -> bool:
    """Check if we can reach GitHub."""
    try:
        urllib.request.urlopen("https://github.com", timeout=5)
        return True
    except Exception:
        return False


@pytest.mark.network
@pytest.mark.timeout(30)
def test_download(tmp_path: Path) -> None:
    """Test downloading a Dockerfile from a URL."""
    if not _network_available():
        pytest.skip("Network unavailable")
    test_url = "https://raw.githubusercontent.com/NEWSLabNTU/autoware-build-images/refs/heads/main/0.45.1/amd64/Dockerfile"

    # Test without cache
    result = download_dockerfile(test_url, cache_dir=None)
    assert result.exists()
    assert result.stat().st_size > 0
    content = result.read_text()
    assert "FROM" in content

    # Test with cache
    cache_dir = tmp_path / "dockerfile_cache"
    result1 = download_dockerfile(test_url, cache_dir=cache_dir)
    assert result1.exists()

    # Test cache hit
    result2 = download_dockerfile(test_url, cache_dir=cache_dir)
    assert result1 == result2
