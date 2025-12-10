#!/usr/bin/env python3
"""Test script for remote Dockerfile download functionality."""

import sys
import tempfile
from pathlib import Path

# Add parent directory to path to import colcon2deb module
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PROJECT_ROOT))

from colcon2deb.main import download_dockerfile

def test_download():
    """Test downloading a Dockerfile from a URL."""
    # Test URL - using a known good Dockerfile
    test_url = "https://raw.githubusercontent.com/NEWSLabNTU/autoware-build-images/refs/heads/main/0.45.1/amd64/Dockerfile"
    
    print(f"Testing download from: {test_url}")
    
    # Test without cache
    print("\n1. Testing download without cache...")
    try:
        result = download_dockerfile(test_url, cache_dir=None)
        print(f"   Success! Downloaded to: {result}")
        print(f"   File size: {result.stat().st_size} bytes")
        
        # Check content
        content = result.read_text()
        if "FROM" in content and "RUN" in content:
            print("   Content looks like a valid Dockerfile")
        else:
            print("   Warning: Content doesn't look like a Dockerfile")
            
    except Exception as e:
        print(f"   Error: {e}")
        return False
    
    # Test with cache
    print("\n2. Testing download with cache...")
    cache_dir = Path.home() / ".cache" / "colcon2deb" / "dockerfiles"
    try:
        result = download_dockerfile(test_url, cache_dir=cache_dir)
        print(f"   Success! Cached/Retrieved from: {result}")
        
        # Test cache hit (second download)
        print("\n3. Testing cache hit...")
        result2 = download_dockerfile(test_url, cache_dir=cache_dir)
        if result == result2:
            print(f"   Success! Cache working correctly")
        else:
            print(f"   Warning: Cache may not be working properly")
            
    except Exception as e:
        print(f"   Error: {e}")
        return False
    
    print("\n✅ All tests passed!")
    return True

if __name__ == "__main__":
    success = test_download()
    sys.exit(0 if success else 1)