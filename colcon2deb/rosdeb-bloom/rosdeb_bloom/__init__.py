try:
    from importlib.metadata import PackageNotFoundError, version
    try:
        __version__ = version("rosdeb-bloom")
    except PackageNotFoundError:
        __version__ = 'unset'
except ImportError:
    __version__ = 'unset'

# Library API exports
from rosdeb_bloom.api import GenerateResult, copy_debian_to_dest, generate_debian

__all__ = ['generate_debian', 'GenerateResult', 'copy_debian_to_dest', '__version__']
