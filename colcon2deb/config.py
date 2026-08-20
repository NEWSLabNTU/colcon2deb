"""Config file loading and validation.

Turns the raw YAML mapping into a typed BuildConfig, rejecting malformed
input with actionable messages instead of tracebacks. All path values are
resolved relative to the config file's directory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, cast

import yaml

SUPPORTED_VERSION = 1

_TOP_LEVEL_KEYS = {"version", "docker", "output", "packages", "build"}
_DOCKER_KEYS = {"image", "dockerfile", "image_name", "build_context", "platform"}
_OUTPUT_KEYS = {"directory"}
_PACKAGES_KEYS = {"directory"}
_BUILD_KEYS = {
    "ros_distro",
    "install_prefix",
    "package_suffix",
    "parallel_jobs",
    "use_nvidia_runtime",
    "pipeline",
    "skip_tests",  # accepted but unused; warned about below
}


class ConfigError(Exception):
    """Raised when the config file is missing, unparsable, or invalid."""


@dataclass
class BuildConfig:
    """Validated configuration values."""

    image: str | None
    dockerfile: str | None
    image_name: str
    build_context: str | None
    platform: str | None
    output_dir: Path
    packages_dir: Path
    ros_distro: str
    install_prefix: str
    package_suffix: str | None
    parallel_jobs: int
    use_nvidia_runtime: bool
    pipeline: bool
    warnings: list[str] = field(default_factory=lambda: [])


def load_config_file(config_path: str | Path) -> Any:
    """Load raw YAML from a config file. Raises ConfigError on failure."""
    config_path = Path(config_path).resolve()
    if not config_path.exists():
        raise ConfigError(f"Config file not found at {config_path}")
    try:
        with open(config_path) as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ConfigError(f"Error parsing config file: {e}") from e


def _require_section(config: dict[str, Any], name: str) -> dict[str, Any]:
    section = config.get(name)
    if section is None:
        raise ConfigError(f"Missing required '{name}:' section in config")
    if not isinstance(section, dict):
        raise ConfigError(
            f"Config section '{name}:' must be a mapping, got {type(section).__name__}: {section!r}"
        )
    return cast("dict[str, Any]", section)


def _resolve_path(value: str, base_dir: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def validate_config(config: Any, config_dir: Path) -> BuildConfig:
    """Validate a raw config mapping and return a BuildConfig.

    config_dir is the directory of the config file; relative paths in the
    config are resolved against it.
    """
    if config is None:
        raise ConfigError("Config file is empty")
    if not isinstance(config, dict):
        raise ConfigError(f"Config must be a YAML mapping, got {type(config).__name__}")
    config = cast("dict[str, Any]", config)

    warnings: list[str] = []

    # Version
    version = config.get("version")
    if version is None:
        raise ConfigError("Missing required 'version' key (expected: version: 1)")
    if str(version) != str(SUPPORTED_VERSION):
        raise ConfigError(
            f"Unsupported config version: {version!r} (supported versions: {SUPPORTED_VERSION})"
        )

    # Docker section
    docker = _require_section(config, "docker")
    image = docker.get("image")
    dockerfile = docker.get("dockerfile")
    if image is not None and dockerfile is not None:
        raise ConfigError("Cannot specify both 'image' and 'dockerfile' in docker section")
    if image is None and dockerfile is None:
        raise ConfigError("Must specify either 'image' or 'dockerfile' in docker section")
    if image is not None and not isinstance(image, str):
        raise ConfigError(f"docker.image must be a string, got {image!r}")
    if dockerfile is not None and not isinstance(dockerfile, str):
        raise ConfigError(f"docker.dockerfile must be a string, got {dockerfile!r}")

    image_name = docker.get("image_name", "colcon2deb_builder")
    if not isinstance(image_name, str):
        raise ConfigError(f"docker.image_name must be a string, got {image_name!r}")

    build_context = docker.get("build_context")
    if build_context is not None and not isinstance(build_context, str):
        raise ConfigError(f"docker.build_context must be a string, got {build_context!r}")

    platform = docker.get("platform")
    if platform is not None and not isinstance(platform, str):
        raise ConfigError(f"docker.platform must be a string, got {platform!r}")

    # Output and packages sections
    output = _require_section(config, "output")
    if not isinstance(output.get("directory"), str):
        raise ConfigError("'output.directory' not specified in config")
    output_dir = _resolve_path(output["directory"], config_dir)

    packages = _require_section(config, "packages")
    if not isinstance(packages.get("directory"), str):
        raise ConfigError("'packages.directory' not specified in config")
    packages_dir = _resolve_path(packages["directory"], config_dir)

    # Build section (optional)
    build_raw = config.get("build")
    if build_raw is None:
        build_raw = {}
    if not isinstance(build_raw, dict):
        raise ConfigError(
            f"Config section 'build:' must be a mapping, got {type(build_raw).__name__}: {build_raw!r}"
        )
    build = cast("dict[str, Any]", build_raw)

    ros_distro = build.get("ros_distro", "humble")
    if not isinstance(ros_distro, str) or not ros_distro:
        raise ConfigError(f"build.ros_distro must be a non-empty string, got {ros_distro!r}")

    install_prefix = build.get("install_prefix", f"/opt/ros/{ros_distro}")
    if not isinstance(install_prefix, str) or not install_prefix.startswith("/"):
        raise ConfigError(f"build.install_prefix must be an absolute path, got {install_prefix!r}")

    package_suffix = build.get("package_suffix")
    if package_suffix is not None:
        package_suffix = str(package_suffix)

    parallel_jobs = build.get("parallel_jobs", 0)
    if isinstance(parallel_jobs, bool) or not isinstance(parallel_jobs, int):
        raise ConfigError(f"build.parallel_jobs must be an integer, got {parallel_jobs!r}")
    if parallel_jobs < 0:
        raise ConfigError(f"build.parallel_jobs must be >= 0, got {parallel_jobs}")

    use_nvidia_runtime = build.get("use_nvidia_runtime", False)
    if not isinstance(use_nvidia_runtime, bool):
        raise ConfigError(
            f"build.use_nvidia_runtime must be true or false, got {use_nvidia_runtime!r}"
        )

    # Pipelined build: run colcon build concurrently with debian generation
    # and gate each package's .deb build on colcon finishing that package.
    pipeline = build.get("pipeline", True)
    if not isinstance(pipeline, bool):
        raise ConfigError(f"build.pipeline must be true or false, got {pipeline!r}")

    # Warn about unknown keys — silently ignored settings mislead users
    for key in config:
        if key not in _TOP_LEVEL_KEYS:
            warnings.append(f"Unknown config key '{key}' is ignored")
    for section_name, section, known in [
        ("docker", docker, _DOCKER_KEYS),
        ("output", output, _OUTPUT_KEYS),
        ("packages", packages, _PACKAGES_KEYS),
        ("build", build, _BUILD_KEYS),
    ]:
        for key in section:
            if key not in known:
                warnings.append(f"Unknown config key '{section_name}.{key}' is ignored")
    if "skip_tests" in build:
        warnings.append("Config key 'build.skip_tests' is ignored (tests are never run)")

    return BuildConfig(
        image=image,
        dockerfile=dockerfile,
        image_name=image_name,
        build_context=build_context,
        platform=platform,
        output_dir=output_dir,
        packages_dir=packages_dir,
        ros_distro=ros_distro,
        install_prefix=install_prefix,
        package_suffix=package_suffix,
        parallel_jobs=parallel_jobs,
        use_nvidia_runtime=use_nvidia_runtime,
        pipeline=pipeline,
        warnings=warnings,
    )
