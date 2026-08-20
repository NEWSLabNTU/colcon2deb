"""Tests for container-side robustness fixes.

Covers:
- phase output tee must survive non-UTF-8 bytes (compiler diagnostics)
- workspace env capture must use NUL-separated env and fail loudly
- events.attach must append to an existing event file without clearing it
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HELPER_DIR = Path(__file__).parent.parent / "colcon2deb" / "helper"
if str(HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(HELPER_DIR))

from colcon2deb.helper import events as emitter  # noqa: E402
from colcon2deb.helper.main import capture_ros_env, run_script  # noqa: E402


class TestNonUtf8Output:
    def test_run_script_survives_invalid_utf8(self, tmp_path: Path) -> None:
        """A phase emitting raw non-UTF-8 bytes must not crash the orchestrator."""
        script = tmp_path / "binary_noise.sh"
        script.write_bytes(b'#!/bin/sh\nprintf "before \\377\\376 after\\n"\nexit 0\n')
        log = tmp_path / "phase.log"
        ok = run_script("binary_noise.sh", tmp_path, {"PATH": "/usr/bin:/bin"}, log_file=log)
        assert ok is True
        assert "before" in log.read_text()

    def test_run_script_failure_still_reported(self, tmp_path: Path) -> None:
        script = tmp_path / "fail.sh"
        script.write_text("#!/bin/sh\nexit 3\n")
        ok = run_script("fail.sh", tmp_path, {"PATH": "/usr/bin:/bin"}, log_file=tmp_path / "l")
        assert ok is False


class TestCaptureRosEnv:
    def test_missing_setup_bash_returns_none(self, tmp_path: Path) -> None:
        assert capture_ros_env(tmp_path / "install" / "setup.bash") is None

    def test_captures_exported_variables(self, tmp_path: Path) -> None:
        setup = tmp_path / "setup.bash"
        setup.write_text("export AMENT_PREFIX_PATH=/opt/test\nexport MY_VAR=hello\n")
        env = capture_ros_env(setup)
        assert env is not None
        assert env["AMENT_PREFIX_PATH"] == "/opt/test"
        assert env["MY_VAR"] == "hello"

    def test_multiline_values_do_not_corrupt_env(self, tmp_path: Path) -> None:
        """Exported bash functions produce multi-line env entries; naive
        line-splitting truncated them and injected garbage keys."""
        setup = tmp_path / "setup.bash"
        setup.write_text(
            "my_func() { echo 'a=b'; echo 'c=d'; }\n"
            "export -f my_func\n"
            "export NORMAL_VAR=value\n"
        )
        env = capture_ros_env(setup)
        assert env is not None
        assert env["NORMAL_VAR"] == "value"
        # Function body lines must not become environment keys
        assert "echo 'a" not in str(sorted(env.keys()))
        assert "c" not in env or env.get("c") != "d'; }"

    def test_failing_setup_returns_none(self, tmp_path: Path) -> None:
        setup = tmp_path / "setup.bash"
        setup.write_text("exit 1\n")
        assert capture_ros_env(setup) is None


class TestEventsAttach:
    def test_attach_appends_without_clearing(self, tmp_path: Path) -> None:
        """Subprocess scripts (build_deb.py) must join the event stream the
        orchestrator already opened, not truncate it."""
        emitter.init(tmp_path)
        emitter.phase_start(8, "Building Debian packages")

        emitter.attach(tmp_path)
        emitter.package_complete(8, "my_pkg", success=True)

        lines = (tmp_path / ".events.jsonl").read_text().splitlines()
        types = [json.loads(ln)["type"] for ln in lines if ln.strip()]
        assert types == ["phase_start", "package_complete"]
