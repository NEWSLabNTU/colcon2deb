"""Tests for container-side robustness fixes.

Covers:
- phase output tee must survive non-UTF-8 bytes (compiler diagnostics)
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
from colcon2deb.helper.main import run_script  # noqa: E402


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


class TestColconTreeImageStamp:
    """Phase 4 must not reuse a colcon tree built under another image."""

    BUILD_SRC = HELPER_DIR / "build-src.sh"

    def _run(self, ws: Path, image_id: str) -> str:
        import subprocess

        bindir = ws.parent / "bin"
        bindir.mkdir(exist_ok=True)
        # Stand-in colcon: record that it ran and leave a tree behind.
        fake = bindir / "colcon"
        fake.write_text("#!/bin/sh\nmkdir -p build install log\ntouch build/ran\nexit 0\n")
        fake.chmod(0o755)
        result = subprocess.run(
            ["bash", str(self.BUILD_SRC)],
            env={
                "PATH": f"{bindir}:/usr/bin:/bin",
                "colcon_work_dir": str(ws),
                "colcon_build": "y",
                "COLCON2DEB_IMAGE_ID": image_id,
            },
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout

    def test_same_image_keeps_tree(self, tmp_path: Path) -> None:
        ws = tmp_path / "ws"
        ws.mkdir()
        self._run(ws, "sha256:aaa")
        (ws / "install" / "keep").write_text("x")
        out = self._run(ws, "sha256:aaa")
        assert "discarding" not in out
        assert (ws / "install" / "keep").exists()

    def test_changed_image_discards_tree(self, tmp_path: Path) -> None:
        ws = tmp_path / "ws"
        ws.mkdir()
        self._run(ws, "sha256:aaa")
        (ws / "install" / "stale").write_text("x")
        out = self._run(ws, "sha256:bbb")
        assert "discarding" in out
        assert not (ws / "install" / "stale").exists()
        assert (ws / ".colcon2deb-image-id").read_text().strip() == "sha256:bbb"

    def test_tree_without_stamp_is_discarded(self, tmp_path: Path) -> None:
        """A tree from a colcon2deb that predates the stamp is of unknown origin."""
        ws = tmp_path / "ws"
        (ws / "install").mkdir(parents=True)
        (ws / "install" / "old").write_text("x")
        out = self._run(ws, "sha256:aaa")
        assert "discarding" in out
        assert not (ws / "install" / "old").exists()
