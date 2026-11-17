"""Phase 9's link check and the .deb relocation, on real ELF files and real .debs.

The fixture reproduces the fault found in autoware-localrepo's 1.5.0 build (2026-11-17):
a library uses a class whose vtable lives in another library (diagnostic_updater 4.0.7's
Updater) without linking that library, so it carries an undefined vtable and fails to load
in a process that has not loaded the other library already.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HELPER_DIR = Path(__file__).parent.parent / "colcon2deb" / "helper"
if str(HELPER_DIR) not in sys.path:
    sys.path.insert(0, str(HELPER_DIR))

import link_check  # noqa: E402
import relocate_deb  # noqa: E402

pytestmark = pytest.mark.skipif(
    any(shutil.which(t) is None for t in ("g++", "dpkg-deb", "readelf", "ldd")),
    reason="needs g++, dpkg-deb, readelf and ldd",
)

PREFIX = "/opt/demo"
BUILD = "/output/workspace/install"

# As diagnostic_updater 4.0.7's Updater: constructor inline in the header (so a user
# references the vtable), the key function out of line (so the vtable lives in the library).
UPDATER_H = """
struct Updater {
  Updater() {}
  virtual ~Updater() {}
  virtual void tick();
};
"""
UPDATER_CPP = (
    UPDATER_H
    + """
void Updater::tick() {}
"""
)
USER_CPP = (
    UPDATER_H
    + """
extern "C" void make() { Updater u; u.tick(); }
"""
)


def _so(tmp: Path, name: str, src: str, *link: str) -> Path:
    c = tmp / f"{name}.cpp"
    c.write_text(src)
    out = tmp / f"lib{name}.so"
    subprocess.run(
        ["g++", "-shared", "-fPIC", "-o", str(out), str(c), f"-L{tmp}", *link],
        check=True,
        capture_output=True,
    )
    return out


def _deb(tmp: Path, name: str, files: dict[str, Path | str]) -> Path:
    """A .deb named ``name`` holding ``files`` ({path under / : source file or text})."""
    root = tmp / f"deb-{name}"
    (root / "DEBIAN").mkdir(parents=True)
    (root / "DEBIAN" / "control").write_text(
        f"Package: {name}\nVersion: 1.0\nArchitecture: all\nMaintainer: t <t@t>\n"
        "Description: test\n"
    )
    sums = []
    for path, src in files.items():
        dst = root / path.lstrip("/")
        dst.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(src, Path):
            shutil.copy(src, dst)
        else:
            dst.write_text(src)
        sums.append(f"{hashlib.md5(dst.read_bytes()).hexdigest()}  {path.lstrip('/')}\n")
    (root / "DEBIAN" / "md5sums").write_text("".join(sums))
    out = tmp / "debs" / f"{name}_1.0_all.deb"
    out.parent.mkdir(exist_ok=True)
    subprocess.run(
        ["dpkg-deb", "--root-owner-group", "-b", str(root), str(out)],
        check=True,
        capture_output=True,
    )
    return out


def _check(
    tmp: Path, mode: str = "error", allow: str | None = None, logs: Path | None = None
) -> tuple[int, dict]:
    reports = tmp / "reports"
    logs = logs or tmp / "logs"
    logs.mkdir(exist_ok=True)
    code = link_check.run(
        release_dir=tmp / "debs",
        log_packages_dir=logs,
        reports_dir=reports,
        markers=["/output/workspace", "/output/packaging"],
        ros_lib_dirs=[],
        mode=mode,
        allowlist=allow,
    )
    return code, json.loads((reports / "link_check.json").read_text())


@pytest.fixture()
def libs(tmp_path: Path) -> dict[str, Path]:
    base = _so(tmp_path, "updater", UPDATER_CPP)
    unlinked = _so(tmp_path, "user", USER_CPP)  # uses Updater, does not link it
    linked = _so(tmp_path, "user_ok", USER_CPP, "-lupdater")
    return {"base": base, "unlinked": unlinked, "linked": linked}


class TestLinkCheck:
    def test_unlinked_vtable_is_an_error(self, tmp_path: Path, libs: dict[str, Path]) -> None:
        _deb(tmp_path, "updater", {f"{PREFIX}/lib/libupdater.so": libs["base"]})
        _deb(tmp_path, "user", {f"{PREFIX}/lib/libuser.so": libs["unlinked"]})
        code, rep = _check(tmp_path)
        assert code == 1
        data = [f for f in rep["findings"] if f["kind"] == "undefined-data"]
        assert data and all(f["package"] == "user" for f in data)
        assert any("vtable for Updater" in f["demangled"] for f in data)
        assert rep["errors"] == len(data)

    def test_linked_library_passes(self, tmp_path: Path, libs: dict[str, Path]) -> None:
        _deb(tmp_path, "updater", {f"{PREFIX}/lib/libupdater.so": libs["base"]})
        _deb(tmp_path, "user", {f"{PREFIX}/lib/libuser_ok.so": libs["linked"]})
        code, rep = _check(tmp_path)
        assert code == 0
        assert rep["findings"] == []
        assert rep["elf_files"] == 2

    def test_warn_mode_reports_and_passes(self, tmp_path: Path, libs: dict[str, Path]) -> None:
        _deb(tmp_path, "user", {f"{PREFIX}/lib/libuser.so": libs["unlinked"]})
        code, rep = _check(tmp_path, mode="warn")
        assert code == 0
        assert rep["errors"] > 0
        assert (tmp_path / "reports" / "link_check.txt").read_text().startswith("link check (warn)")

    def test_allowlist_needs_a_reason_and_clears_a_match(
        self, tmp_path: Path, libs: dict[str, Path]
    ) -> None:
        _deb(tmp_path, "user", {f"{PREFIX}/lib/libuser.so": libs["unlinked"]})
        allow = tmp_path / "allow.yaml"
        allow.write_text("- package: user\n  symbol: Updater\n")
        with pytest.raises(SystemExit, match="reason"):
            _check(tmp_path, allow=str(allow))
        allow.write_text("- package: user\n  symbol: Updater\n  reason: provided by the host\n")
        code, rep = _check(tmp_path, allow=str(allow))
        assert code == 0
        assert all(f["allowed_by"] == "provided by the host" for f in rep["findings"])

    def test_build_path_under_share_is_an_error(self, tmp_path: Path) -> None:
        _deb(
            tmp_path,
            "demo",
            {
                f"{PREFIX}/share/demo/cmake/demoExport.cmake": f'INTERFACE_INCLUDE_DIRECTORIES "{BUILD}/dep/include"\n',
            },
        )
        code, rep = _check(tmp_path)
        assert code == 1
        (f,) = rep["findings"]
        assert f["kind"] == "build-path" and f["detail"] == f"{BUILD}/dep/include"

    def test_shlibdeps_warnings_are_classified(self, tmp_path: Path) -> None:
        (tmp_path / "debs").mkdir()
        log = tmp_path / "logs" / "demo" / "build_deb.log"
        log.parent.mkdir(parents=True)
        log.write_text(
            "dpkg-shlibdeps: warning: debian/x/opt/demo/lib/libdemo.so contains an "
            "unresolvable reference to symbol _ZTVN18diagnostic_updater7UpdaterE: it's "
            "probably a plugin\n"
            "dpkg-shlibdeps: warning: debian/x/opt/demo/lib/libother.so contains an "
            "unresolvable reference to symbol _ZN3fmt2v87vformatEv: it's probably a plugin\n"
        )
        code, rep = _check(tmp_path, logs=tmp_path / "logs")
        kinds = sorted(f["kind"] for f in rep["findings"])
        assert kinds == ["shlibdeps-data", "shlibdeps-function"]
        assert code == 1  # the vtable is data

    def test_is_data(self) -> None:
        assert link_check.is_data("_ZTVN3foo3BarE")
        assert link_check.is_data("_ZTI3Bar")
        assert link_check.is_data("errno_like", "TLS")
        assert link_check.is_data("some_table", "OBJECT")
        assert not link_check.is_data("_ZN3foo3barEv", "FUNC")
        assert not link_check.is_data("jpeg_read_header", "FUNC")


class TestRelocateDeb:
    def test_rewrites_cmake_pc_dsv_and_md5sums(self, tmp_path: Path, libs: dict[str, Path]) -> None:
        deb = _deb(
            tmp_path,
            "demo",
            {
                f"{PREFIX}/share/demo/cmake/demoExport.cmake": f'set(_dirs "{BUILD}/autoware_point_types/include;{BUILD}/x/include")\n',
                f"{PREFIX}/lib/pkgconfig/demo.pc": f"libdir={BUILD}/demo/lib\n",
                f"{PREFIX}/share/demo/hook/path.dsv": f"prepend-non-duplicate;PATH;{BUILD}/demo/bin\n",
                f"{PREFIX}/share/demo/notes.txt": f"{BUILD}/demo stays: not a search-path file\n",
                f"{PREFIX}/lib/libupdater.so": libs["base"],
            },
        )
        before = deb.read_bytes()
        changed = relocate_deb.relocate_deb(deb, BUILD, PREFIX)
        assert sorted(changed) == sorted(
            [
                f"{PREFIX.lstrip('/')}/share/demo/cmake/demoExport.cmake",
                f"{PREFIX.lstrip('/')}/lib/pkgconfig/demo.pc",
                f"{PREFIX.lstrip('/')}/share/demo/hook/path.dsv",
            ]
        )
        assert deb.read_bytes() != before
        out = tmp_path / "x"
        subprocess.run(["dpkg-deb", "-R", str(deb), str(out)], check=True)
        p = out / PREFIX.lstrip("/")
        assert (p / "share/demo/cmake/demoExport.cmake").read_text() == (
            f'set(_dirs "{PREFIX}/include;{PREFIX}/include")\n'
        )
        assert (p / "lib/pkgconfig/demo.pc").read_text() == f"libdir={PREFIX}/lib\n"
        assert BUILD in (p / "share/demo/notes.txt").read_text()
        for line in (out / "DEBIAN" / "md5sums").read_text().splitlines():
            digest, rel = line.split("  ", 1)
            assert hashlib.md5((out / rel).read_bytes()).hexdigest() == digest

    def test_unchanged_deb_is_left_alone(self, tmp_path: Path, libs: dict[str, Path]) -> None:
        deb = _deb(tmp_path, "clean", {f"{PREFIX}/lib/libupdater.so": libs["base"]})
        before = deb.read_bytes()
        assert relocate_deb.relocate_deb(deb, BUILD, PREFIX) == []
        assert deb.read_bytes() == before

    def test_relocated_deb_then_passes_the_build_path_check(self, tmp_path: Path) -> None:
        deb = _deb(
            tmp_path,
            "demo",
            {
                f"{PREFIX}/share/demo/cmake/demoExport.cmake": f'"{BUILD}/dep/include"\n',
            },
        )
        relocate_deb.relocate_deb(deb, BUILD, PREFIX)
        code, rep = _check(tmp_path)
        assert code == 0 and rep["findings"] == []
