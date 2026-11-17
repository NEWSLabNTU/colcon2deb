#!/usr/bin/env python3
"""Phase 9: check that the built packages link and carry no build paths.

Every .deb in the release directory is unpacked into one merged tree (as the packages are
installed together on a target machine), then:

1. Every ELF file is checked with ``ldd -r`` against the merged tree's library directories
   and the ROS installation. An undefined *data* symbol (an object or TLS symbol, or a
   C++ vtable, typeinfo or guard variable) fails at ``dlopen`` and is an error; an undefined
   function fails only when called and is a warning, unless an allowlist entry names it.
2. ``dpkg-shlibdeps`` warnings in the per-package build logs ("contains an unresolvable
   reference to symbol") are reported the same way (data symbols by their mangled prefix).
3. Every text file under ``share/`` is searched for the build's own directories (the colcon
   workspace, the packaging directory): such a path does not exist on a target machine.

Mode (``COLCON2DEB_LINK_CHECK``): ``error`` fails the phase on any error, ``warn`` reports
and passes, ``off`` skips. Allowlist (``COLCON2DEB_LINK_CHECK_ALLOW``): a YAML list of
``{package: <glob>, symbol: <regex>, reason: <text>}``; ``package`` matches the ROS package
name or the .deb's name, ``symbol`` the demangled or the raw symbol, and ``reason`` is
required.

Reports: ``reports/link_check.txt`` and ``reports/link_check.json`` in the log directory.
"""

from __future__ import annotations

import fnmatch
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import cast

DATA_PREFIXES = ("_ZTV", "_ZTI", "_ZTS", "_ZTT", "_ZGV", "_ZTH", "_ZTW")
UNRESOLVABLE = re.compile(
    r"dpkg-shlibdeps: warning: \S*?/([^/\s]+) contains an unresolvable reference to symbol (\S+?):"
)


@dataclass
class Finding:
    package: str  # the .deb's name, or the ROS package for a log finding
    file: str  # path inside the install, or the log
    kind: str  # undefined-data, undefined-function, shlibdeps-data, shlibdeps-function, build-path
    detail: str  # the symbol (raw), or the build path found
    demangled: str = ""
    allowed_by: str = ""  # the allowlist entry's reason, when one matched

    @property
    def is_error(self) -> bool:
        if self.allowed_by:
            return False
        return self.kind in ("undefined-data", "shlibdeps-data", "build-path")


@dataclass
class Report:
    mode: str
    debs: int = 0
    elf_files: int = 0
    findings: list[Finding] = field(default_factory=lambda: list[Finding]())

    @property
    def errors(self) -> list[Finding]:
        return [f for f in self.findings if f.is_error]


# --- symbols -------------------------------------------------------------------------------


def demangle(symbols: list[str]) -> dict[str, str]:
    """``{raw: demangled}`` through c++filt (raw itself when c++filt is missing)."""
    if not symbols:
        return {}
    try:
        out = subprocess.run(
            ["c++filt"], input="\n".join(symbols), capture_output=True, text=True, check=True
        ).stdout.splitlines()
    except (OSError, subprocess.CalledProcessError):
        out = symbols
    return dict(zip(symbols, out, strict=False))


def undefined_types(elf: Path) -> dict[str, str]:
    """``{symbol: type}`` of the undefined dynamic symbols (FUNC, OBJECT, TLS, NOTYPE)."""
    out = subprocess.run(
        ["readelf", "-W", "--dyn-syms", str(elf)], capture_output=True, text=True
    ).stdout
    types: dict[str, str] = {}
    for line in out.splitlines():
        parts = line.split()
        # Num: Value Size Type Bind Vis Ndx Name
        if len(parts) >= 8 and parts[6] == "UND":
            types[parts[7].split("@")[0]] = parts[3]
    return types


def is_data(symbol: str, sym_type: str = "") -> bool:
    """An undefined symbol ``dlopen`` must resolve at once."""
    return sym_type in ("OBJECT", "TLS") or symbol.startswith(DATA_PREFIXES)


def ldd_undefined(elf: Path, env: dict[str, str]) -> list[str]:
    """The symbols ``ldd -r`` reports undefined for ``elf``."""
    out = subprocess.run(["ldd", "-r", str(elf)], capture_output=True, text=True, env=env)
    syms: list[str] = []
    for line in (out.stdout + out.stderr).splitlines():
        line = line.strip()
        if line.startswith("undefined symbol:"):
            syms.append(line.split(":", 1)[1].split()[0].rstrip(","))
    return syms


def is_elf(path: Path) -> bool:
    try:
        with open(path, "rb") as fh:
            return fh.read(4) == b"\x7fELF"
    except OSError:
        return False


# --- the merged tree -------------------------------------------------------------------------


def unpack(debs: list[Path], root: Path) -> dict[str, str]:
    """Unpack every deb into ``root``; returns ``{path in tree: deb package name}``."""
    owner: dict[str, str] = {}
    for deb in debs:
        name = deb.name.split("_")[0]
        subprocess.run(["dpkg-deb", "-x", str(deb), str(root)], check=True)
        listing = subprocess.run(
            ["dpkg-deb", "-c", str(deb)], capture_output=True, text=True, check=True
        ).stdout
        for line in listing.splitlines():
            path = line.split(None, 5)[-1].split(" -> ")[0].lstrip(".")
            owner[path] = name
    return owner


def library_dirs(root: Path) -> list[str]:
    """Every directory under ``root`` that holds a shared library."""
    dirs = {p.parent for p in root.rglob("*.so*") if p.is_file() or p.is_symlink()}
    return sorted(str(d) for d in dirs)


# --- checks ---------------------------------------------------------------------------------


def check_elves(root: Path, owner: dict[str, str], env: dict[str, str], report: Report) -> None:
    for f in sorted(root.rglob("*")):
        if f.is_symlink() or not f.is_file() or not is_elf(f):
            continue
        report.elf_files += 1
        missing = ldd_undefined(f, env)
        if not missing:
            continue
        types = undefined_types(f)
        names = demangle(missing)
        rel = "/" + f.relative_to(root).as_posix()
        pkg = owner.get(rel, "?")
        for sym in missing:
            kind = "undefined-data" if is_data(sym, types.get(sym, "")) else "undefined-function"
            report.findings.append(Finding(pkg, rel, kind, sym, names.get(sym, sym)))


def check_shlibdeps_logs(log_packages_dir: Path, report: Report) -> None:
    for log in sorted(log_packages_dir.glob("*/build_deb.log")):
        try:
            text = log.read_text(errors="replace")
        except OSError:
            continue
        hits = UNRESOLVABLE.findall(text)
        if not hits:
            continue
        names = demangle([s for _, s in hits])
        for lib, sym in hits:
            kind = "shlibdeps-data" if is_data(sym) else "shlibdeps-function"
            report.findings.append(
                Finding(log.parent.name, f"{lib} ({log})", kind, sym, names.get(sym) or sym)
            )


def check_build_paths(
    root: Path, owner: dict[str, str], markers: list[str], report: Report
) -> None:
    markers = [m.rstrip("/") + "/" for m in markers if m]
    for f in sorted((root).rglob("*")):
        if "share" not in f.relative_to(root).parts or f.is_symlink() or not f.is_file():
            continue
        try:
            data = f.read_bytes()
        except OSError:
            continue
        if b"\0" in data[:8192]:
            continue
        text = data.decode(errors="replace")
        for m in markers:
            i = text.find(m)
            if i >= 0:
                end = re.search(r"[\s;\"')]", text[i:])
                found = text[i : i + (end.start() if end else 80)]
                rel = "/" + f.relative_to(root).as_posix()
                report.findings.append(Finding(owner.get(rel, "?"), rel, "build-path", found))
                break


# --- allowlist ------------------------------------------------------------------------------


def load_allowlist(path: str | None) -> list[dict[str, str]]:
    if not path or not Path(path).is_file():
        return []
    import yaml

    entries: object = yaml.safe_load(Path(path).read_text()) or []
    if not isinstance(entries, list):
        raise SystemExit(f"link check allowlist {path}: must be a list of entries")
    checked: list[dict[str, str]] = []
    for i, e in enumerate(cast(list[object], entries)):
        if not isinstance(e, dict):
            raise SystemExit(f"link check allowlist {path}: entry {i + 1} is not a mapping")
        entry = {str(k): str(v) for k, v in cast(dict[object, object], e).items()}
        if not entry.get("symbol") or not entry.get("reason"):
            raise SystemExit(
                f"link check allowlist {path}: entry {i + 1} needs `symbol` and `reason`"
            )
        checked.append(entry)
    return checked


def apply_allowlist(report: Report, entries: list[dict[str, str]]) -> None:
    for f in report.findings:
        if f.kind == "build-path":
            continue
        ros_pkg = f.package
        for e in entries:
            pkg_glob = e.get("package", "*")
            if not (fnmatch.fnmatch(f.package, pkg_glob) or fnmatch.fnmatch(ros_pkg, pkg_glob)):
                continue
            pat = re.compile(e["symbol"])
            if pat.search(f.detail) or pat.search(f.demangled):
                f.allowed_by = e["reason"]
                break


# --- report ---------------------------------------------------------------------------------


def write_reports(report: Report, reports_dir: Path) -> str:
    lines = [
        f"link check ({report.mode}): {report.debs} package(s), {report.elf_files} ELF file(s), "
        f"{len(report.errors)} error(s), "
        f"{len(report.findings) - len(report.errors)} warning(s) or allowed",
    ]
    by_kind: dict[str, list[Finding]] = {}
    for f in report.findings:
        by_kind.setdefault("allowed" if f.allowed_by else f.kind, []).append(f)
    order = [
        "undefined-data",
        "shlibdeps-data",
        "build-path",
        "undefined-function",
        "shlibdeps-function",
        "allowed",
    ]
    for kind in order:
        items = by_kind.get(kind, [])
        if not items:
            continue
        lines.append(f"\n{kind} ({len(items)}):")
        for f in items:
            extra = f"  [{f.allowed_by}]" if f.allowed_by else ""
            lines.append(f"  {f.package}: {f.file}: {f.demangled or f.detail}{extra}")
    text = "\n".join(lines) + "\n"
    reports_dir.mkdir(parents=True, exist_ok=True)
    (reports_dir / "link_check.txt").write_text(text)
    (reports_dir / "link_check.json").write_text(
        json.dumps(
            {
                "mode": report.mode,
                "debs": report.debs,
                "elf_files": report.elf_files,
                "errors": len(report.errors),
                "findings": [asdict(f) | {"error": f.is_error} for f in report.findings],
            },
            indent=1,
        )
        + "\n"
    )
    return lines[0]


def run(
    release_dir: Path,
    log_packages_dir: Path,
    reports_dir: Path,
    markers: list[str],
    ros_lib_dirs: list[str],
    mode: str,
    allowlist: str | None,
) -> int:
    report = Report(mode=mode)
    debs = sorted(release_dir.glob("*.deb"))
    report.debs = len(debs)
    with tempfile.TemporaryDirectory(prefix="link-check-") as tmp:
        root = Path(tmp)
        owner = unpack(debs, root)
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = ":".join(
            library_dirs(root) + ros_lib_dirs + [env.get("LD_LIBRARY_PATH", "")]
        ).strip(":")
        check_elves(root, owner, env, report)
        check_build_paths(root, owner, markers, report)
    check_shlibdeps_logs(log_packages_dir, report)
    apply_allowlist(report, load_allowlist(allowlist))
    summary = write_reports(report, reports_dir)
    print(summary)
    print(f"report: {reports_dir / 'link_check.txt'}")
    if mode == "error" and report.errors:
        for f in report.errors[:20]:
            print(f"error: {f.package}: {f.file}: {f.kind}: {f.demangled or f.detail}")
        return 1
    return 0


def main() -> int:
    mode = os.environ.get("COLCON2DEB_LINK_CHECK", "warn")
    if mode not in ("error", "warn", "off"):
        print(f"error: COLCON2DEB_LINK_CHECK must be error, warn or off, got {mode!r}")
        return 2
    if mode == "off":
        print("link check: off")
        return 0
    ros_prefix = (
        os.environ.get("ROS_BASE_PATH") or f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}"
    )
    ros_lib_dirs = [
        d
        for d in (
            f"{ros_prefix}/lib",
            f"{ros_prefix}/lib/x86_64-linux-gnu",
            f"{ros_prefix}/lib/aarch64-linux-gnu",
        )
        if Path(d).is_dir()
    ]
    colcon_work_dir = os.environ.get("colcon_work_dir", "")
    pkg_build_dir = os.environ.get("pkg_build_dir", "")
    return run(
        release_dir=Path(os.environ["release_dir"]),
        log_packages_dir=Path(os.environ["log_packages_dir"]),
        reports_dir=Path(os.environ["log_reports_dir"]),
        markers=[colcon_work_dir, pkg_build_dir],
        ros_lib_dirs=ros_lib_dirs,
        mode=mode,
        allowlist=os.environ.get("COLCON2DEB_LINK_CHECK_ALLOW") or None,
    )


if __name__ == "__main__":
    sys.exit(main())
