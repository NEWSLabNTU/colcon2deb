# Phase 02: Pipelined Build (Option B)

## Goal

Run colcon build (phase 4) and Debian packaging (phases 5-8) concurrently
instead of serially, cutting wall-clock to roughly `max(colcon, packaging)`
instead of their sum — while preserving debian-overrides handling,
fingerprint skips, exit-code semantics, and all `--skip-*` flags.

## Why it works

Established empirically (see one-pass pipeline study):

- `debian/rules binary` recompiles every package from source; the colcon
  pass only provides the workspace install tree so a package's *dependencies*
  resolve during the dh build.
- Therefore `deb(X)` needs: `debian/` for X, and colcon to have finished
  **deps(X)** — not X's own colcon build. Gating on "colcon finished X"
  is stricter than necessary but free and always correct (colcon finishes
  deps(X) before X).
- Phases 5-7 (rosdep list, package list, bloom) need no build at all.
- Per-package completion signal: colcon `log/latest_build/events.log` lines
  `[t] (pkg) JobEnded: {'identifier': 'pkg', 'rc': 0}` — per-run, immune to
  stale install-tree markers from previous runs.
- `local_setup.bash` enumerates prefixes at source time, so dh builds
  started mid-colcon-run see exactly the dependencies installed so far.

## Design

```
phase1 prepare → phase2 copy-src → phase3 install-deps
     ├── colcon thread: phase4 build-src.sh
     │      └── writes log/build_<ts>/events.log (JobEnded per package)
     │      └── on exit: orchestrator writes <output>/.colcon.status ("0"/"N")
     └── packaging thread: phase5 rosdep list → phase6 package list
            → phase7 generate_debian_dir.py   (no build needed)
            → phase8 build_deb.py             (per-package colcon gate)
join both → summary (existing exit-code logic)
```

- New `helper/colcon_events.py`: tail-parser for events.log
  (`parse_job_ended`, `find_events_log(log_base, after)`), and
  `ColconGate` — per-package wait with three release conditions:
  JobEnded(pkg, rc=0) seen · colcon status file reports success (release
  all) · gate disabled (`--skip-colcon-build`). JobEnded with rc != 0 or
  colcon failure marks the package (and any not-yet-ended package)
  unbuildable; build_deb records those as failed with a clear error.
- `build_deb.py`: before building pkg X, `gate.wait(X)`; on
  "unbuildable" → FAILED "colcon build failed for/before this package".
  Gate config via env: `COLCON2DEB_COLCON_LOG_BASE`,
  `COLCON2DEB_COLCON_STATUS_FILE`, `COLCON2DEB_PIPELINE` (0 disables
  gating = wait for status file only).
- `helper/main.py`: phases 4 and 5-8 run in two threads; phase-level
  events/logs unchanged (TUI already renders two running phases and
  per-package progress). Workspace env capture is dropped — phases 5-7
  need only the base ROS env, and phase 8's rules self-source
  `local_setup.bash` (the previous hard-fail on missing setup.bash moves
  to a post-colcon warning).
- Parallelism budget v1: colcon keeps its workers; dh pool keeps the sqrt
  split. Moderate oversubscription while both run; refine later if needed.
- Config: `build.pipeline: true` (default). `false` restores serial
  ordering by making phase 5-8 thread wait for colcon completion first.

## Failure semantics

- colcon fails → packages with JobEnded rc=0 still package (their deps are
  complete); the rest fail with "colcon build failed"; overall exit 1 via
  existing `last_failing_phase` + failed.txt paths.
- dh failures never stop colcon.
- `--skip-colcon-build`: gate disabled; behaves as today.

## Work items

- [x] `helper/colcon_events.py` with unit tests (parser, log discovery, gate)
- [x] `build_deb.py` gating with unit tests
- [x] `helper/main.py` concurrent orchestration + `.colcon.status`
- [x] `config.py` `build.pipeline` flag + docs
- [x] CLAUDE.md build-flow update
- [x] Integration: suite passes with pipeline on; new test asserts
      packaging (phase 5) starts before phase 4 completes (events.jsonl ordering)

## Acceptance criteria

- [x] Full fixture build produces identical debs with pipeline on
- [x] Broken colcon build still exits non-zero; packages report failure
- [x] Fingerprint suite (incl. renamed-dir case) passes unchanged
- [x] events.jsonl shows packaging starting before phase 4 completes
- [x] `build.pipeline: false` restores serial behavior
