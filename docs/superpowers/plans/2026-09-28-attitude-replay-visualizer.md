# 3D Attitude Replay Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a local Python GUI that replays one or two IMU attitude CSV files as 3D bodies with NED reference axes and two comparison layouts.

**Architecture:** A CSV adapter returns validated immutable records; a renderer-independent controller maps one shared integer-nanosecond playback clock to each stream. A PyVista/PyVistaQt scene consumes selected records while a PySide6 window owns playback controls, file loading and layout switching. The adapter boundary permits a future realtime source without implementing one now.

**Tech Stack:** Local Python installation (no virtual environment), standard-library `csv`/`unittest`, PyVista, PyVistaQt, PySide6; existing C `imu_replay` executable supplies sample CSV.

**Spec:** `docs/superpowers/specs/2026-09-28-attitude-replay-visualizer-design.md`

## Global Constraints

- Do not modify the C estimator or `imu_replay` CSV schema: `timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status`.
- Input `q_nb` is scalar-first Hamilton, maps FRD body to NED navigation. Rotate actors with a matrix derived directly from quaternion, not Euler angles.
- Timestamps are integer nanoseconds. The UI label says `timestamp_ns` is host reference time, not compensated physical attitude epoch.
- Single or dual CSV; dual layout can be side-by-side or common-scene overlay. No inferred physical position or absolute heading.
- Controls: play/pause, scrub, speed 0.25×/0.5×/1×/2×/4×; reach end → pause without loop. Screen refresh is independent of 485 Hz data rate.
- The CSV adapter, clock and selection logic must run without importing Qt or PyVista; future realtime reception is out of scope.
- Install PyVista, PyVistaQt and PySide6 into the user's local Python environment, with no virtual environment. Do not mutate/remove unrelated user packages.
- Preserve existing untracked files (including `README.md`), PDFs, build trees and existing C test failure; add visualizer documentation separately or update only relevant README sections without deleting user content.

## Review Focus

1. CSV with duplicate/descending or `uint64`-overflow timestamp must fail with source line, rather than display time backward; Task 1 tests this.
2. Zero/nonfinite quaternion or out-of-range weight/status must fail before rendering; Task 1 tests this.
3. Large time origin plus small ~2 ms steps must retain nanosecond ordering during scrub/play; Task 2 tests this.
4. A gap or non-overlap in a dual replay must show “no data” and not silently hold the old body indefinitely; Task 2 tests this.
5. Switching two display modes after scrubbing must preserve clock and speed, and indicate degraded status; Task 4 tests controller/UI state and a manual GUI smoke test.

## File Map

- Create `tools/attitude_viewer/__init__.py`: package marker only.
- Create `tools/attitude_viewer/data.py`: typed immutable `AttitudeSample`/`AttitudeTrack`, strict CSV parser, display-hold limit.
- Create `tools/attitude_viewer/playback.py`: `ReplayController`, timestamp lookup and monotonic-clock playback state; no GUI imports.
- Create `tools/attitude_viewer/geometry.py`: quaternion-to-rotation matrix and FRD/NED geometry conventions; no GUI imports.
- Create `tools/attitude_viewer/scene.py`: PyVistaQt actors, static reference axes, actor updates and single/dual layouts.
- Create `tools/attitude_viewer/app.py`: PySide6 window, controls, file input, Qt timer and status text.
- Create `tools/attitude_viewer/__main__.py`: `python -m tools.attitude_viewer` CLI entry.
- Create `tests/python/test_attitude_data.py`, `test_attitude_playback.py`, `test_attitude_geometry.py`, `test_attitude_app.py`.
- Create `docs/attitude-visualizer.md`; modify `README.md` only if still untracked/otherwise owned by this task and without discarding its content.

---

### Task 1: Strict replay data adapter

**Files:** Create `tools/attitude_viewer/__init__.py`, `tools/attitude_viewer/data.py`, `tests/python/test_attitude_data.py`.

**Interfaces:** Produce `@dataclass(frozen=True) AttitudeSample(timestamp_ns: int, q_nb: tuple[float,float,float,float], accel_weight: float, fusion_mode: int, health_flags: int, status: int)`, `@dataclass(frozen=True) AttitudeTrack(samples: tuple[AttitudeSample,...], hold_ns: int, name: str)`, and `load_replay_csv(path: str | Path) -> AttitudeTrack` raising `ValueError` with path/line on invalid content. `hold_ns = min(2 * median(positive adjacent dt), 100_000_000)`; one sample uses 0.

- [ ] **Step 1: Write failing tests** in `test_attitude_data.py`: exact header and one/two rows parse; norm-2 quaternion normalizes to unit; valid `uint64` maximum parses; duplicate/descending/overflow timestamps, NaN/zero quaternion, `accel_weight` outside [0,1], and negative/overflow `uint32` fields each raise path-and-line `ValueError`; positive diffs 2, 4, 8 ms yield 8 ms hold (median 4 ms ×2).
- [ ] **Step 2: Run** `py -m unittest discover -s tests/python -p test_attitude_data.py -v`; expect RED before implementation.
- [ ] **Step 3: Implement** the exact dataclasses and `load_replay_csv` using stdlib CSV, `math.isfinite`, integer range checks and explicit field count. Reject empty data and malformed file I/O with clear path context. No GUI imports.
- [ ] **Step 4: Re-run** the Task 1 command; expect all Task 1 tests PASS.
- [ ] **Step 5: Commit** only Task 1 files, e.g. `git commit -m "feat: parse attitude replay CSV"`.

### Task 2: Quaternion geometry and shared playback clock

**Files:** Create `tools/attitude_viewer/geometry.py`, `tools/attitude_viewer/playback.py`, `tests/python/test_attitude_geometry.py`, `tests/python/test_attitude_playback.py`.

**Interfaces:** Consume Task 1 `AttitudeTrack`. Produce `rotation_matrix(q_nb: tuple[float,float,float,float]) -> tuple[tuple[float,float,float],...]`; `ReplayController(tracks: tuple[AttitudeTrack,...])` with `position_ns`, `speed`, `playing`, `set_position_ns(int)`, `set_speed(float)`, `play(now_ns: int)`, `pause(now_ns: int)`, `advance(now_ns: int)` and `selected_samples() -> tuple[AttitudeSample | None,...]`. `now_ns` is injectable monotonic wall clock; advance clamps at union end and pauses. Use bisect on integer timestamps, no interpolation.

- [ ] **Step 1: Write failing tests**: identity quaternion preserves FRD x/y/z; +90° yaw maps forward x to east y in NED; dual union range and integer-ns seek near `2**63`; hold expires at `sample.timestamp_ns + hold_ns + 1`; before first/after last is no-data; speed changes preserve current position; 0.25× and 4× advance correctly; end clamps/pauses without loop.
- [ ] **Step 2: Run** `py -m unittest discover -s tests/python -p 'test_attitude_*.py' -v`; expect RED for Task 2 cases.
- [ ] **Step 3: Implement** direct quaternion matrix and controller. Validate exactly one/two tracks and approved speed set. Store `base_position_ns` and `base_wall_ns`; compute elapsed using integer monotonic nanoseconds and a rational speed multiplier so epoch size cannot erase 2 ms resolution.
- [ ] **Step 4: Re-run** Task 2 and Task 1 tests; expect PASS.
- [ ] **Step 5: Commit** only Task 2 files.

### Task 3: PyVista attitude scene

**Files:** Create `tools/attitude_viewer/scene.py`; create a focused scene smoke/check in `tests/python/test_attitude_scene.py` if this host's Qt/VTK offscreen renderer works (otherwise record manual-only GUI validation and test pure actor transforms through geometry).

**Interfaces:** Consume Task 2 `rotation_matrix` and selected `AttitudeSample | None`; produce `AttitudeScene(parent: QWidget, layout: Literal["single","side_by_side","overlay"])`, `set_layout(layout)`, `show_samples(samples, timestamp_ns)`, `reset_cameras()`, `close()`. Scene owns one or two `pyvistaqt.QtInteractor` widgets/actors, not the playback clock.

- [ ] **Step 1: Install dependencies into the local Python environment**, after recording `py --version` and `py -m pip --version`: `py -m pip install PyVista PyVistaQt PySide6`. Verify imports and note resolved versions. No venv.
- [ ] **Step 2: Write a focused failing geometry/scene test**: actor transform from +90° yaw points the nose east; overlay positions are fixed offsets and marked display-only; missing sample hides its body; layout change does not mutate controller state. If offscreen VTK fails, retain non-GUI transform tests and a manual GUI checklist rather than claiming headless GUI pass.
- [ ] **Step 3: Implement** PyVista body mesh with visible nose, fixed NED axes, rotated FRD body axes, distinct stream colors, common scale/camera preset and explicit no-data text. Use `QtInteractor` via PySide6; no Euler actor rotation.
- [ ] **Step 4: Run** the focused tests and launch a simple scene in the host graphical session; verify both layouts and close cleanup. Record any platform limitation.
- [ ] **Step 5: Commit** Task 3 files only.

### Task 4: Window, CLI and user documentation

**Files:** Create `tools/attitude_viewer/app.py`, `tools/attitude_viewer/__main__.py`, `tests/python/test_attitude_app.py`, `docs/attitude-visualizer.md`; optionally modify `README.md` per File Map constraint.

**Interfaces:** Consume Task 1 CSV loader, Task 2 controller and Task 3 scene. `python -m tools.attitude_viewer first.csv [second.csv]` launches UI; no positional CSV opens a file chooser. One file enables single layout, two enables both comparison layouts. Invalid file reports a non-destructive GUI/CLI error with path and line.

- [ ] **Step 1: Write failing tests** for CLI argument count and bad-file diagnostics, playback/scrub/speed state transitions, switching layouts without changing playback clock, and display of non-OK/health flags; use mocked scene where possible to keep tests GUI-light.
- [ ] **Step 2: Run** `py -m unittest discover -s tests/python -p test_attitude_app.py -v`; expect RED.
- [ ] **Step 3: Implement** Qt controls and timer. Clock uses `time.monotonic_ns()`; render at ~30–60 Hz by choosing the sample at current integer-ns playback time. Do not process one GUI event per sensor row. Show host-reference-time label, no-data state and degraded text. Provide layout selection and camera reset.
- [ ] **Step 4: Document** local dependency installation, `imu_replay` → viewer command, one/two file examples, controls, timestamp meaning, display-only overlay offset, known C scenario failure and lack of realtime input. Optionally add a README pointer without replacing its existing content.
- [ ] **Step 5: Verify** `py -m unittest discover -s tests/python -v`, a fresh `imu_replay` smoke CSV, one-file and two-file GUI sessions, plus `cmake --build <existing-build>` and `ctest --test-dir <existing-build> --output-on-failure`. Expect Python tests pass; C CTest may still show its pre-existing 14/15 result and must be reported as such.
- [ ] **Step 6: Commit** only Task 4-owned files and deliberate README addition.

## Final verification

Run all Python tests, exercise both comparison layouts and known +90° rotation on the host display, verify spec-to-task coverage and `git diff --check`, and record exactly which GUI checks were actually observed. Do not push or publish without a fresh user request. The existing C peak gate remains visible and is not retuned for this feature.
