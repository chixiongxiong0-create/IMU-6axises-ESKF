# IMU verification report

Base implementation: `aff16bb` (Task 7 fix round 1). Verification date: 2026-09-23 on Windows, x86-64 MinGW-w64 GCC 13.2.0. This is **host evidence**, not Zynq hardware qualification. The ESKF does **not** meet the full frozen scenario gate. Keep the complementary filter available as the current release/default candidate until the ESKF heading/bias behavior is corrected and rerun through the same gate.

## Documentation verification checklist

This checklist was written before the integration guide, then checked against `docs/integration-zynq.md` and the public headers.

- [x] State the strict 485 Hz Zynq trigger and nominal 2.061856 ms period; use measured monotonic nanosecond timestamps.
- [x] Recommend simultaneous chip select assertion for all three ADXRS453s and three parallel MISO lines when available; distinguish register latch alignment from unsynchronized internal MEMS clocks.
- [x] Specify ADXL355 `ODR_LPF=500 Hz/125 Hz`, `EXT_SYNC=10`, external 485 Hz SYNC, and disabled high-pass filter.
- [x] Give the approximately 4.00 ms gyro and 5.27 ms accelerometer group delays, their configured compensation, and the per-device latch-time/skew rule.
- [x] Document SI input units, FRD body and NED navigation signs, quaternion direction, and relative yaw.
- [x] Define freshness separately from SPI transaction success and enumerate every sensor status and output health flag.
- [x] Describe monotonic frame epoch, first-frame behavior, invalid/repeated timestamps, time gaps, and publication decisions.
- [x] Describe calibration record checks, temperature policy, raw-to-body transform, and diagnostic uncalibrated marking.
- [x] Describe stationary or known-quaternion startup, qualified frame requirements, and controlled reset.
- [x] State that the magnetometer API is a state-preserving unavailable stub and absolute heading is unavailable.
- [x] Include the 485 Hz callback pseudocode matching the public `imu_sample_t` and `imu_process` API.
- [x] Verify numeric claims and Sections 1-14 coverage against fresh measured outputs.

## Reproducible host run

The existing project-local `build` directory resolved to `E:\workspace\AI_Projects\IMU\build`, with a CMake cache pointing to this source tree. Automatic approval review **rejected** its requested recursive removal. To preserve those old artifacts, the run used a previously nonexistent, empty `build-task8` directory. This is a fresh configure/build, but the requested deletion of `build` did not occur. No source, acceptance constant, seed, or CMake target changed.

| Command | Exit/result |
| --- | --- |
| `cmake -S . -B build-task8 -G Ninja -DCMAKE_BUILD_TYPE=Debug` | **PASS**, exit 0; GNU C 13.2.0. |
| `cmake --build build-task8` | **PASS**, exit 0; 43/43 build steps, **0 warnings**. |
| `ctest --test-dir build-task8 --output-on-failure` | **FAIL**, exit 1; **13/14** tests pass, only `scenarios` fails (77 assertions, 1 grouped assertion failure). |
| `.\build-task8\tools\imu_compare.exe --seeds 30 --output build-task8\comparison.csv` | **PASS**, exit 0; 30 fixed seeds, ESKF tilt RMS **0.017359389** versus complementary **0.0219991227 rad**. |
| `.\build-task8\tools\imu_replay.exe tests\data\replay_smoke.csv build-task8\replay_output.csv` | **PASS**, exit 0; five data rows emitted under the expected CSV header. CTest `replay_smoke` also passed. |
| `.\build-task8\test_scenarios.exe --diagnose build-task8\peak_trace.csv` | **PASS**, exit 0; 3 assertions, 0 failures; 5,821 data rows. Diagnostic trace does not alter estimator input or acceptance. |

Compiler: `D:/Programs/Strawberry/c/bin/gcc.exe` (MinGW-W64 x86_64-ucrt-posix-seh, GCC 13.2.0); CMake 3.31.12, Ninja 1.12.0. CMake cache records `CMAKE_BUILD_TYPE=Debug`, `CMAKE_C_FLAGS` empty, and Debug `-g`. `CMakeLists.txt` requires C99 without extensions and applies `-Wall -Wextra -Wpedantic -Werror` to the library and test/tool targets; no compile warning appeared. These are host settings, not measured Zynq compiler settings.

The original full CTest run passed `types`, `math`, `calibration`, `biquad`, `motion_quality`, `esekf`, `complementary`, `imu`, `simulator`, `recovery_metrics`, `comparison`, `replay_parser`, and `replay_smoke`. `scenarios` failed for the combined vibration/bias fixture. The other ten scenarios met their checks. The runner printed `combined_vibration_bias roll=0.0145121 pitch=0.00949131 peak=0.182522 recovery=0.0371134 failures=0`. The original assertion grouped peak and yaw, but **only peak breaches its executable limit**. The final fix separates those assertions for clearer diagnostics without changing the gate.

## Frozen acceptance criteria

The scenario fixtures in `tools/imu_compare.c` freeze limits of roll/pitch RMS <0.05236 rad each, full-quaternion peak <0.174533 rad, relative-yaw RMS <0.349066 rad, recovery <=4 s, bias error <0.08 rad/s, consistency surrogate <100, zero numerical failures/unreported degraded frames, and no error discontinuity above 0.034907 rad. Commands in this table use seed 42 and the existing scenario runner; values are the worst among its eleven scenarios. `scenario_metrics.exe` was a build-local diagnostic that calls `imu_run_scenario(i,42,m)` and prints its `imu_metrics_t`; it is not part of the library or test gate.

Correction: the previous report reversed the yaw and discontinuity thresholds by misreading the positional initializer relative to `imu_scenario_t` field order. Its claim of a yaw gate failure was wrong. The executable frozen mapping is the agreed source of truth: yaw 0.349066 rad, discontinuity 0.034907 rad. Named initialization and the `frozen_limits` regression now make that mapping explicit. No numeric threshold was relaxed. If the original intent had instead been a 0.034907 rad yaw threshold, that would be an additional failure, but it was not the gate executed or approved for preservation.

| Criterion | Producing command; measured result | Result |
| --- | --- | --- |
| Roll RMS <0.05236 rad | `.\build-task8\scenario_metrics.exe`; max **0.0145121199** rad (combined) | **PASS** |
| Pitch RMS <0.05236 rad | same; max **0.0094913058** rad (combined) | **PASS** |
| Quaternion peak <0.174533 rad | `ctest --test-dir build-task8 --output-on-failure` and diagnostic; **0.182522478** rad (combined, frame 5542) | **FAIL** |
| Relative-yaw RMS <0.349066 rad | `.\build-task8\scenario_metrics.exe`; **0.062285284** rad (combined) | **PASS** |
| Recovery <=4 s | same; max **0.0371134021 s** (combined, seed 42) | **PASS** |
| Bias error <0.08 rad/s | same; max **0.0143807246 rad/s** (combined) | **PASS** |
| Consistency surrogate <100 | same; max **2.09329737** (combined) | **PASS** |
| No numerical failures or unreported degraded frames | same; **0** across all eleven scenarios (64,031 frame outputs) | **PASS** |
| No error discontinuity above 0.034907 rad | same; **0** counted discontinuities | **PASS** |
| 30-seed ESKF tilt RMS lower than complementary, no additional numerical failures/discontinuities | `imu_compare --seeds 30`; **0.017359389 < 0.0219991227 rad**, both **0** failures and **0** discontinuities | **PASS** |
| Complete CTest suite | `ctest --test-dir build-task8 --output-on-failure`; **13/14**, one grouped scenario assertion failed | **FAIL** |
| CSV replay smoke | `imu_replay` command above and `ctest` `replay_smoke`; exit **0**, five output rows | **PASS** |

The combined peak is at frame **5542**, time **11.4268041237 s** within the trajectory, input timestamp **12,426,804,123 ns**. Task 7's diagnostic localized most of the excess to relative yaw and vertical gyro-bias behavior during gravity reacquisition; roll/pitch were small at the peak. This report does not change the frozen full-quaternion or yaw limits. The 30-seed tilt result proves an improvement in that specific aggregate measure, while the failed seed-42 peak limit prevents whole-suite acceptance. Across the 30 comparison seeds, ESKF maximum peak was **0.19400185 rad** versus complementary **0.0503783399 rad**; neither had a numerical failure or discontinuity. Thus the complementary filter is the measured lower-peak release candidate, although it has worse aggregate tilt RMS. The comparison executable's “ESKF accepted” message refers only to its narrower tilt/failure/discontinuity criterion, not complete acceptance.

## Numerical and resource evidence

| Check | Producing command or code path; result | Limit |
| --- | --- | --- |
| Quaternion normalization | `.\build-task8\test_scenarios.exe --diagnose build-task8\peak_trace.csv`; reconstructing norm from its rounded `qw,qx,qy,qz` columns gives max observed `abs(norm-1)` approximately **9.01e-8** for 5,821 combined seed-42 frames. The scenario runner also checks every ESKF frame against **1e-5**, with zero counted failures. | The trace number is limited by CSV rounding and one scenario; it is not a global maximum or target-hardware bound. |
| Covariance definiteness | `imu_run_scenario` calls `imu_esekf_covariance_valid` every ESKF frame; **64,031/64,031** scenario checks passed. It checks finite positive diagonals, relative symmetry tolerance `1e-5`, and a normalized-correlation Cholesky pivot margin `8*FLT_EPSILON`, permitting at most `32*FLT_EPSILON` diagonal-loading repair. | **Minimum eigenvalue was not measured**. This is a positive-definiteness/check surrogate, not a reported eigenvalue. |
| Worst simulation recovery | `.\build-task8\scenario_metrics.exe`: seed-42 scenario max **0.0371134021 s**. `build-task8\comparison.csv`: among 30 combined seeds, ESKF max **0.0391752577 s**, complementary max **0.612371134 s**. | Simulator recovery uses the physical disturbance endpoint, a 2-degree tilt band, and a final 0.5-second settled window. |
| Fixed object memory | `build-task8\comparison.csv` reports **`sizeof(imu_t)=4432` bytes** and complementary plus shared quality storage **3216** bytes on this host ABI. | Excludes caller frame/output, call stack, code, linker data, and platform driver; rerun size and map analysis on Zynq. |
| Host CPU sampling (unequal workloads) | Historical `build-task8\comparison.csv`: 30 runs of 5,821 frames each total **3.724 s** for complete `imu_process` calls and **0.324 s** for `imu_complementary_process` only (`clock()` process time); worst single run **0.199 s** and **0.017 s**. ESKF timing includes calibration, notches, delay compensation, quality, lifecycle and covariance; complementary timing excludes its caller's calibration/delay/shared-quality work. | Not a fair estimator-only speed ratio or a Zynq worst-case execution-time bound. Per-call `clock()` sampling has coarse resolution; do not infer microsecond precision. Instrument equal workloads and target interrupt load for qualification. |
| Heap allocation | `rg -n '\b(malloc|calloc|realloc|free)\s*\(' src include` returned no matches (exit 1). | Static source scan, not a whole-system allocation trace. |
| Stack-heavy production sites | Source inspection: `imu_process` makes a local full `imu_t n` (host size 4432 B) plus calibrated/sample copies; `imu_init` and `imu_reset` also make local `imu_t n`; ESKF updates use several fixed 6×6/3×3 arrays. | No measured maximum stack depth or target stack watermark; size the task stack on Zynq. |
| Strict warnings | `cmake --build build-task8`: **0** warnings, exit 0 with `-Werror`. | Host compiler only. |
| Sanitizers | Tiny link probes with `gcc -x c - -fsanitize=undefined` and `-fsanitize=address` both **failed** to link: `cannot find -lubsan` and `cannot find -lasan`. | **No UBSan or ASan runtime results** on this MinGW host; use a supported compiler/runtime in a separate run. |

## Specification coverage, Sections 1–14

| Section | Implementation/test/documentation evidence | Remaining limitation |
| --- | --- | --- |
| 1. Purpose/scope | `src/imu.c`, `include/imu/imu_sensor_port.h`; guide states driver/FPGA ownership. | Board integration not exercised. |
| 2. FRD/NED, yaw | `imu_types.h`, math tests, guide sign test and relative-yaw warning. | Absolute yaw unavailable as designed. |
| 3. Sensor timing | `imu_sensor_port.h`, config delay fields, simulator timing tests, guide's 485 Hz wiring and latch rules. | True gyro internal phases, FPGA skew, and ADXL355 sync not measured on hardware. |
| 4. Six-state ESKF | `imu_esekf.c`, `test_esekf.c`; covariance and quaternion checks above. | Combined scenario ESKF gate fails. |
| 5. Gravity qualification | `imu_motion_quality.c`, `imu_esekf_update_gravity`, quality/scenario tests. | Combined gravity reacquisition still produces excess relative yaw. |
| 6. Static/zero-rate | Motion-quality static dwell and ESKF zero-rate tests. | Validate static thresholds on vehicle logs. |
| 7. Vibration | Optional fixed biquads, quality weighting, broadband/combined simulations, guide mounting check. | Actual mechanical resonance and vibration rectification unmeasured. |
| 8. Calibration | `imu_calibration.c`, `test_calibration.c`, guide storage/checksum/temperature sequence. | Board calibration record and temperature sweep unmeasured. |
| 9. Architecture | Public `include/imu/*`, `src/*`, C99 build; no production heap calls; guide. | Target memory/stack not measured. |
| 10. Error/numerical safety | `test_imu.c`, `test_esekf.c`, replay/parser tests, scenario covariance checks. | ASan/UBSan unavailable in this host toolchain. |
| 11. Initialization | `imu_init`, `imu_reset`, lifecycle tests; guide startup sequence. | Physical stationary alignment not field tested. |
| 12. Magnetometer extension | `imu_update_magnetometer` and ABI no-op test; guide says unavailable. | Heading aiding intentionally unimplemented. |
| 13. Verification/acceptance | 14 CTest registrations, 11 scenarios, 30-seed comparison, CSV replay, measured tables above. | Full suite **FAIL**; no external-truth hardware replay, no binary capture parser, no target CPU/stack validation. |
| 14. Deliverables | Library, complementary benchmark, placeholder port, simulator, replay/compare tools, integration guide, report. | Acceptance and hardware qualification remain open. |

## Final fix wave verification and repository handoff

Base head `f684064`; rerun on 2026-09-23 in a newly created `build-final-fix` directory, preserving existing builds. GNU C 13.2.0, C99, Debug, `-Wall -Wextra -Wpedantic -Werror`; configure and all 43 build steps passed with no compiler warnings.

| Fresh command | Observed result |
| --- | --- |
| `cmake -S . -B build-final-fix -G Ninja -DCMAKE_BUILD_TYPE=Debug` and `cmake --build build-final-fix` | Exit 0 each. |
| `ctest --test-dir build-final-fix --output-on-failure` | Exit 1; **14/15 pass**. Only `scenarios` fails: 88 assertions, one failed peak assertion. `frozen_limits` is the new fifteenth test. |
| `build-final-fix/tools/imu_compare --seeds 30 --output build-final-fix/comparison.csv` | Exit 0; ESKF tilt RMS **0.017359389**, complementary **0.0219991227 rad**, unchanged. |
| `build-final-fix/tools/imu_replay tests/data/replay_smoke.csv build-final-fix/replay_output.csv` | Exit 0, five rows under the existing CSV header. |

Fresh seed-42 combined metrics remain peak **0.182522 > 0.174533 rad (FAIL)** and yaw RMS **0.0622853 < 0.349066 rad (PASS)**. The new vertical-attitude, equal-delay restriction, physical-epoch, and frozen-field-mapping tests pass. Startup epoch underflow has explicit degraded output; no unsigned wrap or current-host-epoch prediction is claimed. The comparison still scores the same host-time truth as before, so these metadata fixes do not change its metric or gate.

The extra output timestamp changes the pre-release ABI: recompile callers. Fresh host `sizeof(imu_t)` is **4440 bytes**, up from the historical 4432; complementary plus shared quality remains **3216 bytes**. Earlier memory/CPU tables describe the historical Task 8 run, not this ABI. Fresh comparison still has zero numerical failures/discontinuities, ESKF maximum peak **0.19400185 rad**, and complementary maximum peak **0.0503783399 rad**. CPU columns remain unequal workloads as labeled above.

The approved design and implementation plan under `docs/superpowers/specs` and `docs/superpowers/plans` are now included in Git. Four accidentally tracked scratch reports (`task-4-fix1-report.md`, `task-4-fix2-report.md`, and the canonical SDD `task-2-fix1-report.md` / `task-2c-report.md`) were removed from the Git index only; their on-disk files remain. User PDFs, `.vscode`, and existing build directories were preserved. No project ignore rule or user editor preference was changed. New verification artifacts are in untracked `build-final-fix`; the detailed RED/GREEN record is in the local canonical SDD `final-fix-report.md`. Full acceptance remains blocked by the preserved peak criterion.

The original numeric tables come from the fresh Task 8 `build-task8` run; the threshold interpretation is corrected above. The final fix wave changes singular Euler uncertainty handling, makes physical attitude epochs explicit, and rejects unsupported unequal gyro delays. It does not tune the estimator or change numeric acceptance gates. The known peak failure and unavailable sanitizers remain explicit. Whole-branch acceptance requires the separate independent review and final fresh verification gate specified in the implementation plan; this report does not claim that gate has passed.
