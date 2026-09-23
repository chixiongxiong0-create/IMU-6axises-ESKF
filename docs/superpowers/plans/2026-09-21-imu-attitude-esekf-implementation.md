# ADXRS453 + ADXL355 IMU Attitude Estimator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a portable C99, float-only, 485 Hz six-axis attitude-estimation library using a six-error-state multiplicative ESKF, with an adaptive complementary-filter benchmark and future magnetometer interface.

**Architecture:** Timestamped SI-unit samples pass through calibration, optional notches, motion-quality analysis, and then quaternion/bias estimators. The primary ESKF maintains `q_nb`, three-axis gyroscope bias, and a 6 x 6 covariance; qualified acceleration supplies gravity observations while static periods enable zero-angular-rate updates. Host tests provide deterministic truth simulation, Monte Carlo comparison, and log replay without coupling the core library to any driver or operating system.

**Tech Stack:** C99, `float`, CMake 3.20+, CTest, standard C library and `libm`; no runtime allocation and no third-party production dependency.

**Spec:** `docs/superpowers/specs/2026-09-21-imu-attitude-esekf-design.md`

## Global Constraints

- Body frame is FRD, navigation frame is NED, and `q_nb` rotates body vectors into NED.
- The production library uses C99 `float`, allocates no dynamic memory, and has deterministic execution.
- The public processing task runs nominally at 485 Hz but derives every `dt` from monotonic nanosecond timestamps.
- ADXRS453 useful-range operation is assumed; angular-rate saturation recovery is out of scope.
- Absolute yaw is unobservable without heading aiding and must never be reported as absolute or artificially constrained.
- Accelerometer offset, scale, nonorthogonality, alignment, and temperature effects are calibrated outside the ESKF; accelerometer bias is not an online state.
- Every production function that accepts external numeric input rejects nonfinite values before modifying estimator state.
- Tests are written and observed failing before the corresponding production implementation is added.
- Each sub-agent owns only the files listed for its task; changes to a previously published interface require primary-agent approval.

## Review Focus

- A fresh flag that is false on one gyroscope axis must not silently reuse that axis as an independent new observation; Task 5 tests degraded propagation and health reporting.
- A timestamp jump within the legal range after several rejected frames must integrate only the actual elapsed interval once; Task 5 tests dropout recovery.
- A nearly singular gravity innovation covariance must reject the update without corrupting quaternion or covariance; Task 4 tests rollback behavior.
- A persistent one-g-magnitude lateral acceleration must lose gravity confidence even though its norm can appear plausible; Task 3 tests innovation persistence and quality decay.
- A magnetometer call before magnetic configuration must leave every estimator state byte unchanged and return `IMU_STATUS_AIDING_UNAVAILABLE`; Task 5 tests this ABI promise.

---

## File Map and Task Ownership

```text
CMakeLists.txt                         Task 1: build and test targets
include/imu/imu_types.h               Task 1: shared public types and flags
include/imu/imu.h                     Task 5: public lifecycle API
include/imu/imu_sensor_port.h         Task 1: acquisition contract
include/imu/imu_math.h                Task 2: vector, matrix, quaternion API
include/imu/imu_calibration.h         Task 2: calibration API
include/imu/imu_biquad.h              Task 2: optional notch API
include/imu/imu_motion_quality.h      Task 3: quality/static API
include/imu/imu_esekf.h               Task 4: ESKF API
include/imu/imu_complementary.h       Task 6: comparison estimator API
src/imu_math.c                        Task 2
src/imu_calibration.c                 Task 2
src/imu_biquad.c                      Task 2
src/imu_motion_quality.c              Task 3
src/imu_esekf.c                       Task 4
src/imu.c                             Task 5
src/imu_complementary.c               Task 6
tests/test_framework.h                Task 1: dependency-free test macros
tests/test_types.c                    Task 1
tests/test_math.c                     Task 2
tests/test_calibration.c              Task 2
tests/test_biquad.c                   Task 2
tests/test_motion_quality.c           Task 3
tests/test_esekf.c                    Task 4
tests/test_imu.c                      Task 5
tests/test_complementary.c            Task 6
tests/sim/imu_sim.h                   Task 7: truth/noise generator
tests/sim/imu_sim.c                   Task 7
tests/test_scenarios.c                Task 7
tools/imu_replay.c                    Task 7: host CSV replay
tools/imu_compare.c                   Task 7: seeded estimator comparison
docs/integration-zynq.md              Task 8: integration contract
docs/verification-report.md           Task 8: measured results
```

## Execution Topology

Task 1 establishes all cross-task scalar conventions and build plumbing. After Task 1 review, Tasks 2 and 3 may execute in parallel because they own disjoint files and consume only Task 1 types. Task 4 starts after Task 2; Task 5 starts after Tasks 2, 3, and 4. Task 6 starts after Tasks 2 and 3 and may run in parallel with Task 4 or 5. Task 7 starts after Tasks 5 and 6. Task 8 runs last. Every task receives a fresh implementation sub-agent and then a fresh review sub-agent before dependent work starts.

---

### Task 1: Build Foundation, Public Types, and Acquisition Contract

**Files:**
- Create: `CMakeLists.txt`
- Create: `include/imu/imu_types.h`
- Create: `include/imu/imu_sensor_port.h`
- Create: `tests/test_framework.h`
- Create: `tests/test_types.c`

**Interfaces:**
- Consumes: none.
- Produces: `imu_vec3f_t`, `imu_quatf_t`, `imu_sample_t`, `imu_config_t`, `imu_calibration_t`, `imu_output_t`, `imu_status_t`, sensor flags, and the driver callback contract used by every later task.

- [ ] **Step 1: Initialize version control and write the failing public-type test**

Run `git init` only if `git rev-parse --is-inside-work-tree` fails. Add `tests/test_types.c` with compile-time and runtime assertions:

```c
#include "test_framework.h"
#include "imu/imu_types.h"

int main(void) {
    imu_sample_t s = {0};
    s.timestamp_ns = 2061856ULL;
    s.gyro_sample_fresh[0] = true;
    TEST_ASSERT(sizeof(float) == 4U);
    TEST_ASSERT(s.timestamp_ns == 2061856ULL);
    TEST_ASSERT(s.gyro_sample_fresh[0]);
    TEST_ASSERT(IMU_FRAME_FRD_NED == 1);
    return test_summary();
}
```

- [ ] **Step 2: Configure and run the test to verify RED**

Create only the test target reference in `CMakeLists.txt`, then run:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target test_types
```

Expected: compilation fails because `imu/imu_types.h` does not exist.

- [ ] **Step 3: Add exact shared types and status contracts**

Define fixed public fields, including:

```c
typedef struct { float x, y, z; } imu_vec3f_t;
typedef struct { float w, x, y, z; } imu_quatf_t;

typedef enum {
    IMU_STATUS_OK = 0,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_BAD_ARGUMENT,
    IMU_STATUS_NONFINITE,
    IMU_STATUS_TIME_REVERSED,
    IMU_STATUS_TIME_GAP,
    IMU_STATUS_SENSOR_DEGRADED,
    IMU_STATUS_NUMERICAL_FAULT,
    IMU_STATUS_AIDING_UNAVAILABLE
} imu_status_t;

typedef struct {
    uint64_t timestamp_ns;
    float gyro_rad_s[3];
    float gyro_temperature_c[3];
    uint32_t gyro_status[3];
    float accel_m_s2[3];
    float accel_temperature_c;
    uint32_t accel_status;
    bool gyro_sample_fresh[3];
    bool accel_sample_fresh;
} imu_sample_t;
```

Define sensor bits for data valid, parity, sequence, PLL, power, self-test, FIFO, and overrange. Define configuration fields for gravity, nominal/min/max `dt`, signal-chain delays, process noises, observation noises, quality thresholds, static thresholds, notch definitions, and initial covariance. Define calibration matrices and temperature polynomials with fixed-size arrays only.

- [ ] **Step 4: Add the dependency-free test macros and complete CMake targets**

Use a header-only test framework whose assertions increment counters and return a nonzero summary on failure. Configure C99 with extensions disabled and warnings-as-errors for project sources:

```cmake
cmake_minimum_required(VERSION 3.20)
project(imu_attitude C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()
add_executable(test_types tests/test_types.c)
target_include_directories(test_types PRIVATE include tests)
if(MSVC)
  target_compile_options(test_types PRIVATE /W4 /WX)
else()
  target_compile_options(test_types PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
add_test(NAME types COMMAND test_types)
```

- [ ] **Step 5: Verify GREEN and checkpoint**

Run `cmake --build build --target test_types` and `ctest --test-dir build --output-on-failure`. Expected: one test passes and compiler output contains no warnings. Commit `chore: establish C99 IMU interfaces and test harness`.

---

### Task 2: Quaternion Math, Calibration, and Optional Biquads

**Files:**
- Create: `include/imu/imu_math.h`
- Create: `include/imu/imu_calibration.h`
- Create: `include/imu/imu_biquad.h`
- Create: `src/imu_math.c`
- Create: `src/imu_calibration.c`
- Create: `src/imu_biquad.c`
- Create: `tests/test_math.c`
- Create: `tests/test_calibration.c`
- Create: `tests/test_biquad.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 vector/quaternion/config/calibration types.
- Produces: `imu_quat_integrate()`, `imu_quat_rotate()`, fixed 3 x 3 and 6 x 6 operations, `imu_calibrate_sample()`, and `imu_biquad_process()`.

- [ ] **Step 1: Write failing quaternion convention and integration tests**

Test identity, 90-degree FRD yaw, inverse rotation, small-angle integration, 300 deg/s integration for 2.061856 ms, and zero-norm rejection:

```c
imu_quatf_t q = {1.0f, 0.0f, 0.0f, 0.0f};
imu_vec3f_t w = {0.0f, 0.0f, 5.23598776f};
TEST_ASSERT(imu_quat_integrate(&q, w, 0.002061856f));
TEST_ASSERT_NEAR(imu_quat_norm(q), 1.0f, 2.0e-6f);
TEST_ASSERT_NEAR(imu_quat_yaw_ned(q), 0.0107958f, 2.0e-5f);
```

- [ ] **Step 2: Verify quaternion tests fail for missing functions**

Run `cmake --build build --target test_math`. Expected: link or compile failure naming `imu_quat_integrate`.

- [ ] **Step 3: Implement minimal guarded math primitives**

Implement dot/cross/norm, 3 x 3 multiplication, 6 x 6 zero/identity/multiply/transpose/symmetrize, quaternion multiply/conjugate/normalize/rotate, and exponential-map integration. Use the small-angle branch when `|omega * dt| < 1.0e-4f`; otherwise use `sinf` and `cosf`. Return `false` before output mutation for nonfinite input or norm below `1.0e-12f`.

- [ ] **Step 4: Verify math GREEN**

Run `ctest --test-dir build -R math --output-on-failure`. Expected: all math cases pass with no warnings.

- [ ] **Step 5: Write failing calibration tests**

Use a nonidentity axis matrix and quadratic temperature bias to prove operation order:

```c
/* expected = body_rotation * (scale_alignment * raw - bias(T)) */
imu_sample_t in = calibrated_fixture_input();
imu_calibrated_sample_t out;
TEST_ASSERT(imu_calibrate_sample(&cal, &in, &out));
TEST_ASSERT_VEC3_NEAR(out.gyro_rad_s, expected_gyro, 1.0e-6f);
TEST_ASSERT_VEC3_NEAR(out.accel_m_s2, expected_accel, 1.0e-5f);
```

Also assert rejection of NaN coefficients, a nonorthonormal board rotation, checksum-invalid metadata, and temperatures outside the calibrated range when extrapolation is disabled.

- [ ] **Step 6: Verify calibration RED, implement, and verify GREEN**

Run the test before adding `src/imu_calibration.c`; expect a missing-symbol failure. Implement Horner-form temperature polynomials, matrix application, rotation validation, and transactional output assignment. Re-run `ctest --test-dir build -R calibration --output-on-failure`; expect pass.

- [ ] **Step 7: Write failing biquad passthrough and notch tests**

Assert bypass is bitwise transparent, state reset is deterministic, coefficients remain finite, and a configured 100 Hz notch at 485 Hz attenuates a steady sinusoid while leaving 10 Hz substantially unchanged.

- [ ] **Step 8: Verify biquad RED, implement transposed direct-form II, and verify GREEN**

Reject notch frequencies outside `(0, 0.49 * sample_rate)` and nonpositive Q. Normalize coefficients by `a0`, process with two float states per axis, and reset state when configuration changes. Run all three Task 2 tests and the full suite. Commit `feat: add guarded IMU math calibration and notch primitives`.

---

### Task 3: Motion Quality, Observation Weight, and Static Detection

**Files:**
- Create: `include/imu/imu_motion_quality.h`
- Create: `src/imu_motion_quality.c`
- Create: `tests/test_motion_quality.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 public types and configuration.
- Produces: `imu_motion_quality_init()`, `imu_motion_quality_update()`, and `imu_motion_quality_t` containing `accel_weight`, `vibration_rms`, `jerk`, `innovation_angle_rad`, `static_detected`, and mode reasons.

- [ ] **Step 1: Write failing quality and hysteresis tests**

Cover stable one-g data, a single 4 g impulse, broadband alternating vibration, slow sustained lateral acceleration with near-plausible total magnitude, invalid status, recovery slew limiting, and static entry/exit dwell:

```c
for (unsigned i = 0; i < cfg.static_enter_samples - 1U; ++i)
    imu_motion_quality_update(&mq, &cfg, stable_sample(i), 0.0f);
TEST_ASSERT(!mq.static_detected);
imu_motion_quality_update(&mq, &cfg, stable_sample(cfg.static_enter_samples), 0.0f);
TEST_ASSERT(mq.static_detected);
```

The persistent lateral-acceleration test must show decreasing weight due to persistent innovation even when the chosen vector magnitude lies inside the magnitude soft gate.

- [ ] **Step 2: Verify RED**

Run `cmake --build build --target test_motion_quality`. Expected: missing header/function failure.

- [ ] **Step 3: Implement fixed-window statistics and continuous weight**

Use fixed-size ring buffers bounded by `IMU_QUALITY_WINDOW_MAX`. Calculate acceleration magnitude residual, jerk, high-pass residual RMS, and innovation persistence. Combine smoothstep factors in `[0,1]`, apply hard status rejection, then limit weight rise and fall per second. Maintain independent static entry/exit counters with saturation.

- [ ] **Step 4: Add boundary and nonfinite tests, then verify GREEN**

Test zero-length configuration, maximum window, counter saturation, timestamp-sized `dt`, and NaN/Inf without state mutation. Run `ctest --test-dir build -R motion_quality --output-on-failure`, then the full suite. Commit `feat: add vibration-aware acceleration quality and static detection`.

---

### Task 4: Six-Error-State Multiplicative ESKF

**Files:**
- Create: `include/imu/imu_esekf.h`
- Create: `src/imu_esekf.c`
- Create: `tests/test_esekf.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 types and Task 2 math functions.
- Produces: `imu_esekf_init()`, `imu_esekf_predict()`, `imu_esekf_update_gravity()`, `imu_esekf_update_zero_rate()`, `imu_esekf_get_output()`, and covariance validation.

- [ ] **Step 1: Write failing propagation tests**

Assert identity remains identity at zero rate, a known compound rotation matches a high-resolution truth quaternion, bias subtraction is applied, yaw covariance grows without heading observations, and repeated 485 Hz propagation keeps quaternion norm within `2e-5` over ten simulated minutes.

- [ ] **Step 2: Verify propagation RED**

Run `cmake --build build --target test_esekf`. Expected: missing ESKF symbols.

- [ ] **Step 3: Implement state and covariance prediction**

Use state transition blocks:

```text
F(theta,theta) = I - skew(omega_corrected) * dt
F(theta,bias)  = -I * dt
F(bias,bias)   = I
Qtheta         = gyro_noise_density^2 * dt
Qbias          = gyro_bias_rw^2 * dt
Pnew           = F * P * F^T + Q
```

Integrate `q_nb` with Task 2 exponential-map integration, symmetrize `P`, enforce finite positive diagonals, and commit output only after all guards pass.

- [ ] **Step 4: Verify propagation GREEN**

Run only propagation cases, then all existing ESKF cases. Expected: propagation passes; gravity-update tests remain absent.

- [ ] **Step 5: Write failing gravity-update tests**

Initialize known roll/pitch errors and verify qualified static gravity reduces them while leaving yaw unchanged within tolerance. Verify zero weight is a no-op, higher measurement noise reduces correction, NIS rejection is transactional, a near-singular innovation covariance returns `IMU_STATUS_NUMERICAL_FAULT`, and covariance remains symmetric.

- [ ] **Step 6: Implement gravity update with Joseph covariance form**

Normalize calibrated specific force, predict the body-frame specific-force direction as the negative body-frame gravity direction, form the cross-product residual and 3 x 6 Jacobian, invert the symmetric 3 x 3 innovation matrix with determinant/finite guards, and update with:

```text
P = (I-KH) P (I-KH)^T + K R K^T
q = q * exp(0.5 * delta_theta)
b_g = b_g + delta_b_g
```

Apply the documented quaternion multiplication side consistently with body-frame error coordinates and reset covariance after injection.

- [ ] **Step 7: Write failing zero-rate and rollback tests**

Verify a stationary rate observation converges all three bias axes, bias step size is bounded, invalid rate status is rejected, and every failing update leaves `q`, `b_g`, and `P` byte-identical.

- [ ] **Step 8: Implement zero-rate update and verify full Task 4 GREEN**

Use measurement `z = gyro_calibrated = b_g + noise`, a 3 x 6 Jacobian selecting bias error, and Joseph covariance update. Run `ctest --test-dir build -R esekf --output-on-failure` and the full suite. Commit `feat: implement six-state multiplicative attitude ESKF`.

---

### Task 5: Estimator Orchestration, Initialization, Timing, and Magnetometer Stub

**Files:**
- Create: `include/imu/imu.h`
- Create: `src/imu.c`
- Create: `tests/test_imu.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 1-4 types, calibration, filters, motion quality, and ESKF.
- Produces: stable public functions `imu_init()`, `imu_process()`, `imu_update_magnetometer()`, `imu_reset()`, and `imu_get_output()`.

- [ ] **Step 1: Write failing initialization tests**

Test caller-supplied quaternion initialization and stationary initialization. Require multiple qualified samples for gravity alignment and bias averaging. Assert invalid calibration, unhealthy sensors, or one sample cannot report initialized.

- [ ] **Step 2: Verify initialization RED, implement state machine, and verify GREEN**

Implement `UNINITIALIZED`, `ALIGNING`, `RUNNING`, `DEGRADED`, and `FAULT` internal states. Stationary alignment derives roll/pitch from averaged calibrated specific force, uses caller yaw, averages gyroscope bias, and initializes covariance from configuration.

- [ ] **Step 3: Write failing timestamp and freshness tests**

Cover exact 485 Hz timestamps, legal jitter, reversed time, `dt` below minimum, a gap above maximum, three missing gyro-axis freshness combinations, repeated accelerometer data, and dropout recovery. For recovery, assert elapsed time is neither integrated twice nor discarded silently.

- [ ] **Step 4: Implement transactional 485 Hz processing**

Processing order is: validate sample; compute `dt`; calibrate; update optional notches; predict if all required gyro axes are valid/fresh; compute quality; apply gravity update when fresh/qualified; apply zero-rate update only when static; generate output and health flags. On recoverable observation failure preserve propagation; on hard numerical failure enter `FAULT`.

- [ ] **Step 5: Write failing output and yaw-observability tests**

Assert quaternion normalization, FRD/NED Euler signs, bias-corrected rate, finite uncertainty, fusion-mode flags, acceleration-quality range, yaw variance growth, and no gravity-induced yaw correction.

- [ ] **Step 6: Implement output conversion and uncertainty reporting**

Derive roll/pitch/yaw using the Task 2 convention. Map covariance diagonals to nonnegative standard deviations with finite guards. Mark yaw as relative until heading aiding becomes active.

- [ ] **Step 7: Write and implement the magnetometer ABI no-op test**

Snapshot the entire `imu_t`, call `imu_update_magnetometer()` without magnetic configuration, assert `IMU_STATUS_AIDING_UNAVAILABLE`, and compare the snapshot byte-for-byte. Implement argument validation and the exact state-preserving return.

- [ ] **Step 8: Verify Task 5 and full suite GREEN**

Run `ctest --test-dir build -R imu --output-on-failure`, then `ctest --test-dir build --output-on-failure`. Commit `feat: expose deterministic IMU lifecycle and 485 Hz processing`.

---

### Task 6: Adaptive Complementary-Filter Benchmark

**Files:**
- Create: `include/imu/imu_complementary.h`
- Create: `src/imu_complementary.c`
- Create: `tests/test_complementary.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 types, Task 2 quaternion math, and Task 3 acceleration quality.
- Produces: `imu_complementary_init()`, `imu_complementary_process()`, and output compatible with common comparison metrics.

- [ ] **Step 1: Write failing nominal and rejected-observation tests**

Assert exact zero-rate stability, known-rate quaternion propagation, gravity correction proportional to the shared quality weight, no correction at zero weight, integral bias clamping, and no absolute yaw correction.

- [ ] **Step 2: Verify RED**

Run `cmake --build build --target test_complementary`. Expected: missing symbols.

- [ ] **Step 3: Implement the minimal adaptive Mahony-style estimator**

Use the same gravity direction residual as the ESKF. Apply proportional correction `kp * weight * residual` to angular rate and integrate a bounded bias term `ki * weight * residual * dt`. Use Task 2 quaternion integration and identical timestamp guards.

- [ ] **Step 4: Add nonfinite, recovery, and gain-boundary tests and verify GREEN**

Test zero/negative gains, weight transitions, rejected stale acceleration, and state rollback on nonfinite input. Run the complementary test and full suite. Commit `feat: add adaptive complementary benchmark estimator`.

---

### Task 7: Truth Simulation, Monte Carlo Comparison, and Log Replay

**Files:**
- Create: `tests/sim/imu_sim.h`
- Create: `tests/sim/imu_sim.c`
- Create: `tests/test_scenarios.c`
- Create: `tools/imu_replay.c`
- Create: `tools/imu_compare.c`
- Create: `tests/data/replay_smoke.csv`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 5 and 6 public estimators.
- Produces: deterministic sensor/truth sequences, scenario assertions, CSV replay output, and comparison metrics for Task 8 reporting.

- [ ] **Step 1: Write failing simulator self-tests**

Verify seeded repeatability, zero-noise static output, exact constant-rate truth, configured white-noise standard deviation over a long run, temperature bias polynomial injection, and fixed group-delay queues for 4.00 ms gyro and 5.27 ms acceleration paths.

- [ ] **Step 2: Verify RED, implement deterministic simulator, and verify GREEN**

Use an internal fixed-seed xorshift32 generator and Box-Muller normal samples. Generate truth at a configurable substep, then emit 485 Hz frames with delay queues, status/freshness flags, calibration errors, white noise, bias random walk, vibration, and linear acceleration.

- [ ] **Step 3: Write failing scenario tests with thresholds fixed before estimator runs**

Add named scenarios for static arbitrary attitude, compound rotation, 300 deg/s boundary motion, linear acceleration pulse, sustained turn, broadband vibration, shock, bias step, acceleration dropout, and recovery. Each scenario defines maximum roll/pitch RMS, peak error, discontinuity, and recovery time in its fixture before calling either estimator.

- [ ] **Step 4: Implement scenario runner and make ESKF scenarios GREEN**

Compute quaternion angular error plus roll/pitch/yaw components. Fail on NaN, covariance invalidity, threshold violation, or an unreported degraded mode. Keep yaw thresholds relative and never compare it to absolute north.

- [ ] **Step 5: Write failing ESKF-versus-complementary comparison test**

For at least 30 fixed seeds in the combined vibration-plus-bias-change scenario, calculate aggregate roll/pitch RMS and assert ESKF is lower than the complementary filter without more numerical failures or output discontinuities. Do not alter thresholds or seed selection after observing results; if the assertion fails, preserve the failure and report the complementary filter as the supported default candidate.

- [ ] **Step 6: Implement comparison metrics and Monte Carlo executable**

Output CSV columns for seed, estimator, roll RMS, pitch RMS, peak error, recovery time, bias error, NIS summary, numerical failures, CPU time, and static memory size. Return nonzero when acceptance criteria fail.

- [ ] **Step 7: Write failing replay parser tests, implement parser, and verify GREEN**

Define CSV columns explicitly: `timestamp_ns,gx,gy,gz,tgx,tgy,tgz,gstatus_x,gstatus_y,gstatus_z,gfresh_x,gfresh_y,gfresh_z,ax,ay,az,ta,astatus,afresh`. Reject missing columns, malformed numbers, nonmonotonic time, and nonfinite input with a line-numbered diagnostic. Process `tests/data/replay_smoke.csv` and emit timestamped quaternion/quality/health CSV.

- [ ] **Step 8: Run scenario, Monte Carlo, replay, and full-suite verification**

Run `ctest --test-dir build --output-on-failure`, `build/tools/imu_compare --seeds 30 --output build/comparison.csv`, and `build/tools/imu_replay tests/data/replay_smoke.csv build/replay_output.csv`. Commit `test: add IMU truth simulation comparison and replay`.

---

### Task 8: Zynq Integration Guide and Final Verification Report

**Files:**
- Create: `docs/integration-zynq.md`
- Create: `docs/verification-report.md`
- Modify: `CMakeLists.txt` only if a final verification target is required.

**Interfaces:**
- Consumes: all implementation and measured Task 7 outputs.
- Produces: integration instructions and an evidence-backed acceptance report.

- [ ] **Step 1: Write a documentation verification checklist before authoring**

The checklist must require the 485 Hz timer period, simultaneous three-CS latch recommendation, three-MISO option, ADXL355 `ODR_LPF=500 Hz/125 Hz`, `EXT_SYNC=10`, 485 Hz SYNC, 4.00/5.27 ms delays, SI units, FRD/NED signs, freshness semantics, every status flag, initialization sequence, and magnetometer-unavailable behavior.

- [ ] **Step 2: Write the Zynq integration guide against the public headers**

Include an exact pseudocode loop:

```c
on_485_hz_tick(timestamp_ns) {
    latch_three_adxrs453_cs_simultaneously();
    read_three_rate_responses(sample.gyro_rad_s, sample.gyro_status);
    read_synced_adxl355(sample.accel_m_s2, &sample.accel_status);
    sample.timestamp_ns = timestamp_ns;
    status = imu_process(&imu, &sample, &output);
    publish_only_if_status_allows(status, &output);
}
```

State explicitly that simultaneous register latch does not synchronize the ADXRS453 internal MEMS clocks.

- [ ] **Step 3: Perform fresh clean build and test verification**

Remove only the verified project-local `build` directory, configure from scratch, compile, and run the complete suite. Record compiler version, flags, test count, pass/fail count, and command outputs. Run the 30-seed comparison and replay smoke test again from the clean build.

- [ ] **Step 4: Add static and runtime safety verification**

Record `sizeof(imu_t)`, stack-heavy call sites, absence of production `malloc/calloc/realloc/free`, warning count, sanitizer results where supported, maximum observed quaternion norm error, minimum covariance eigenvalue/check surrogate, and worst simulation recovery time.

- [ ] **Step 5: Write the verification report using measured results only**

Report every acceptance criterion as PASS or FAIL with its producing command and numeric result. If ESKF does not outperform the complementary filter in the prescribed combined scenario, state that result and recommend the measured winner; do not weaken the criterion.

- [ ] **Step 6: Review spec coverage and commit documentation**

Map specification Sections 1-14 to code/tests/documentation, list any unmet requirement, run the documentation checklist, and commit `docs: add Zynq integration and verified IMU results` only when the report matches fresh outputs.

---

## Final Whole-Branch Review Gate

After Task 8, dispatch a fresh reviewer who did not implement any task. The reviewer must inspect the complete diff, rerun the clean build, full test suite, comparison executable, and replay smoke test, then check every global constraint and Review Focus item. Any finding returns to a new failing test before correction. Completion is claimed only after this independent review and a final fresh verification run by the primary agent.
