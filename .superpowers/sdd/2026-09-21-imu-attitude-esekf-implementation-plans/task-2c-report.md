# Task 2C Report: Optional Biquad Notch

## Scope

Implemented the fixed-storage, three-axis optional notch biquad in:

- `include/imu/imu_biquad.h`
- `src/imu_biquad.c`
- `tests/test_biquad.c`
- `CMakeLists.txt`

No Task 2A/2B implementation files were changed.

## TDD evidence

- Baseline: the existing suite passed 3/3 tests before Task 2C changes.
- RED: `cmake --build build --target test_biquad` failed at link time with
  undefined references to `imu_biquad_configure_notch`, `imu_biquad_reset`,
  and `imu_biquad_process`.
- GREEN: the focused biquad test passed 1/1 after the minimal implementation.

## Behaviors covered

- Bitwise-transparent disabled bypass, including signed zero and a NaN payload.
- Deterministic state reset and state reset on successful reconfiguration.
- Finite RBJ notch coefficients normalized by `a0`.
- Per-axis transposed direct-form II processing with two states per axis.
- Strict frequency range `(0, 0.49 * sample_rate)` and positive-Q validation.
- Transactional rejection of invalid configuration and nonfinite enabled input.
- A 100 Hz notch at 485 Hz strongly attenuates 100 Hz while preserving 10 Hz.

## Verification

- Task 2 trio: 3/3 passed (`math`, `calibration`, `biquad`).
- Full suite: 4/4 passed (`types`, `math`, `calibration`, `biquad`).
- Compiler warnings remain errors (`/W4 /WX` or
  `-Wall -Wextra -Wpedantic -Werror`).
- `git diff --check` passed.
