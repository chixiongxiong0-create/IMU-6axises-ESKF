# Task 2 Review Fix Round 1

## Matrix rejection contract

- RED: `cmake --build build --target test_math` failed with `invalid use of
  void expression` after tests required boolean success/failure results.
- GREEN: `cmake --build build --target test_math; ctest --test-dir build -R
  '^math$' --output-on-failure` passed 1/1.
- `imu_mat6_transpose()` and `imu_mat6_symmetrize()` now return `false` for
  null or nonfinite input before mutating caller storage, and `true` after a
  successful transactional copy.
- Regression assertions inject NaN and use `memcmp` to prove the transpose
  destination and symmetrize in/out matrix are byte-identical after rejection.

## Platform math linkage

- Added the `imu_platform_math` interface target, which propagates `m` on
  `UNIX AND NOT APPLE` and is linked by the math, calibration, and biquad test
  targets that compile C math users.
- `cmake -S . -B build; cmake --build build --target test_math
  test_calibration test_biquad; ctest --test-dir build -R
  '^(math|calibration|biquad)$' --output-on-failure` passed 3/3.

## Final verification

- `cmake --build build; ctest --test-dir build --output-on-failure` passed
  4/4 (`types`, `math`, `calibration`, `biquad`).
- `git diff --check` passed; Git emitted only existing LF-to-CRLF working-copy
  warnings.
