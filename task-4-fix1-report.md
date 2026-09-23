# Task 4 review round 1: covariance PSD validation

## Root cause and hypothesis

`imu_esekf_covariance_valid` checked only finite entries, symmetry, and positive
diagonals. `state_valid` inherited that incomplete check. A covariance containing
the block `[1, 2; 2, 1]` therefore passed, although its eigenvalues are 3 and -1.
Prediction and Joseph/reset update completion used the same predicate, so small
negative modes introduced by float arithmetic could accumulate without a guard.

The single implementation hypothesis was that a float factorization of the
diagonally normalized covariance, with a fixed roundoff repair budget, would
distinguish corrupted covariance from benign near-semidefinite roundoff and stop
the latter from accumulating. A zero-tolerance factorization alone would reject
legitimate float propagation; diagonal/symmetry checks alone cannot certify PSD.

## Tests written before production changes (RED)

Added an independent **double Jacobi eigenvalue** oracle to the tests. It uses
neither the production validator nor its Cholesky algorithm, and checks the
correlation matrix so angle/bias units do not determine the error tolerance.

- A hand-derived positive-diagonal indefinite block with normalized eigenvalue
  -1 is tested at covariance scales `1e-12`, `1`, and `1e12`. Validation,
  prediction, gravity, zero-rate, and output must reject it without mutation.
- A near-semidefinite fixture has an off-diagonal correlation of
  `1 + 2*FLT_EPSILON`. The read-only validator accepts that roundoff-sized error;
  each committing prediction/measurement API must repair it. The oracle requires
  the resulting minimum eigenvalue to be at least `-1e-12`, and diagonal growth
  is bounded. Measurement variance scales with the fixture to preserve the
  existing innovation-conditioning guard.
- The 291,000-step (485 Hz, 600 seconds) propagation test checks independent
  eigenvalues every 1,000 steps and at the final step, using an acceptance
  tolerance of `8*FLT_EPSILON` in correlation coordinates.

Commands before production changes:

```text
cmake --build build --target test_esekf
./build/test_esekf.exe psd
  162 assertions, 30 failures
./build/test_esekf.exe propagation
  411946 assertions, 1 failure
  non-PSD covariance at step 205000:
  normalized minimum eigenvalue -1.92747747e-06
```

The initial near-semidefinite test used a fixed measurement variance even at
scale `1e12`; this separately tripped the existing innovation guard. Scaling
that variance corrected the fixture before any production edit. The remaining
30 failures were missing PSD rejection/rollback or missing roundoff repair.

## Single production change

The new fixed-storage, float-only validator scales by each diagonal standard
deviation and factors the resulting 6x6 correlation matrix. This congruence
preserves definiteness and makes small bias variances visible alongside larger
angle variances. It first checks pivots with an `8*FLT_EPSILON` margin. If that
fails, a single trial of `32*FLT_EPSILON` diagonal loading determines whether
the covariance is within the roundoff budget. There is no escalating jitter.

Materially indefinite matrices fail validation transactionally. For a repairable
candidate, the conditioner adds `32*FLT_EPSILON*P[i][i]` to each diagonal and
rechecks positive pivots with margin. It preserves off-diagonal terms. This
conditioning runs after prediction and after Joseph covariance plus error-state
reset, before the local candidate is committed. Failed checks leave the original
filter and output byte-identical. The public validator remains read-only.

The repair budget is dimensionless and relative to each variance, not a fixed
absolute jitter or a global maximum variance. Ordinary well-conditioned states
remain unchanged. The tolerance is a finite-precision PSD policy, not a claim of
exact real-arithmetic positive definiteness. Extremely ill-conditioned candidates
that cannot pass the bounded repair check are rejected rather than enlarged
without limit.

## GREEN and regression verification

```text
cmake --build build --target test_esekf
./build/test_esekf.exe psd
  162 assertions, 0 failures
./build/test_esekf.exe propagation
  584118 assertions, 0 failures
cmake --build build
ctest --test-dir build --output-on-failure
  6/6 test targets passed, 0 failures
```

The full suite includes existing Joseph covariance values, gain limiting,
gravity/yaw behavior, nonfinite/overflow guards, and byte-for-byte rollback tests.
The implementation uses no double arithmetic, dynamic allocation, or added
dependencies. Independent double precision is confined to the test oracle.
