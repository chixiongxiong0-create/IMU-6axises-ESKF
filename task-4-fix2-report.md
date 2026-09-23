# Task 4 review round 2: subnormal covariance normalization

## Root cause and single hypothesis

The covariance validator averaged symmetric entries as `0.5f*p + 0.5f*q`
before diagonal normalization. For `p == q == -FLT_TRUE_MIN`, each half rounds
to signed zero. With all six diagonals equal to `FLT_TRUE_MIN` and every
off-diagonal equal to its negative, normalization consequently produced the
identity instead of the true correlation matrix `2I - 11'`. That true matrix
has eigenvalues -4 and 2 (the latter repeated five times), so the lost entries
masked a materially indefinite covariance.

The hypothesis was that normalizing each entry before averaging would preserve
the dimensionless correlations without changing the existing PSD tolerance or
repair policy.

## RED before production changes

Added `test_subnormal_indefinite_covariance`, using `nextafterf(0, 1)` as the
C99-compatible smallest positive float. The independent double Jacobi oracle
confirms the fixture's normalized minimum eigenvalue is -4. The fixture requires
the public validator to reject it and verifies byte-identical rollback for
prediction, zero-weight and active gravity updates, zero-rate updates, and output.

```text
cmake --build build --target test_esekf
./build/test_esekf.exe psd
177 assertions, 8 failures
```

The failures showed acceptance by the public validator and all API paths,
including state mutation by prediction and output mutation by `get_output`.

## Minimal production fix

Each symmetric entry now divides sequentially by the two diagonal standard
deviations before multiplication by one half. Dividing by the smaller standard
deviation first avoids prematurely underflowing an entry when state scales
differ. Sequential divisions avoid constructing a scale product that could
underflow. The change retains float-only arithmetic, fixed storage, read-only
validation, bounded diagonal conditioning, and transactional API behavior.

## GREEN and full regression results

```text
cmake --build build --target test_esekf
./build/test_esekf.exe psd
177 assertions, 0 failures
./build/test_esekf.exe propagation
584118 assertions, 0 failures
cmake --build build
ctest --test-dir build --output-on-failure
6/6 test targets passed, 0 failures
```

The ten-minute propagation test retained its independent tolerance-aware
eigenvalue checks. Existing normal-range covariance fixtures, Joseph/reset
checks, gain limits, and rollback tests all passed unchanged.
