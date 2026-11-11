# Devlog

Short notes on what I tried, what broke and why.

## Day 1
- Set up CMake, Eigen, GoogleTest.
- Wrote the first test: a 1D constant-velocity robot with noisy position
  measurements. The filter must beat the raw measurements.

## Day 2: Kalman core
- Implemented `predict` and `update`. K is computed by solving `S K^T = H P`
  with an LDLT factorization instead of forming `S^-1` explicitly.
- Covariance uses the Joseph form `P = (I-KH) P (I-KH)^T + K R K^T`. The plain
  `(I-KH) P` is only correct for the exact optimal K; with rounding error it
  slowly loses symmetry and can produce slightly negative eigenvalues.
  The Joseph form is a sum of two PSD terms, so it stays PSD for any K.
  P is also symmetrised after each update.
- Added tests: huge R ignores the measurement, tiny R follows it, P stays
  symmetric and PSD over 2000 steps, predict-only grows the trace of P.
- Added GitHub Actions CI (gcc and clang on Ubuntu).
