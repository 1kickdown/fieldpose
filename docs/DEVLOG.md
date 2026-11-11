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

## Day 3: model, simulator, fusion, evaluation
- **Model choice:** state `[x, y, theta]` with odometry as control input, not
  `[x, y, theta, vx, vy, omega]`. Wheel odometry already measures increments,
  so velocity states add nothing without a force/acceleration model. Heading
  makes the field-frame increment nonlinear; I rotate with the current heading
  estimate (a "linearised control input" shortcut). A full EKF is left as an
  optional extension.
- **Angles:** the state heading is kept unwrapped; measurements are unwrapped
  next to it before the update. Otherwise a 179 -> -179 deg jump looks like a
  358 deg innovation (test `HeadingWrapAroundIsHandled`).
- **Logs:** only `start_Info: Path=` lines are in the absolute field frame. The
  `Path=` inside `StateInfo` events is relative to the robot at that moment, so
  it is ignored. Tag `absPose` angles come unnormalised (632.8 deg), so the
  loader wraps them. Each Auto log gives 12 waypoints.
- **Result that surprised me:** on the real-log trajectories raw odometry
  drifts only about 0.5 in, because they are short. The filter gain shows up
  mostly on the long synthetic scenarios. Without gating, tag fixes did not
  improve RMSE over odometry+IMU (0.57 vs 0.57 in) because 3 % outliers cancel
  the benefit; with gating RMSE drops to 0.41 in.
- Gate rejected all 390 simulated outliers and 8 of about 13,000 good fixes.
- **Recovery metric:** time after a tag blackout until error < 1 in is almost
  always 0, so that metric is weak. Recovery after an outlier is more telling.
- Ran the tests under ASan + UBSan: clean.
- Benchmark: about 0.6 us (predict + IMU) and 1.0 us (with tag) per tick on
  M1 Pro, with dynamic-size Eigen.
- Not done: EKF, TTC detector, clang-tidy.
