# fieldpose

Robot pose estimation for an FTC robot, written in C++17.

A competition robot knows where it is from several imperfect sources:
wheel encoders drift, the IMU gives heading, and AprilTags give occasional
absolute fixes. `fieldpose` fuses them with a Kalman filter into a single
pose estimate on the 144 x 144 inch FTC field.

> Status: work in progress. Simulation-based; not tested on a physical robot.

## Planned

- [ ] Linear Kalman filter core (`KalmanFilter`)
- [ ] Motion model + wheel-odometry / IMU measurement models
- [ ] Asynchronous updates, outlier gating, dropped measurements
- [ ] Field simulator driven by trajectories from real match logs
- [ ] AprilTag absolute-pose updates
- [ ] Evaluation: position error (raw odometry vs filtered), time per update

## Build and test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Requires CMake 3.20+ and a C++17 compiler. Eigen and GoogleTest are fetched
automatically.

## Data and attribution

Field dimensions, encoder resolution and trajectories come from my FTC team's
robot and match logs. 

## Design notes and limitations

See `docs/DEVLOG.md`.

## License

MIT, see `LICENSE`.
