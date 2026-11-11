#include <benchmark/benchmark.h>

#include "fieldpose/pose_fusion.hpp"

using namespace fieldpose;

// One control-loop tick: odometry predict + IMU update.
static void BM_PredictPlusImuUpdate(benchmark::State& state) {
  PoseFusion f({0, 0, 0}, FusionConfig{});
  for (auto _ : state) {
    f.predictOdometry(0.3, 0.05, 0.002);
    f.updateImu(0.001);
    benchmark::DoNotOptimize(f.pose());
  }
}
BENCHMARK(BM_PredictPlusImuUpdate);

// Tick that also carries an AprilTag fix (3-dof update with gating).
static void BM_PredictImuAndTagUpdate(benchmark::State& state) {
  PoseFusion f({0, 0, 0}, FusionConfig{});
  for (auto _ : state) {
    f.predictOdometry(0.3, 0.05, 0.002);
    f.updateImu(0.001);
    f.updateTag(0.1, 0.1, 0.0);
    benchmark::DoNotOptimize(f.pose());
  }
}
BENCHMARK(BM_PredictImuAndTagUpdate);

BENCHMARK_MAIN();
