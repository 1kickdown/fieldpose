// Runs every scenario over many seeds and prints raw-odometry vs filter error.
//   evaluate <trajectory_dir> [--seeds N] [--trace <scenario> <seed> <out.csv>]
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <string>

#include "fieldpose/evaluation.hpp"
#include "fieldpose/log_loader.hpp"

using namespace fieldpose;
namespace fs = std::filesystem;

namespace {

struct Scenario {
  std::string name;
  Trajectory traj;
};

std::vector<Scenario> load_scenarios(const fs::path& dir) {
  std::vector<Scenario> out;
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(dir))
    if (e.path().extension() == ".csv") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  for (const auto& f : files)
    out.push_back({f.stem().string(), Trajectory(read_waypoints_csv(f))});
  // Synthetic scenarios: longer drives where odometry drift accumulates.
  out.push_back({"synth_loop",
                 Trajectory({{-50, -50, 0}, {55, -50, 0}, {55, 50, 1.57}, {-50, 50, 3.14},
                             {-50, -50, -1.57}, {55, -50, 0}, {55, 0, 1.57}})});
  out.push_back({"synth_arc", Trajectory::arc(0, 0, 55, -kPi / 2, 1.5 * kPi)});
  out.push_back({"synth_spin_drive",
                 Trajectory({{0, 0, 0}, {0, 0, 3.0}, {60, 0, 3.0}, {60, 0, -3.0}, {60, 50, -3.0}})});
  return out;
}

struct Acc {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double mean() const {
    return v.empty() ? 0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  }
};

struct Row {
  Acc pos, theta, below, final_err;
};

void write_trace(const fs::path& out, const SimResult& sim, const FusionRun& run) {
  std::ofstream f(out);
  f << "t,tx,ty,tth,ox,oy,oth,ex,ey,eth,tag\n";
  for (size_t i = 0; i < sim.truth.size(); ++i)
    f << i * sim.dt << ',' << sim.truth[i].x << ',' << sim.truth[i].y << ','
      << sim.truth[i].theta << ',' << sim.raw_odometry[i].x << ',' << sim.raw_odometry[i].y
      << ',' << wrap_angle(sim.raw_odometry[i].theta) << ',' << run.estimate[i].x << ','
      << run.estimate[i].y << ',' << run.estimate[i].theta << ',' << sim.tag_visible[i]
      << '\n';
  std::ofstream g(out.string() + ".tags");
  g << "t,x,y,outlier\n";
  for (const auto& e : sim.events)
    if (e.type == SimEvent::Type::Tag) g << e.t << ',' << e.a << ',' << e.b << ',' << e.outlier << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: evaluate <trajectory_dir> [--seeds N] [--trace name seed out.csv]\n";
    return 2;
  }
  int seeds = 50;
  std::string trace_name, trace_out;
  uint32_t trace_seed = 1;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc) seeds = std::stoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--trace") && i + 3 < argc) {
      trace_name = argv[++i];
      trace_seed = std::stoul(argv[++i]);
      trace_out = argv[++i];
    }
  }

  const double kThr = 2.0;  // inches
  const auto scenarios = load_scenarios(argv[1]);
  const SimConfig sim_cfg;
  SimConfig no_tag_cfg = sim_cfg;
  no_tag_cfg.tag_enabled = false;
  FusionConfig gated, ungated;
  ungated.gating = false;

  const char* names[] = {"raw odometry", "filter: odometry+IMU", "filter: +AprilTag, no gating",
                         "filter: +AprilTag, gating"};
  std::map<std::string, std::array<Row, 4>> per_scn;
  std::array<Row, 4> total;
  Acc recovery, out_rec_gated, out_rec_ungated;
  const double kRecThr = 1.0;  // inches
  int outliers_total = 0, outliers_rejected = 0, good_total = 0, good_rejected = 0;

  for (const auto& sc : scenarios) {
    for (int s = 1; s <= seeds; ++s) {
      const SimResult sim = simulate(sc.traj, sim_cfg, s);
      const SimResult sim_nt = simulate(sc.traj, no_tag_cfg, s);
      const FusionRun r_imu = run_fusion(sim_nt, gated);
      const FusionRun r_ung = run_fusion(sim, ungated);
      const FusionRun r_gat = run_fusion(sim, gated);

      const ErrorStats st[4] = {compute_stats(sim.raw_odometry, sim.truth, kThr),
                                compute_stats(r_imu.estimate, sim_nt.truth, kThr),
                                compute_stats(r_ung.estimate, sim.truth, kThr),
                                compute_stats(r_gat.estimate, sim.truth, kThr)};
      for (int k = 0; k < 4; ++k)
        for (Row* row : {&per_scn[sc.name][k], &total[k]}) {
          row->pos.add(st[k].pos_rmse);
          row->theta.add(st[k].theta_rmse_deg);
          row->below.add(st[k].frac_below);
          row->final_err.add(st[k].final_pos_err);
        }
      for (double t : recovery_times(sim, r_gat, kRecThr, 3.0)) recovery.add(t);
      for (double t : outlier_recovery_times(sim, r_gat, kRecThr)) out_rec_gated.add(t);
      for (double t : outlier_recovery_times(sim, r_ung, kRecThr)) out_rec_ungated.add(t);
      outliers_total += r_gat.accepted_outliers + r_gat.rejected_outliers;
      outliers_rejected += r_gat.rejected_outliers;
      good_total += (r_gat.accepted_tags + r_gat.rejected_tags) -
                    (r_gat.accepted_outliers + r_gat.rejected_outliers);
      good_rejected += r_gat.rejected_tags - r_gat.rejected_outliers;

      if (sc.name == trace_name && static_cast<uint32_t>(s) == trace_seed)
        write_trace(trace_out, sim, r_gat);
    }
  }

  std::printf("Scenarios: %zu, seeds per scenario: %d, runs: %zu\n", scenarios.size(), seeds,
              scenarios.size() * seeds);
  std::printf("Position error threshold: %.1f in\n\n", kThr);
  std::printf("| Method | Position RMSE (in) | Heading RMSE (deg) | Time under %.0f in | Final error (in) |\n", kThr);
  std::printf("|---|---|---|---|---|\n");
  for (int k = 0; k < 4; ++k)
    std::printf("| %s | %.2f | %.2f | %.0f%% | %.2f |\n", names[k], total[k].pos.mean(),
                total[k].theta.mean(), 100 * total[k].below.mean(), total[k].final_err.mean());

  std::printf("\nPer scenario, mean position RMSE (in): raw / odometry+IMU / +tag no gate / +tag gate\n");
  for (const auto& [name, rows] : per_scn)
    std::printf("  %-24s %6.2f / %6.2f / %6.2f / %6.2f\n", name.c_str(), rows[0].pos.mean(),
                rows[1].pos.mean(), rows[2].pos.mean(), rows[3].pos.mean());

  std::printf("\nGate: rejected %d of %d true outliers, and %d of %d good tag fixes.\n",
              outliers_rejected, outliers_total, good_rejected, good_total);
  auto report = [&](const char* what, const Acc& a) {
    if (a.v.empty()) return;
    auto v = a.v;
    std::sort(v.begin(), v.end());
    const size_t over = std::count_if(v.begin(), v.end(), [](double t) { return t > 0.0; });
    std::printf("%s: %zu events, %zu pushed above %.1f in; mean %.2f s, median %.2f s, max %.2f s\n",
                what, v.size(), over, kRecThr, a.mean(), v[v.size() / 2], v.back());
  };
  std::printf("\nRecovery time to < %.1f in:\n", kRecThr);
  report("  after tag blackout (>=3 s), gated   ", recovery);
  report("  after an outlier tag fix, no gating ", out_rec_ungated);
  report("  after an outlier tag fix, gating    ", out_rec_gated);
}
