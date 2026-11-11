#!/usr/bin/env python3
"""Plot one simulated run: truth, raw odometry, filter estimate, tag fixes.

Usage: plot.py trace.csv out.png   (trace.csv.tags is read alongside)
"""
import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    with open(path) as f:
        r = csv.DictReader(f)
        cols = {k: [] for k in r.fieldnames}
        for row in r:
            for k, v in row.items():
                cols[k].append(float(v))
    return cols


def main(trace, out):
    d = read(trace)
    tags = read(trace + ".tags")
    err_raw = [math.hypot(a - b, c - e) for a, b, c, e in zip(d["ox"], d["tx"], d["oy"], d["ty"])]
    err_est = [math.hypot(a - b, c - e) for a, b, c, e in zip(d["ex"], d["tx"], d["ey"], d["ty"])]

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [1.1, 1]})
    ax.plot(d["tx"], d["ty"], "k-", lw=2, label="truth")
    ax.plot(d["ox"], d["oy"], "-", color="tab:red", lw=1.2, label="raw odometry")
    ax.plot(d["ex"], d["ey"], "-", color="tab:blue", lw=1.2, label="filter")
    good = [(x, y) for x, y, o in zip(tags["x"], tags["y"], tags["outlier"]) if not o]
    bad = [(x, y) for x, y, o in zip(tags["x"], tags["y"], tags["outlier"]) if o]
    if good:
        ax.scatter(*zip(*good), s=6, color="tab:green", label="AprilTag fix", zorder=3)
    if bad:
        ax.scatter(*zip(*bad), s=30, marker="x", color="magenta", label="outlier fix", zorder=3)
    ax.set_aspect("equal")
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")
    ax.set_title("Trajectory")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    ax2.plot(d["t"], err_raw, color="tab:red", label="raw odometry")
    ax2.plot(d["t"], err_est, color="tab:blue", label="filter")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("position error (in)")
    ax2.set_title("Position error vs truth")
    ax2.legend()
    ax2.grid(alpha=0.3)
    fig.suptitle("fieldpose, simulated run")
    fig.tight_layout()
    fig.savefig(out, dpi=130)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
