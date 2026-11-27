"""
Plots for lab 4. Runs without ROS: it only reads the CSVs written by
decisions.py / localization.py and the map files (room.pgm / room.yaml).

  path_<n>_planned.csv   A* path for goal n                       (decisions.py)
  path_<n>_executed.csv  pose while driving to goal n, EKF + odom (decisions.py)
  robotPose.csv          IMU vs EKF output, same columns as lab 3 (localization.py)
  odom.csv               raw odometry, same columns as lab 3      (localization.py)

Usage (from labs/lab_4, after the run):
  python3 plot_lab4.py                 # every path_<n> found + sensor/EKF plots
  python3 plot_lab4.py --paths 1 3     # only goals 1 and 3
  python3 plot_lab4.py --no-sensors    # skip the lab 3 style sensor plots
  python3 plot_lab4.py --save plots    # also write PNGs into ./plots
"""

import argparse
import glob
import os
import re

import matplotlib.pyplot as plt
import numpy as np


PLANNED_COLOR = "#257de0"   # blue
EKF_COLOR = "#f47230"       # orange
ODOM_COLOR = "#00b894"      # aqua


# ---------------------------------------------------------------- file reading

def read_csv(filename):
    """Read a Logger CSV (trailing ', ' on every line). Returns headers, 2D array."""
    with open(filename) as f:
        headers = [h.strip() for h in f.readline().split(",") if h.strip()]
        rows = []
        for line in f:
            vals = [v.strip() for v in line.split(",") if v.strip()]
            if len(vals) == len(headers):
                rows.append([float(v) for v in vals])
    return headers, np.array(rows).reshape(-1, len(headers))


def column(headers, values, name):
    return values[:, headers.index(name)]


def seconds_since_start(stamps_ns):
    return (stamps_ns - stamps_ns[0]) / 1e9


def read_map(name="room"):
    """Load room.pgm + room.yaml. Returns the image and its extent in metres,
    using the same cell<->position convention as mapUtilities.cell_2_position."""
    with open(name + ".yaml") as f:
        text = f.read()
    res = float(re.search(r"resolution:\s*([-\d.]+)", text).group(1))
    origin = [float(v) for v in re.search(r"origin:\s*\[([^\]]*)\]", text).group(1).split(",")]

    with open(name + ".pgm", "rb") as f:
        assert f.readline().strip() == b"P5", "expected a binary PGM"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = map(int, line.split())
        f.readline()  # max value
        img = np.frombuffer(f.read(), dtype=np.uint8)[: width * height].reshape(height, width)

    o_x, o_y = origin[0], origin[1]
    extent = [o_x, o_x + width * res, o_y, o_y + height * res]
    return img, extent


# ---------------------------------------------------------------- path plots

def plot_path(n, map_img, extent, save_dir=None):
    planned_file = f"path_{n}_planned.csv"
    executed_file = f"path_{n}_executed.csv"

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(14, 6),
                                         gridspec_kw={"width_ratios": [1.3, 1]})
    fig.suptitle(f"Goal {n}: planned vs executed path")

    ax_map.imshow(map_img, cmap="gray", extent=extent, origin="upper", alpha=0.6)

    _, planned = read_csv(planned_file)
    if len(planned) == 0:
        print(f"goal {n}: {planned_file} is empty, skipping")
        plt.close(fig)
        return
    ax_map.plot(planned[:, 0], planned[:, 1], "--", color=PLANNED_COLOR, lw=2, zorder=3, label="A* planned path")
    ax_map.plot(planned[0, 0], planned[0, 1], "o", color="black", ms=8, label="start")
    ax_map.plot(planned[-1, 0], planned[-1, 1], "*", color="black", ms=14, label="goal")

    if os.path.exists(executed_file):
        h, ex = read_csv(executed_file)
        if len(ex):
            kf_x, kf_y = column(h, ex, "kf_x"), column(h, ex, "kf_y")
            od_x, od_y = column(h, ex, "odom_x"), column(h, ex, "odom_y")
            t = seconds_since_start(column(h, ex, "stamp"))

            ax_map.plot(kf_x, kf_y, color=EKF_COLOR, lw=2, label="executed (EKF)")
            ax_map.plot(od_x, od_y, color=ODOM_COLOR, lw=2, label="executed (odom)")

            # Distance from each executed pose to the nearest planned point
            def dist_to_path(x, y):
                d = np.hypot(x[:, None] - planned[None, :, 0], y[:, None] - planned[None, :, 1])
                return d.min(axis=1)

            ax_err.plot(t, dist_to_path(kf_x, kf_y), color=EKF_COLOR, lw=2, label="EKF")
            ax_err.plot(t, dist_to_path(od_x, od_y), color=ODOM_COLOR, lw=2, label="odom")
            ax_err.legend()

            final = np.hypot(kf_x[-1] - planned[-1, 0], kf_y[-1] - planned[-1, 1])
            print(f"goal {n}: {len(planned)} planned points, {len(ex)} executed samples, "
                  f"{t[-1]:.1f} s, final EKF distance to goal {final:.3f} m")
    else:
        print(f"goal {n}: no {executed_file}, plotting the planned path only")

    # Zoom to the paths with some margin instead of the whole map
    xs, ys = [planned[:, 0]], [planned[:, 1]]
    for line in ax_map.get_lines():
        xs.append(np.asarray(line.get_xdata())); ys.append(np.asarray(line.get_ydata()))
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    pad = 0.75
    ax_map.set_xlim(xs.min() - pad, xs.max() + pad)
    ax_map.set_ylim(ys.min() - pad, ys.max() + pad)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]"); ax_map.set_ylabel("y [m]")
    ax_map.legend(loc="best", fontsize=9)
    ax_map.grid(alpha=0.3)

    ax_err.set_title("distance from planned path")
    ax_err.set_xlabel("time [s]"); ax_err.set_ylabel("distance [m]")
    ax_err.grid(alpha=0.3)

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, f"path_{n}.png"), dpi=150)


def plot_all_paths(numbers, map_img, extent, save_dir=None):
    """Every goal on one map, so a multi-goal run can be seen at once."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(map_img, cmap="gray", extent=extent, origin="upper", alpha=0.6)
    for i, n in enumerate(numbers):
        _, planned = read_csv(f"path_{n}_planned.csv")
        ax.plot(planned[:, 0], planned[:, 1], "--", color=PLANNED_COLOR, lw=2, zorder=3,
                label="A* planned" if i == 0 else None)
        ax.annotate(str(n), planned[-1], textcoords="offset points", xytext=(6, 6))
        if os.path.exists(f"path_{n}_executed.csv"):
            h, ex = read_csv(f"path_{n}_executed.csv")
            if len(ex):
                ax.plot(column(h, ex, "kf_x"), column(h, ex, "kf_y"), color=EKF_COLOR, lw=2,
                        label="executed (EKF)" if i == 0 else None)
                ax.plot(column(h, ex, "odom_x"), column(h, ex, "odom_y"), color=ODOM_COLOR, lw=2,
                        label="executed (odom)" if i == 0 else None)
    ax.set_title("All goals")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend()
    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "all_paths.png"), dpi=150)


# ---------------------------------------------------------------- lab 3 style sensor plots

def plot_robot_pose(filename="robotPose.csv", save_dir=None):
    """Same layout as rahul/lab_3 plot_errors.py: state space, all states, ax, ay."""
    headers, values = read_csv(filename)
    t = seconds_since_start(values[:, -1])

    fig, axes = plt.subplots(2, 2, figsize=(14, 6))
    fig.suptitle(f"Sensor vs EKF ({filename})")

    axes[0, 0].plot(column(headers, values, "kf_x"), column(headers, values, "kf_y"))
    axes[0, 0].set_title("state space (EKF x, y)")
    axes[0, 0].grid()

    axes[1, 0].set_title("each individual state")
    for i in range(len(headers) - 1):
        axes[1, 0].plot(t, values[:, i], label=headers[i])
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid()

    axes[0, 1].plot(t, column(headers, values, "imu_ax"), label="imu_ax")
    axes[0, 1].plot(t, column(headers, values, "kf_ax"), label="kf_ax")
    axes[0, 1].set_title("ax")
    axes[0, 1].legend()
    axes[0, 1].grid()

    axes[1, 1].plot(t, column(headers, values, "imu_ay"), label="imu_ay")
    axes[1, 1].plot(t, column(headers, values, "kf_ay"), label="kf_ay")
    axes[1, 1].set_title("ay")
    axes[1, 1].legend()
    axes[1, 1].grid()

    for ax in axes[1]:
        ax.set_xlabel("time [s]")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "robotPose.png"), dpi=150)


def plot_odom(filename="odom.csv", save_dir=None):
    """Same layout as rahul/lab_3 odom(): state space and each state over time."""
    headers, values = read_csv(filename)
    t = seconds_since_start(values[:, -1])

    fig, axes = plt.subplots(2, 1, figsize=(14, 6))
    fig.suptitle(f"Odometry ({filename})")

    axes[0].plot(values[:, 0], values[:, 1])
    axes[0].set_title("state space")
    axes[0].grid()

    axes[1].set_title("each individual state")
    for i in range(len(headers) - 1):
        axes[1].plot(t, values[:, i], label=headers[i])
    axes[1].set_xlabel("time [s]")
    axes[1].legend()
    axes[1].grid()

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "odom.png"), dpi=150)


# ----------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot lab 4 planned/executed paths and EKF data")
    parser.add_argument("--paths", nargs="+", type=int, help="goal numbers to plot (default: all found)")
    parser.add_argument("--no-sensors", action="store_true", help="skip robotPose.csv / odom.csv plots")
    parser.add_argument("--map", default="room", help="map name without extension (default: room)")
    parser.add_argument("--save", metavar="DIR", help="also save PNGs into this directory")
    args = parser.parse_args()

    if args.save:
        os.makedirs(args.save, exist_ok=True)

    numbers = args.paths or sorted(int(re.search(r"path_(\d+)_planned", f).group(1))
                                   for f in glob.glob("path_*_planned.csv"))

    if numbers:
        map_img, extent = read_map(args.map)
        for n in numbers:
            plot_path(n, map_img, extent, args.save)
        if len(numbers) > 1:
            plot_all_paths(numbers, map_img, extent, args.save)
    else:
        print("no path_<n>_planned.csv files found")

    if not args.no_sensors:
        for name, fn in (("robotPose.csv", plot_robot_pose), ("odom.csv", plot_odom)):
            if os.path.exists(name):
                fn(name, args.save)
            else:
                print(f"{name} not found, skipping")

    plt.show()
