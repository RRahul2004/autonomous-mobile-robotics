import os
from math import exp
import matplotlib.pyplot as plt
from utilities import FileReader


# Logged runs of the point controller (goal at [1.0, 1.0])
RUNS = {
    "P": {
        "linear": "P-ctrl_point_linear.csv",
        "angular": "p_ctrl_point_angular.csv",
        "pose": "p_ctrl_point_robot_pose.csv",
    },
    "PID": {
        "linear": "PID_ctrl_point_linear.csv",
        "angular": "PID_ctrl_point_angular.csv",
        "pose": "PID_ctrl_point_ robot_pose.csv",
    },
}
GOAL = [1.0, 1.0]

# All four point-controller runs, for the comparison figure
ALL_POINT_RUNS = {
    "P": RUNS["P"],
    "PI": {
        "linear": "pi_ctrl_point_linear.csv",
        "angular": "pi_ctrl_point.csv",
    },
    "PD": {
        "linear": "pd_ctrl_linear.csv",
        "angular": "PD_ctrl_angular.csv",
    },
    "PID": RUNS["PID"],
}

# Logged runs of the final PID trajectory controller, with the reference curves from planner.py
TRAJECTORIES = {
    "parabola": {
        "linear": "PID_parabola_linear.csv",
        "angular": "PID_parabola_angular.csv",
        "pose": "PID_parabola_robot_pose.csv",
        "x_max": 1.5,
        "f": lambda x: x**2,
        "equation": "y = x²",
    },
    "sigmoid": {
        "linear": "PID_sigmoid_linear.csv",
        "angular": "PID_sigmoid_angular.csv",
        "pose": "PID_sigmoid_robot_pose.csv",
        "x_max": 2.5,
        "f": lambda x: 2 / (1 + exp(-2 * x)) - 1,
        "equation": "y = 2/(1+e^(-2x)) - 1",
    },
}

COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
MARKERS = ["o", "s", "^", "D"]
STYLES = ["-", "--", "-.", ":"]


def read_columns(filename):
    headers, values = FileReader(filename).read_file()
    return {h: [row[i] for row in values] for i, h in enumerate(headers)}


def series_style(i, n_points):
    # Distinct colour, line style and marker per series; markers thinned out so the line stays readable
    return dict(color=COLORS[i], linestyle=STYLES[i], marker=MARKERS[i], markersize=5,
                markevery=max(1, n_points // 15), linewidth=1.5)


def finish(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.4)
    ax.legend()


def plot_controller(name, files, out_dir):
    lin = read_columns(files["linear"])
    ang = read_columns(files["angular"])
    pose = read_columns(files["pose"])

    # One shared time origin so all plots of a run line up; stamps are in nanoseconds
    t0 = min(lin["stamp"][0], ang["stamp"][0], pose["stamp"][0])
    t_err = [(s - t0) / 1e9 for s in lin["stamp"]]
    t_pose = [(s - t0) / 1e9 for s in pose["stamp"]]

    # {e-t, edot-t}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for ax, data, kind, unit in [(axes[0], lin, "Linear", "m"), (axes[1], ang, "Angular", "rad")]:
        n = len(data["e"])
        ax.plot(t_err, data["e"], label=f"e [{unit}]", **series_style(0, n))
        ax.plot(t_err, data["e_dot"], label=f"ė [{unit}/s]", **series_style(1, n))
        finish(ax, f"{kind} error and error rate vs time", "time [s]", f"e [{unit}], ė [{unit}/s]")
    fig.suptitle(f"{name} point controller: error vs time")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"{name}_point_e_edot_t.png"), dpi=200)

    # {x-t, y-t, th-t}
    fig, ax = plt.subplots(figsize=(9, 5))
    n = len(pose["x"])
    ax.plot(t_pose, pose["x"], label="x [m]", **series_style(0, n))
    ax.plot(t_pose, pose["y"], label="y [m]", **series_style(1, n))
    ax.plot(t_pose, pose["theta"], label="θ [rad]", **series_style(2, n))
    finish(ax, f"{name} point controller: robot states vs time", "time [s]", "x, y [m] / θ [rad]")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"{name}_point_states_t.png"), dpi=200)

    # {x-y}
    fig, ax = plt.subplots(figsize=(6.5, 6))
    ax.plot(pose["x"], pose["y"], color=COLORS[0], linewidth=1.5, label="robot path")
    ax.plot(pose["x"][0], pose["y"][0], "o", color=COLORS[2], markersize=9, label="start")
    ax.plot(pose["x"][-1], pose["y"][-1], "s", color=COLORS[1], markersize=9, label="end")
    ax.plot(GOAL[0], GOAL[1], "x", color="#0b0b0b", markersize=11, markeredgewidth=2, label="goal")
    ax.set_aspect("equal", adjustable="datalim")
    finish(ax, f"{name} point controller: x-y path", "x [m]", "y [m]")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"{name}_point_xy.png"), dpi=200)

    # {e-edot}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for i, (ax, data, kind, unit) in enumerate([(axes[0], lin, "Linear", "m"), (axes[1], ang, "Angular", "rad")]):
        ax.plot(data["e"], data["e_dot"], label=f"{kind.lower()} error", **series_style(i, len(data["e"])))
        finish(ax, f"{kind} error phase plot (ė vs e)", f"e [{unit}]", f"ė [{unit}/s]")
    fig.suptitle(f"{name} point controller: error phase plots")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"{name}_point_e_edot.png"), dpi=200)

    plt.close("all")


def plot_point_comparison(out_dir):
    # Linear and angular error of every point controller, side by side
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    for i, (name, files) in enumerate(ALL_POINT_RUNS.items()):
        for ax, kind in zip(axes, ["linear", "angular"]):
            data = read_columns(files[kind])
            t = [(s - data["stamp"][0]) / 1e9 for s in data["stamp"]]
            ax.plot(t, data["e"], label=name, **series_style(i, len(t)))
    finish(axes[0], "Linear error vs time", "time [s]", "e [m]")
    finish(axes[1], "Angular error vs time", "time [s]", "e [rad]")
    fig.suptitle("Point controller (goal [1, 1]): P vs PI vs PD vs PID")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "point_controllers_errors_t.png"), dpi=200)
    plt.close(fig)


def plot_trajectories(out_dir):
    # {x-y} for both trajectories in one figure
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for ax, (name, traj) in zip(axes, TRAJECTORIES.items()):
        pose = read_columns(traj["pose"])
        xs = [traj["x_max"] * i / 500 for i in range(501)]
        ax.plot(xs, [traj["f"](x) for x in xs], color="#0b0b0b", linestyle="--", linewidth=1.5,
                label=f"reference: {traj['equation']}")
        ax.plot(pose["x"], pose["y"], color=COLORS[0], linewidth=2, label="robot path")
        ax.plot(pose["x"][0], pose["y"][0], "o", color=COLORS[2], markersize=9, label="start")
        ax.plot(pose["x"][-1], pose["y"][-1], "s", color=COLORS[1], markersize=9, label="end")
        ax.set_aspect("equal", adjustable="datalim")
        finish(ax, f"{name.capitalize()} trajectory", "x [m]", "y [m]")
    fig.suptitle("PID trajectory controller: x-y path vs reference")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "PID_trajectories_xy.png"), dpi=200)

    # Linear and angular error recorded during each trajectory
    for name, traj in TRAJECTORIES.items():
        lin = read_columns(traj["linear"])
        ang = read_columns(traj["angular"])
        t0 = lin["stamp"][0]
        t = [(s - t0) / 1e9 for s in lin["stamp"]]
        fig, ax = plt.subplots(figsize=(9, 5))
        n = len(t)
        ax.plot(t, lin["e"], label="linear error to final point [m]", **series_style(0, n))
        ax.plot(t, ang["e"], label="angular error to lookahead point [rad]", **series_style(1, n))
        finish(ax, f"PID {name} trajectory: linear and angular error vs time", "time [s]", "e [m] / e [rad]")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"PID_{name}_errors_t.png"), dpi=200)

    plt.close("all")


if __name__ == "__main__":
    out_dir = "plots"
    os.makedirs(out_dir, exist_ok=True)
    for name, files in RUNS.items():
        plot_controller(name, files, out_dir)
    plot_point_comparison(out_dir)
    plot_trajectories(out_dir)
    print("saved plots to", out_dir)
