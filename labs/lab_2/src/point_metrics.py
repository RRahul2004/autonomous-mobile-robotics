import numpy as np
from utilities import FileReader
from plot_report import RUNS, GOAL


def read_array(filename):
    headers, values = FileReader(filename).read_file()
    return np.array(values, dtype=float)


def first_time(t, mask):
    return t[np.argmax(mask)]


def settling_time(t, outside):
    # time after which the signal never leaves the band again
    idx = np.where(outside)[0]
    return t[idx[-1] + 1] if len(idx) and idx[-1] + 1 < len(t) else np.nan


def metrics(files):
    pose = read_array(files["pose"])
    ang = read_array(files["angular"])
    goal = np.array(GOAL)

    t = (pose[:, 3] - pose[0, 3]) / 1e9
    start = pose[0, :2]
    dist0 = np.linalg.norm(goal - start)
    u = (goal - start) / dist0

    # Linear: normalised progress along the start->goal line (1 = at the goal), and distance to goal
    progress = (pose[:, :2] - start) @ u / dist0
    dist = np.linalg.norm(pose[:, :2] - goal, axis=1)

    lin = {
        "rise": first_time(t, progress >= 0.9) - first_time(t, progress >= 0.1),
        "settle": settling_time(t, dist > 0.02 * dist0),
        "overshoot": max(0.0, progress.max() - 1) * 100,
        "ess": dist[-1],
    }

    # Angular: only while the robot is > 5 cm from the goal, since the bearing to the goal is undefined at the goal
    t_ang = (ang[:, 3] - pose[0, 3]) / 1e9
    near = np.interp(t_ang, t, dist) > 0.05
    e = ang[near, 0]
    r = e / e[0]
    t_e = t_ang[near]

    angular = {
        "e0": e[0],
        "rise": first_time(t_e, r <= 0.1) - first_time(t_e, r <= 0.9),
        "settle": settling_time(t_e, np.abs(r) > 0.05),
        "overshoot": max(0.0, -r.min()) * 100,
        "ess": abs(e[-1]),
    }
    return dist0, lin, angular, t[-1]


def save_table_png(results, filename):
    import matplotlib.pyplot as plt

    p, pid = results["P"], results["PID"]
    # (quantity, metric, P value, PID value, which is better: "low" or None)
    rows = [
        ("Agility", "Linear rise time (10→90%)", p[1]["rise"], pid[1]["rise"], "{:.2f} s"),
        ("", "Linear settling time (2%)", p[1]["settle"], pid[1]["settle"], "{:.2f} s"),
        ("", "Angular rise time (90→10%)", p[2]["rise"], pid[2]["rise"], "{:.2f} s"),
        ("", "Angular settling time (5%)", p[2]["settle"], pid[2]["settle"], "{:.2f} s"),
        ("", "Total time to goal", p[3], pid[3], "{:.2f} s"),
        ("Accuracy", "Linear steady-state error", p[1]["ess"] * 1000, pid[1]["ess"] * 1000, "{:.1f} mm"),
        ("", "Angular steady-state error", p[2]["ess"], pid[2]["ess"], "{:.4f} rad"),
        ("Overshoot", "Linear %OS", p[1]["overshoot"], pid[1]["overshoot"], "{:.0f} %"),
        ("", "Angular %OS", p[2]["overshoot"], pid[2]["overshoot"], "{:.0f} %"),
    ]

    cells, bold = [], []
    for quantity, metric, a, b, fmt in rows:
        cells.append([quantity, metric, fmt.format(a), fmt.format(b)])
        # lower is better for every metric; mark the winner unless it's a tie
        bold.append(None if fmt.format(a) == fmt.format(b) else (2 if a < b else 3))
    cells.append(["Initial conditions", "Start distance / initial heading error",
                  f"{p[0]:.3f} m / {p[2]['e0']:.2f} rad", f"{pid[0]:.3f} m / {pid[2]['e0']:.2f} rad"])
    bold.append(None)

    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.axis("off")
    table = ax.table(cellText=cells, colLabels=["Quantity", "Metric", "P", "PID"],
                     colWidths=[0.17, 0.43, 0.2, 0.2], loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(10.5)
    table.scale(1, 1.6)

    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#c3c2b7")
        if r == 0:
            cell.set_facecolor("#2a78d6")
            cell.set_text_props(color="white", weight="bold")
        elif r == len(cells):
            cell.set_facecolor("#f0efec")
            cell.set_text_props(style="italic")
        else:
            if c == 0:
                cell.set_text_props(weight="bold")
            if c == 1:
                cell.set_text_props(ha="left")
                cell._loc = "left"
            if bold[r - 1] == c:
                cell.set_facecolor("#e3f4ec")
                cell.set_text_props(weight="bold")

    ax.set_title("P vs PID point controller: agility, accuracy and overshoot", fontsize=13, pad=12)
    fig.text(0.5, 0.03, "Better value (lower) highlighted in green. Angular metrics use samples > 5 cm from the goal.",
             ha="center", fontsize=9, color="#52514e")
    fig.tight_layout()
    fig.savefig(filename, dpi=200, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    results = {}
    for name, files in RUNS.items():
        dist0, lin, angular, total = metrics(files)
        results[name] = (dist0, lin, angular, total)
        print(f"== {name}  (start distance {dist0:.3f} m, initial heading error {angular['e0']:.3f} rad, run time {total:.2f} s)")
        print(f"  linear : rise(10-90%) {lin['rise']:.2f} s, settling(2%) {lin['settle']:.2f} s, "
              f"overshoot {lin['overshoot']:.2f} %, steady-state error {lin['ess'] * 1000:.1f} mm")
        print(f"  angular: rise(90-10%) {angular['rise']:.2f} s, settling(5%) {angular['settle']:.2f} s, "
              f"overshoot {angular['overshoot']:.2f} %, steady-state error {angular['ess']:.4f} rad")

    save_table_png(results, "plots/P_vs_PID_point_table.png")
    print("saved plots/P_vs_PID_point_table.png")
