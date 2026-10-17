
import argparse
import csv
import os
import re

import matplotlib.pyplot as plt
import numpy as np


# --------------------------------------------------
# Plot title generation
# --------------------------------------------------

def get_plot_title(sensor, filename):
    name = os.path.basename(filename).lower()

    motions = {
        "circle": "Circle",
        "line": "Line",
        "straight": "Straight",
        "spiral": "Spiral",
    }

    motion = "Unknown Motion"

    for keyword, label in motions.items():
        if keyword in name:
            motion = label
            break

    return f"Robot {sensor}: {motion}"


# --------------------------------------------------
# Read IMU and odometry CSV files
# --------------------------------------------------

def read_standard_csv(filename):
    with open(filename, "r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames:
            raise ValueError(f"No CSV headers found in {filename}")

        reader.fieldnames = [
            header.strip().lower() if header else ""
            for header in reader.fieldnames
        ]

        rows = []

        for row in reader:
            cleaned_row = {
                key.strip().lower(): value.strip()
                for key, value in row.items()
                if key is not None and value is not None
            }
            rows.append(cleaned_row)

        return reader.fieldnames, rows


# --------------------------------------------------
# Parse LiDAR ranges
# --------------------------------------------------

def parse_ranges(array_text):
    start = array_text.find("[")
    end = array_text.rfind("]")

    if start == -1 or end == -1:
        raise ValueError("Could not parse LiDAR ranges array.")

    values_text = array_text[start + 1:end]
    values = []

    for value in values_text.split(","):
        value = value.strip()

        if not value:
            continue

        if value.lower() in ("inf", "+inf", "infinity", "+infinity"):
            values.append(float("inf"))
        elif value.lower() in ("-inf", "-infinity"):
            values.append(float("-inf"))
        elif value.lower() == "nan":
            values.append(float("nan"))
        else:
            values.append(float(value))

    return np.array(values, dtype=float)


# --------------------------------------------------
# Read LiDAR CSV files
# Supports array('f', [...]),angle_increment,stamp
# --------------------------------------------------

def read_lidar_csv(filename):
    rows = []

    row_pattern = re.compile(
        r"""^\s*(array\(\s*['"]f['"]\s*,\s*\[.*?\]\s*\))"""
        r"\s*,\s*"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        r"\s*,\s*"
        r"([-+]?\d+)"
        r"\s*,?\s*$"
    )

    with open(filename, "r", encoding="utf-8-sig") as file:
        first_line = file.readline().strip()

        if not first_line:
            raise ValueError(f"Empty LiDAR file: {filename}")

        headers = [
            item.strip().lower()
            for item in first_line.split(",")
        ]

        if not all(
            key in headers
            for key in ("ranges", "angle_increment", "stamp")
        ):
            raise ValueError(
                f"Unexpected LiDAR CSV headers in {filename}: {headers}"
            )

        for line_number, line in enumerate(file, start=2):
            line = line.strip()

            if not line:
                continue

            match = row_pattern.match(line)

            if not match:
                print(
                    f"Warning: Skipping unrecognized LiDAR row "
                    f"{line_number} in {filename}"
                )
                continue

            ranges_text, angle_increment, stamp = match.groups()

            rows.append({
                "ranges": parse_ranges(ranges_text),
                "angle_increment": float(angle_increment),
                "stamp": int(stamp),
            })

    if not rows:
        raise ValueError(f"No valid LiDAR scans found in {filename}")

    return rows


# --------------------------------------------------
# Convert timestamps to elapsed seconds
# --------------------------------------------------

def elapsed_seconds(rows, stamp_key="stamp"):
    stamps = np.array(
        [float(row[stamp_key]) for row in rows],
        dtype=float
    )

    return (stamps - stamps[0]) * 1e-9


# --------------------------------------------------
# Plot all sensors in one figure: 2 rows x 3 columns
# --------------------------------------------------

def plot_all(imu_file, odom_file, lidar_file):

    # Read CSV files
    imu_headers, imu_rows = read_standard_csv(imu_file)
    odom_headers, odom_rows = read_standard_csv(odom_file)
    lidar_rows = read_lidar_csv(lidar_file)

    if not imu_rows:
        raise ValueError("IMU file contains no data.")

    if not odom_rows:
        raise ValueError("Odometry file contains no data.")

    # Validate required columns
    imu_required = ["acc_x", "acc_y", "angular_z", "stamp"]
    odom_required = ["x", "y", "th", "stamp"]

    if not all(key in imu_headers for key in imu_required):
        raise ValueError(
            f"IMU file must contain {imu_required}. "
            f"Found: {imu_headers}"
        )

    if not all(key in odom_headers for key in odom_required):
        raise ValueError(
            f"Odometry file must contain {odom_required}. "
            f"Found: {odom_headers}"
        )

    # ----------------------------------------------
    # Extract IMU data
    # ----------------------------------------------

    imu_time = elapsed_seconds(imu_rows)

    acc_x = np.array([float(row["acc_x"]) for row in imu_rows])
    acc_y = np.array([float(row["acc_y"]) for row in imu_rows])
    angular_z = np.array(
        [float(row["angular_z"]) for row in imu_rows]
    )

    # ----------------------------------------------
    # Extract odometry data
    # ----------------------------------------------

    odom_time = elapsed_seconds(odom_rows)

    x = np.array([float(row["x"]) for row in odom_rows])
    y = np.array([float(row["y"]) for row in odom_rows])
    theta = np.array([float(row["th"]) for row in odom_rows])

    # ----------------------------------------------
    # Extract the first LiDAR scan
    # ----------------------------------------------

    scan = lidar_rows[0]

    ranges = scan["ranges"].copy()
    angle_increment = scan["angle_increment"]

    # Ignore invalid, infinite, and non-positive ranges
    ranges[~np.isfinite(ranges)] = np.nan
    ranges[ranges <= 0] = np.nan

    angles = np.arange(len(ranges)) * angle_increment

    lidar_x = ranges * np.cos(angles)
    lidar_y = ranges * np.sin(angles)

    valid = np.isfinite(lidar_x) & np.isfinite(lidar_y)

    # ----------------------------------------------
    # Create ONE figure: 2 rows x 3 columns
    # ----------------------------------------------

    fig, axes = plt.subplots(
        3, 2,
        figsize=(18, 9)
    )

    fig.suptitle(
        get_plot_title("Sensor Data", imu_file),
        fontsize=16
    )

    # ----------------------------------------------
    # 1. Robot trajectory (top-left)
    # ----------------------------------------------

    ax = axes[0, 0]

    ax.plot(x, y, label="Robot Trajectory")
    ax.scatter(x[0], y[0], marker="o", label="Start")
    ax.scatter(x[-1], y[-1], marker="x", label="End")

    ax.set_title("Robot Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    ax.grid(True)

    # ----------------------------------------------
    # 2. LiDAR scan (top-middle)
    # ----------------------------------------------

    ax = axes[0, 1]

    ax.scatter(
        lidar_x[valid],
        lidar_y[valid],
        s=3,
        label="LiDAR Measurements"
    )

    ax.scatter(
        0, 0,
        marker="x",
        s=100,
        label="Robot Position"
    )

    ax.set_title("LiDAR 2D Scan (First Scan)")
    ax.set_xlabel("X Distance (m)")
    ax.set_ylabel("Y Distance (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best")
    ax.grid(True)

    # ----------------------------------------------
    # 3. Linear acceleration (top-right)
    # ----------------------------------------------

    ax = axes[1, 0]

    ax.plot(imu_time, acc_x, label="Acceleration X")
    ax.plot(imu_time, acc_y, label="Acceleration Y")

    ax.set_title("IMU: Linear Acceleration")
    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.legend()
    ax.grid(True)

    # ----------------------------------------------
    # 4. Angular velocity (bottom-left)
    # ----------------------------------------------

    ax = axes[1, 1]

    ax.plot(
        imu_time,
        angular_z,
        label="Angular Velocity Z"
    )

    ax.set_title("IMU: Angular Velocity")
    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Angular Velocity (rad/s)")
    ax.legend()
    ax.grid(True)

    # ----------------------------------------------
    # 5. Position versus time (bottom-middle)
    # ----------------------------------------------

    ax = axes[2, 0]

    ax.plot(odom_time, x, label="X Position")
    ax.plot(odom_time, y, label="Y Position")

    ax.set_title("Odometry: Position vs Time")
    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend()
    ax.grid(True)

    # ----------------------------------------------
    # 6. Heading versus time (bottom-right)
    # ----------------------------------------------

    ax = axes[2, 1]

    ax.plot(odom_time, theta, label="Heading")

    ax.set_title("Odometry: Heading vs Time")
    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Heading (rad)")
    ax.legend()
    ax.grid(True)

    # Improve spacing and display one window
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot IMU, odometry, and LiDAR CSV files "
            "in six subplots."
        )
    )

    parser.add_argument(
        "--files",
        nargs=3,
        required=True,
        metavar=("IMU_CSV", "ODOM_CSV", "LIDAR_CSV"),
        help="Provide IMU, odometry, and LiDAR CSV files in that order."
    )

    args = parser.parse_args()

    imu_file, odom_file, lidar_file = args.files

    for filename in args.files:
        if not os.path.isfile(filename):
            parser.error(f"File not found: {filename}")

    try:
        plot_all(imu_file, odom_file, lidar_file)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
