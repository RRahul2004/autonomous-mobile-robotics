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
# Read ordinary CSV files: IMU and Odometry
# --------------------------------------------------


def read_standard_csv(filename):
    with open(filename, "r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames:
            raise ValueError(f"No CSV headers found in {filename}")

        # Remove whitespace and normalize header names.
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
# Parse LiDAR ranges, including inf and nan
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
#
# The ranges field contains commas inside array('f',
# [...]), so normal csv.DictReader cannot parse it.
# --------------------------------------------------

def read_lidar_csv(filename):
    rows = []

    row_pattern = re.compile(
        r"^\s*(array\(\s*['\"]f['\"]\s*,\s*\[.*?\]\s*\))"
        r"\s*,\s*"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        r"\s*,\s*"
        r"([-+]?\d+)"
        r"\s*,?\s*$"
    )

    with open(filename, "r", encoding="utf-8-sig") as file:
        first_line = file.readline().strip()

        if not first_line:
            raise ValueError(f"Empty CSV file: {filename}")

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
# Plot IMU
# --------------------------------------------------

def plot_imu(filename, headers, rows):
    required = ["acc_x", "acc_y", "angular_z", "stamp"]

    if not all(key in headers for key in required):
        raise ValueError(
            f"IMU file must contain {required}. Found: {headers}"
        )

    time = elapsed_seconds(rows)

    acc_x = np.array([float(row["acc_x"]) for row in rows])
    acc_y = np.array([float(row["acc_y"]) for row in rows])
    angular_z = np.array([float(row["angular_z"]) for row in rows])

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(time, acc_x, label="Acceleration X")
    axes[0].plot(time, acc_y, label="Acceleration Y")
    axes[0].set_ylabel("Acceleration (m/s²)")
    axes[0].set_title("Linear Acceleration")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time, angular_z, label="Angular Velocity Z")
    axes[1].set_xlabel("Elapsed Time (s)")
    axes[1].set_ylabel("Angular Velocity (rad/s)")
    axes[1].set_title("Angular Velocity")
    axes[1].legend()
    axes[1].grid(True)

    fig.suptitle(get_plot_title("IMU", filename))
    fig.tight_layout()
    plt.show()


# --------------------------------------------------
# Plot Odometry
# --------------------------------------------------

def plot_odom(filename, headers, rows):
    required = ["x", "y", "th", "stamp"]

    if not all(key in headers for key in required):
        raise ValueError(
            f"Odometry file must contain {required}. Found: {headers}"
        )

    time = elapsed_seconds(rows)

    x = np.array([float(row["x"]) for row in rows])
    y = np.array([float(row["y"]) for row in rows])
    theta = np.array([float(row["th"]) for row in rows])

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))

    axes[0].plot(time, x, label="X Position")
    axes[0].plot(time, y, label="Y Position")
    axes[0].set_xlabel("Elapsed Time (s)")
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title("Position")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time, theta, label="Heading")
    axes[1].set_xlabel("Elapsed Time (s)")
    axes[1].set_ylabel("Heading (rad)")
    axes[1].set_title("Heading")
    axes[1].legend()
    axes[1].grid(True)

    fig.suptitle(get_plot_title("Odometry", filename))
    fig.tight_layout()
    plt.show()


# --------------------------------------------------
# Plot LiDAR
# --------------------------------------------------

def plot_lidar(filename):
    rows = read_lidar_csv(filename)

    # Plot the first scan in the file.
    scan = rows[0]

    ranges = scan["ranges"]
    angle_increment = scan["angle_increment"]

    # Replace invalid or infinite distances with zero.
    ranges = np.nan_to_num(
        ranges,
        nan=0.0,
        posinf=0.0,
        neginf=0.0
    )

    angles = np.arange(len(ranges)) * angle_increment

    # Convert polar coordinates to Cartesian coordinates.
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Do not plot zero-distance measurements.
    valid = ranges > 0

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.scatter(
    x[valid],
    y[valid],
    s=3,
    label="LiDAR Measurements"
)

    # Mark the robot's middle position (origin).
    ax.scatter(
        0,
        0,
        color="red",
        marker="x",
        s=100,
        label="Robot Position"
    )

    ax.legend(loc="best")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(get_plot_title("LiDAR", filename))
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)

    plt.show()


# --------------------------------------------------
# Detect sensor type and plot the file
# --------------------------------------------------

def plot_file(filename):
    print(f"Processing: {filename}")

    if not os.path.isfile(filename):
        print(f"Error: File not found: {filename}")
        return

    try:
        with open(
            filename, "r", encoding="utf-8-sig"
        ) as file:
            first_line = file.readline().strip().lower()

        if not first_line:
            print(f"Error: Empty file: {filename}")
            return

        if (
            "ranges" in first_line
            and "angle_increment" in first_line
            and "stamp" in first_line
        ):
            plot_lidar(filename)
            return

        headers, rows = read_standard_csv(filename)

        if not rows:
            print(f"Error: No data rows in {filename}")
            return

        headers = [header.strip().lower() for header in headers]

        if all(
            key in headers
            for key in ("acc_x", "acc_y", "angular_z", "stamp")
        ):
            plot_imu(filename, headers, rows)

        elif all(
            key in headers
            for key in ("x", "y", "th", "stamp")
        ):
            plot_odom(filename, headers, rows)

        else:
            print(f"Could not identify sensor type in {filename}")
            print(f"Found headers: {headers}")

    except (ValueError, KeyError, OSError) as error:
        print(f"Error processing {filename}: {error}")


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plot robot IMU, Odometry, and LiDAR CSV data."
    )

    parser.add_argument(
        "--files",
        nargs="+",
        required=True,
        help="One or more CSV files to plot."
    )

    args = parser.parse_args()

    for filename in args.files:
        plot_file(filename)


if __name__ == "__main__":
    main()
