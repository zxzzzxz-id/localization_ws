#!/usr/bin/env python3
"""Estimate stationary gyro bias and acceleration residual for two IMUs."""

import argparse
import json
import os
import warnings
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/dual_imu_recorder_matplotlib")
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


IMU_TOPICS = {"imu1": "/imu1/imu/data", "imu2": "/imu2/imu/data"}
GRAVITY_M_S2 = 9.80665


def message_time_seconds(message, bag_time_ns):
    stamp = message.header.stamp
    if stamp.sec or stamp.nanosec:
        return stamp.sec + stamp.nanosec * 1e-9
    return bag_time_ns * 1e-9


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    message_types = {item.name for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in IMU_TOPICS.values() if topic not in message_types]
    if missing:
        raise RuntimeError("Bag is missing required topics: " + ", ".join(missing))

    samples = {name: [] for name in IMU_TOPICS}
    topic_to_name = {topic: name for name, topic in IMU_TOPICS.items()}
    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        if topic in topic_to_name:
            message = deserialize_message(raw, Imu)
            samples[topic_to_name[topic]].append(
                {
                    "time": message_time_seconds(message, bag_time_ns),
                    "frame_id": message.header.frame_id,
                    "acceleration": [
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ],
                    "angular_velocity": [
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ],
                    "orientation_wxyz": [
                        message.orientation.w,
                        message.orientation.x,
                        message.orientation.y,
                        message.orientation.z,
                    ],
                    "orientation_valid": message.orientation_covariance[0] != -1.0,
                }
            )
    return samples


def as_arrays(samples):
    times = np.asarray([sample["time"] for sample in samples], dtype=float)
    acceleration = np.asarray([sample["acceleration"] for sample in samples], dtype=float)
    angular_velocity = np.asarray(
        [sample["angular_velocity"] for sample in samples], dtype=float
    )
    return times, acceleration, angular_velocity


def finite_list(values):
    return [float(value) if np.isfinite(value) else None for value in values]


def rotate_body_to_enu(quaternion_wxyz, vector_body):
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm == 0.0:
        return np.full(3, np.nan)
    w, x, y, z = quaternion / norm
    vector = np.asarray(vector_body, dtype=float)
    vector_quaternion = np.array([x, y, z])
    return vector + 2.0 * (
        w * np.cross(vector_quaternion, vector)
        + np.cross(vector_quaternion, np.cross(vector_quaternion, vector))
    )


def yaw_from_body_to_enu_quaternion(quaternion_wxyz):
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm == 0.0:
        return np.nan
    w, x, y, z = quaternion / norm
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stationary_bias_summary(samples, max_gyro_rad_s, gravity_tolerance_m_s2):
    times, acceleration, angular_velocity = as_arrays(samples)
    if len(times) < 2:
        raise RuntimeError("Each IMU topic needs at least two messages")
    gyro_norm = np.linalg.norm(angular_velocity, axis=1)
    acceleration_norm = np.linalg.norm(acceleration, axis=1)
    stationary = (
        (gyro_norm <= max_gyro_rad_s)
        & (np.abs(acceleration_norm - GRAVITY_M_S2) <= gravity_tolerance_m_s2)
    )
    if np.count_nonzero(stationary) < 100:
        raise RuntimeError(
            "Fewer than 100 stationary samples. Keep both IMUs still, or relax the thresholds."
        )
    stationary_acceleration = acceleration[stationary]
    stationary_gyro = angular_velocity[stationary]
    result = {
        "message_count": int(len(times)),
        "frame_ids": sorted({sample["frame_id"] for sample in samples}),
        "stationary_selection": {
            "sample_count": int(np.count_nonzero(stationary)),
            "fraction": float(np.mean(stationary)),
            "max_gyro_rad_s": max_gyro_rad_s,
            "gravity_tolerance_m_s2": gravity_tolerance_m_s2,
        },
        "gyro_bias_candidate_rad_s": {
            "mean_xyz": finite_list(np.mean(stationary_gyro, axis=0)),
            "std_xyz": finite_list(np.std(stationary_gyro, axis=0)),
        },
        "acceleration_body_mean_m_s2": finite_list(
            np.mean(stationary_acceleration, axis=0)
        ),
    }
    orientation_valid = np.asarray(
        [sample["orientation_valid"] for sample in samples], dtype=bool
    )
    if np.all(orientation_valid[stationary]):
        orientation = [sample["orientation_wxyz"] for sample in samples]
        acceleration_enu = np.asarray(
            [
                rotate_body_to_enu(quaternion, value)
                for quaternion, value in zip(orientation, acceleration)
            ]
        )
        residual = acceleration_enu[stationary] - np.array([0.0, 0.0, GRAVITY_M_S2])
        result["acceleration_static_residual_enu_m_s2"] = {
            "mean_xyz": finite_list(np.mean(residual, axis=0)),
            "std_xyz": finite_list(np.std(residual, axis=0)),
            "note": (
                "This residual includes accelerometer bias and ENU attitude error. "
                "It is not a standalone calibrated accelerometer bias."
            ),
        }
        yaw = np.unwrap(
            np.asarray(
                [yaw_from_body_to_enu_quaternion(value) for value in orientation],
                dtype=float,
            )[stationary]
        )
        stationary_time = times[stationary] - times[stationary][0]
        slope, intercept = np.polyfit(stationary_time, yaw, 1)
        yaw_jitter = yaw - (slope * stationary_time + intercept)
        result["yaw_static_jitter"] = {
            "variance_rad2": float(np.var(yaw_jitter)),
            "std_rad": float(np.std(yaw_jitter)),
            "std_deg": float(np.degrees(np.std(yaw_jitter))),
            "peak_to_peak_deg": float(np.degrees(np.ptp(yaw_jitter))),
            "linear_drift_deg_per_hour": float(np.degrees(slope) * 3600.0),
            "note": (
                "Yaw is unwrapped and linearly detrended before calculating jitter. "
                "The variance excludes this fitted linear drift."
            ),
        }
    else:
        result["acceleration_static_residual_enu_m_s2"] = {
            "unavailable_reason": "The stationary samples do not all contain valid orientation."
        }
        result["yaw_static_jitter"] = {
            "unavailable_reason": "The stationary samples do not all contain valid orientation."
        }
    return result


def bias_difference(summary):
    gyro_1 = np.asarray(summary["imu1"]["gyro_bias_candidate_rad_s"]["mean_xyz"])
    gyro_2 = np.asarray(summary["imu2"]["gyro_bias_candidate_rad_s"]["mean_xyz"])
    result = {"gyro_bias_candidate_difference_rad_s": finite_list(gyro_1 - gyro_2)}
    residual_1 = summary["imu1"]["acceleration_static_residual_enu_m_s2"]
    residual_2 = summary["imu2"]["acceleration_static_residual_enu_m_s2"]
    if "mean_xyz" in residual_1 and "mean_xyz" in residual_2:
        result["acceleration_static_residual_difference_enu_m_s2"] = finite_list(
            np.asarray(residual_1["mean_xyz"]) - np.asarray(residual_2["mean_xyz"])
        )
    yaw_1 = summary["imu1"]["yaw_static_jitter"]
    yaw_2 = summary["imu2"]["yaw_static_jitter"]
    if "variance_rad2" in yaw_1 and "variance_rad2" in yaw_2:
        result["yaw_jitter"] = {
            "variance_ratio_imu1_over_imu2": float(
                yaw_1["variance_rad2"] / yaw_2["variance_rad2"]
            ) if yaw_2["variance_rad2"] > 0.0 else None,
            "std_difference_deg": float(yaw_1["std_deg"] - yaw_2["std_deg"]),
        }
    return result


def write_plot(output_dir, summary):
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    labels = ("x", "y", "z")
    index = np.arange(3)
    width = 0.36
    gyro_1 = summary["imu1"]["gyro_bias_candidate_rad_s"]["mean_xyz"]
    gyro_2 = summary["imu2"]["gyro_bias_candidate_rad_s"]["mean_xyz"]
    axes[0].bar(index - width / 2, gyro_1, width, label="imu1")
    axes[0].bar(index + width / 2, gyro_2, width, label="imu2")
    axes[0].set_title("Stationary gyro bias candidate")
    axes[0].set_ylabel("rad/s")
    axes[0].set_xticks(index, labels)
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()
    residuals = [
        summary[name]["acceleration_static_residual_enu_m_s2"]
        for name in ("imu1", "imu2")
    ]
    if all("mean_xyz" in residual for residual in residuals):
        axes[1].bar(index - width / 2, residuals[0]["mean_xyz"], width, label="imu1")
        axes[1].bar(index + width / 2, residuals[1]["mean_xyz"], width, label="imu2")
        axes[1].set_title("Static acceleration residual in ENU")
        axes[1].set_ylabel("m/s²")
        axes[1].set_xticks(index, labels)
        axes[1].grid(axis="y", alpha=0.25)
        axes[1].legend()
    else:
        axes[1].text(0.5, 0.5, "Valid orientation is required", ha="center", va="center")
        axes[1].set_axis_off()
    yaw = [summary[name]["yaw_static_jitter"] for name in ("imu1", "imu2")]
    if all("std_deg" in item for item in yaw):
        axes[2].bar(("imu1", "imu2"), [item["std_deg"] for item in yaw])
        axes[2].set_title("Yaw jitter after linear detrending")
        axes[2].set_ylabel("standard deviation (deg)")
        axes[2].grid(axis="y", alpha=0.25)
    else:
        axes[2].text(0.5, 0.5, "Valid orientation is required", ha="center", va="center")
        axes[2].set_axis_off()
    figure.tight_layout()
    figure.savefig(output_dir / "bias_comparison.png", dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Path to a stationary dual_imu_recorder rosbag")
    parser.add_argument("--output-dir", type=Path, help="Directory for JSON and PNG output")
    parser.add_argument("--max-gyro-rad-s", type=float, default=0.02, help="Stationary gyro threshold")
    parser.add_argument("--gravity-tolerance-m-s2", type=float, default=0.2, help="Stationary gravity-norm threshold")
    args = parser.parse_args()
    if not args.bag.is_dir():
        parser.error(f"Bag directory does not exist: {args.bag}")
    output_dir = args.output_dir or args.bag / "bias_analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    samples = read_bag(args.bag)
    streams = {
        name: stationary_bias_summary(
            data, args.max_gyro_rad_s, args.gravity_tolerance_m_s2
        )
        for name, data in samples.items()
    }
    summary = {
        "bag": str(args.bag.resolve()),
        "gravity_m_s2": GRAVITY_M_S2,
        "streams": streams,
        "comparison": bias_difference(streams),
    }
    write_plot(output_dir, streams)
    summary_path = output_dir / "bias_summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"\nAnalysis files written to: {output_dir}")


if __name__ == "__main__":
    main()
