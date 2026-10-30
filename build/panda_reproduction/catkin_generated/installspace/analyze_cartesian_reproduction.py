#!/usr/bin/env python3

"""Measure Cartesian-pose reproduction accuracy from a ROS bag."""

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_COMMAND_TOPIC = (
    "/cartesian_pose_torque_controller/desired_cartesian_pose_command"
)
DEFAULT_STATE_TOPIC = "/franka_state_controller/franka_states"


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return number


def nonnegative_float(value):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return number


def normalize_quaternions(quaternions):
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any(norms < 1e-9):
        raise ValueError("encountered a zero quaternion")
    return quaternions / norms[:, None]


def rotation_matrix_to_quaternion(matrix):
    """Return an xyzw quaternion for a proper 3-by-3 rotation matrix."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    return quaternion / np.linalg.norm(quaternion)


def load_trajectory(path):
    try:
        trajectory = np.loadtxt(path, dtype=np.float64, ndmin=2)
    except (OSError, ValueError) as error:
        raise ValueError("cannot read trajectory '{}': {}".format(path, error)) from error
    if trajectory.shape[1] != 15 or not len(trajectory):
        raise ValueError("trajectory must contain nonempty 15-value rows")
    poses = trajectory[:, :7]
    if not np.all(np.isfinite(poses)):
        raise ValueError("trajectory contains a non-finite pose")
    poses[:, 3:7] = normalize_quaternions(poses[:, 3:7])
    return poses


def message_time(message, bag_time):
    if hasattr(message, "header") and message.header.stamp.to_sec() > 0.0:
        return message.header.stamp.to_sec()
    return bag_time.to_sec()


def load_bag(path, command_topic, state_topic):
    command_time, command_bag_time, commands = [], [], []
    state_time, state_bag_time, positions, orientations = [], [], [], []
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(
                topics=[command_topic, state_topic]
            ):
                if topic == command_topic:
                    pose = message.pose
                    command_time.append(message_time(message, bag_time))
                    command_bag_time.append(bag_time.to_sec())
                    commands.append(
                        [
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ]
                    )
                else:
                    transform = np.asarray(message.O_T_EE, dtype=np.float64).reshape(
                        (4, 4), order="F"
                    )
                    rotation = transform[:3, :3]
                    if (
                        not np.all(np.isfinite(transform))
                        or not np.allclose(transform[3, :], [0, 0, 0, 1], atol=1e-3)
                        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
                        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)
                    ):
                        raise ValueError("a Franka state contains an invalid transform")
                    state_time.append(message_time(message, bag_time))
                    state_bag_time.append(bag_time.to_sec())
                    positions.append(transform[:3, 3])
                    orientations.append(rotation_matrix_to_quaternion(rotation))
    except (OSError, rosbag.ROSBagException) as error:
        raise ValueError("cannot read bag '{}': {}".format(path, error)) from error

    arrays = tuple(
        np.asarray(values, dtype=np.float64)
        for values in (
            command_time,
            command_bag_time,
            commands,
            state_time,
            state_bag_time,
            positions,
            orientations,
        )
    )
    if not len(arrays[0]):
        raise ValueError("bag has no messages on '{}'".format(command_topic))
    if not len(arrays[3]):
        raise ValueError("bag has no messages on '{}'".format(state_topic))
    if not all(np.all(np.isfinite(array)) for array in arrays):
        raise ValueError("bag contains a non-finite pose or timestamp")
    if np.any(np.diff(arrays[0]) <= 0.0) or np.any(np.diff(arrays[3]) <= 0.0):
        raise ValueError("command and state header timestamps must increase")
    commands_array = arrays[2].copy()
    commands_array[:, 3:7] = normalize_quaternions(commands_array[:, 3:7])
    return arrays[:2] + (commands_array,) + arrays[3:]


def timing_statistics(times):
    intervals = np.diff(times)
    return {
        "samples": int(len(times)),
        "duration_s": float(times[-1] - times[0]),
        "mean_rate_hz": float(1.0 / np.mean(intervals)),
        "median_rate_hz": float(1.0 / np.median(intervals)),
        "maximum_gap_s": float(np.max(intervals)),
    }


def held_commands(command_times, commands, sample_times, delay=0.0):
    indices = np.searchsorted(command_times, sample_times - delay, side="right") - 1
    indices = np.clip(indices, 0, len(commands) - 1)
    return commands[indices]


def quaternion_errors(desired, actual):
    dots = np.sum(desired * actual, axis=1)
    signs = np.where(dots < 0.0, -1.0, 1.0)
    actual_aligned = actual * signs[:, None]
    desired_xyz, desired_w = desired[:, :3], desired[:, 3]
    actual_xyz, actual_w = actual_aligned[:, :3], actual_aligned[:, 3]
    vector = (
        actual_w[:, None] * desired_xyz
        - desired_w[:, None] * actual_xyz
        + np.cross(actual_xyz, desired_xyz)
    )
    scalar = actual_w * desired_w + np.sum(actual_xyz * desired_xyz, axis=1)
    scalar = np.clip(scalar, 0.0, 1.0)
    vector_norm = np.linalg.norm(vector, axis=1)
    angles = 2.0 * np.arctan2(vector_norm, scalar)
    rotation_vectors = np.zeros_like(vector)
    nonzero = vector_norm > 1e-12
    rotation_vectors[nonzero] = (
        vector[nonzero]
        * (angles[nonzero] / vector_norm[nonzero])[:, None]
    )
    return angles, rotation_vectors, actual_aligned


def errors_at_delay(command_times, commands, state_times, positions, orientations, delay):
    mask = (
        (state_times >= command_times[0] + delay)
        & (state_times <= command_times[-1] + delay)
    )
    desired = held_commands(command_times, commands, state_times[mask], delay)
    position_errors = desired[:, :3] - positions[mask]
    orientation_angles, orientation_vectors, aligned_actual = quaternion_errors(
        desired[:, 3:7], orientations[mask]
    )
    return (
        mask,
        desired,
        position_errors,
        orientation_angles,
        orientation_vectors,
        aligned_actual,
    )


def scalar_metrics(values):
    values = np.asarray(values)
    absolute = np.abs(values)
    return {
        "rmse": float(np.sqrt(np.mean(np.square(values)))),
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(absolute, 95.0)),
        "maximum": float(np.max(absolute)),
    }


def vector_metrics(errors):
    norms = np.linalg.norm(errors, axis=1)
    return {
        "component_rmse": np.sqrt(np.mean(np.square(errors), axis=0)).tolist(),
        "component_bias": np.mean(errors, axis=0).tolist(),
        "norm_rmse": float(np.sqrt(np.mean(np.square(norms)))),
        "norm_mean": float(np.mean(norms)),
        "norm_p95": float(np.percentile(norms, 95.0)),
        "norm_maximum": float(np.max(norms)),
    }


def find_best_lags(command_times, commands, state_times, positions, orientations, max_lag, step):
    candidates = np.arange(0.0, max_lag + 0.5 * step, step)
    position_scores, orientation_scores = [], []
    for delay in candidates:
        result = errors_at_delay(
            command_times, commands, state_times, positions, orientations, delay
        )
        position_scores.append(float(np.sqrt(np.mean(np.square(result[2])))))
        orientation_scores.append(float(np.sqrt(np.mean(np.square(result[3])))))
    return (
        float(candidates[int(np.argmin(position_scores))]),
        float(candidates[int(np.argmin(orientation_scores))]),
    )


def save_tracking_plot(path, command_times, commands, state_times, positions, orientations, end):
    start = command_times[0]
    command_mask = command_times <= end
    state_mask = (state_times >= start) & (state_times <= end)
    desired_at_state = held_commands(command_times, commands, state_times[state_mask])
    _, _, aligned_actual = quaternion_errors(
        desired_at_state[:, 3:7], orientations[state_mask]
    )
    labels = ["x [m]", "y [m]", "z [m]", "qx", "qy", "qz", "qw"]
    measured = np.column_stack((positions[state_mask], aligned_actual))
    figure, axes = plt.subplots(7, 1, figsize=(13, 16), sharex=True)
    for index, axis in enumerate(axes):
        axis.step(
            command_times[command_mask] - start,
            commands[command_mask, index],
            where="post",
            label="desired",
            linewidth=1.2,
        )
        axis.plot(
            state_times[state_mask] - start,
            measured[:, index],
            label="measured",
            linewidth=1.0,
        )
        axis.set_ylabel(labels[index])
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time from first command [s]")
    figure.suptitle("Cartesian reproduction: desired and measured pose")
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def save_error_plot(path, aligned, position_compensated, orientation_compensated, start):
    aligned_times, aligned_position, aligned_angle = aligned
    position_times, position_error, _ = position_compensated
    orientation_times, _, orientation_angle = orientation_compensated
    figure, axes = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    labels = ["x error [mm]", "y error [mm]", "z error [mm]"]
    for index in range(3):
        axes[index].plot(aligned_times - start, 1e3 * aligned_position[:, index], label="aligned")
        axes[index].plot(position_times - start, 1e3 * position_error[:, index], label="lag compensated")
        axes[index].set_ylabel(labels[index])
    axes[3].plot(aligned_times - start, 1e3 * np.linalg.norm(aligned_position, axis=1))
    axes[3].plot(position_times - start, 1e3 * np.linalg.norm(position_error, axis=1))
    axes[3].set_ylabel("position norm [mm]")
    axes[4].plot(aligned_times - start, np.degrees(aligned_angle))
    axes[4].plot(orientation_times - start, np.degrees(orientation_angle))
    axes[4].set_ylabel("orientation [deg]")
    axes[4].set_xlabel("Time from first command [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    figure.suptitle("Cartesian reproduction errors")
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def analyze(arguments):
    trajectory = load_trajectory(arguments.trajectory_file)
    (
        command_times,
        command_bag_times,
        commands,
        state_times,
        state_bag_times,
        positions,
        orientations,
    ) = load_bag(arguments.bag, arguments.command_topic, arguments.state_topic)
    dynamic_count = len(trajectory)
    if len(commands) < dynamic_count:
        raise ValueError("bag has fewer commands than the trajectory")
    position_difference = np.max(np.abs(commands[:dynamic_count, :3] - trajectory[:, :3]))
    orientation_difference = 2.0 * np.arccos(
        np.clip(
            np.abs(np.sum(commands[:dynamic_count, 3:7] * trajectory[:, 3:7], axis=1)),
            0.0,
            1.0,
        )
    )
    if position_difference > 1e-8 or np.max(orientation_difference) > 1e-6:
        raise ValueError("recorded commands do not match the supplied trajectory")

    dynamic_times = command_times[:dynamic_count]
    dynamic_commands = commands[:dynamic_count]
    dynamic_end = dynamic_times[-1]
    aligned = errors_at_delay(
        dynamic_times, dynamic_commands, state_times, positions, orientations, 0.0
    )
    position_lag, orientation_lag = find_best_lags(
        dynamic_times,
        dynamic_commands,
        state_times,
        positions,
        orientations,
        arguments.max_lag,
        arguments.lag_step,
    )
    position_compensated = errors_at_delay(
        dynamic_times, dynamic_commands, state_times, positions, orientations, position_lag
    )
    orientation_compensated = errors_at_delay(
        dynamic_times, dynamic_commands, state_times, positions, orientations, orientation_lag
    )

    hold_end = min(state_times[-1], command_times[-1])
    hold_mask = (state_times >= dynamic_end + arguments.settle_time) & (state_times <= hold_end)
    hold_position = None
    hold_angle = None
    hold_rotation_vector = None
    if np.count_nonzero(hold_mask):
        final = dynamic_commands[-1]
        hold_position = final[:3] - positions[hold_mask]
        hold_angle, hold_rotation_vector, _ = quaternion_errors(
            np.repeat(final[None, 3:7], np.count_nonzero(hold_mask), axis=0),
            orientations[hold_mask],
        )

    position_mask, _, position_error, _, _, _ = position_compensated
    orientation_mask, _, _, orientation_angle, _, _ = orientation_compensated
    summary = {
        "bag": str(arguments.bag.resolve()),
        "trajectory_file": str(arguments.trajectory_file.resolve()),
        "command_topic": arguments.command_topic,
        "state_topic": arguments.state_topic,
        "dynamic_commands": dynamic_count,
        "hold_commands": int(len(commands) - dynamic_count),
        "dynamic_duration_s": float(dynamic_end - dynamic_times[0]),
        "command_timing": timing_statistics(command_times),
        "state_timing": timing_statistics(state_times),
        "command_header_to_bag_delay_ms": {
            "median": float(1e3 * np.median(command_bag_times - command_times)),
            "p95": float(1e3 * np.percentile(command_bag_times - command_times, 95)),
            "maximum": float(1e3 * np.max(command_bag_times - command_times)),
        },
        "state_header_to_bag_delay_ms": {
            "median": float(1e3 * np.median(state_bag_times - state_times)),
            "p95": float(1e3 * np.percentile(state_bag_times - state_times, 95)),
            "maximum": float(1e3 * np.max(state_bag_times - state_times)),
        },
        "best_position_lag_s": position_lag,
        "best_orientation_lag_s": orientation_lag,
        "aligned_position_m": vector_metrics(aligned[2]),
        "aligned_orientation_rad": scalar_metrics(aligned[3]),
        "aligned_orientation_vector_rad": vector_metrics(aligned[4]),
        "lag_compensated_position_m": vector_metrics(position_error),
        "lag_compensated_orientation_rad": scalar_metrics(orientation_angle),
        "hold_samples": int(np.count_nonzero(hold_mask)),
    }
    if hold_position is not None:
        summary["hold_position_m"] = vector_metrics(hold_position)
        summary["hold_orientation_rad"] = scalar_metrics(hold_angle)
        summary["hold_orientation_vector_rad"] = vector_metrics(hold_rotation_vector)

    output_dir = arguments.output_dir or Path(str(arguments.bag.with_suffix("")) + "_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    save_tracking_plot(
        output_dir / "cartesian_tracking.png",
        command_times,
        commands,
        state_times,
        positions,
        orientations,
        dynamic_end,
    )
    save_error_plot(
        output_dir / "cartesian_errors.png",
        (state_times[aligned[0]], aligned[2], aligned[3]),
        (state_times[position_mask], position_error, position_compensated[3]),
        (state_times[orientation_mask], orientation_compensated[2], orientation_angle),
        dynamic_times[0],
    )

    print("Analyzed {} trajectory commands over {:.3f} s".format(dynamic_count, summary["dynamic_duration_s"]))
    print("Position: RMSE {:.3f} mm, p95 {:.3f} mm, lag {:.3f} s".format(
        1e3 * summary["aligned_position_m"]["norm_rmse"],
        1e3 * summary["aligned_position_m"]["norm_p95"],
        position_lag,
    ))
    print("Orientation: RMSE {:.3f} deg, p95 {:.3f} deg, lag {:.3f} s".format(
        math.degrees(summary["aligned_orientation_rad"]["rmse"]),
        math.degrees(summary["aligned_orientation_rad"]["p95"]),
        orientation_lag,
    ))
    if hold_position is not None:
        print("Final hold: {:.3f} mm position RMSE, {:.3f} deg orientation RMSE".format(
            1e3 * summary["hold_position_m"]["norm_rmse"],
            math.degrees(summary["hold_orientation_rad"]["rmse"]),
        ))
    print("Saved analysis to {}".format(output_dir.resolve()))


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="recorded Cartesian reproduction bag")
    parser.add_argument("trajectory_file", type=Path, help="15-column trajectory used for replay")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--max-lag", type=nonnegative_float, default=1.0)
    parser.add_argument("--lag-step", type=positive_float, default=0.001)
    parser.add_argument("--settle-time", type=nonnegative_float, default=1.0)
    return parser.parse_args()


def main():
    try:
        analyze(parse_arguments())
    except (ValueError, OSError) as error:
        print("error: {}".format(error))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
