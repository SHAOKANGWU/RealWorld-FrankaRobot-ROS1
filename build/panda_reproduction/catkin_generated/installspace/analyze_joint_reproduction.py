#!/usr/bin/env python3

"""Measure joint-reproduction tracking accuracy from a ROS bag."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_COMMAND_TOPIC = (
    "/joint_position_torque_controller/desired_joint_position_command"
)
DEFAULT_STATE_TOPIC = "/franka_state_controller/franka_states"
NUMBER_OF_JOINTS = 7


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


def load_trajectory(path):
    try:
        trajectory = np.loadtxt(path, dtype=np.float64, ndmin=2)
    except (OSError, ValueError) as error:
        raise ValueError("cannot read trajectory '{}': {}".format(path, error)) from error
    if trajectory.shape[1] != 15 or not len(trajectory):
        raise ValueError("trajectory must contain nonempty 15-value rows")
    if not np.all(np.isfinite(trajectory)):
        raise ValueError("trajectory contains a non-finite value")
    return trajectory[:, 7:14]


def message_time(message, bag_time):
    if hasattr(message, "header") and message.header.stamp.to_sec() > 0.0:
        return message.header.stamp.to_sec()
    return bag_time.to_sec()


def load_bag(path, command_topic, state_topic):
    command_time = []
    command_bag_time = []
    commands = []
    state_time = []
    state_bag_time = []
    positions = []
    velocities = []

    try:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(
                topics=[command_topic, state_topic]
            ):
                if topic == command_topic:
                    values = np.asarray(message.joint_positions, dtype=np.float64)
                    if values.shape != (NUMBER_OF_JOINTS,):
                        raise ValueError("a command does not contain seven joints")
                    command_time.append(message_time(message, bag_time))
                    command_bag_time.append(bag_time.to_sec())
                    commands.append(values)
                else:
                    q = np.asarray(message.q, dtype=np.float64)
                    dq = np.asarray(message.dq, dtype=np.float64)
                    if q.shape != (NUMBER_OF_JOINTS,) or dq.shape != (NUMBER_OF_JOINTS,):
                        raise ValueError("a Franka state does not contain seven joints")
                    state_time.append(message_time(message, bag_time))
                    state_bag_time.append(bag_time.to_sec())
                    positions.append(q)
                    velocities.append(dq)
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
            velocities,
        )
    )
    if not len(arrays[0]):
        raise ValueError("bag has no messages on '{}'".format(command_topic))
    if not len(arrays[3]):
        raise ValueError("bag has no messages on '{}'".format(state_topic))
    if not all(np.all(np.isfinite(array)) for array in arrays):
        raise ValueError("bag contains a non-finite command, state, or timestamp")
    if np.any(np.diff(arrays[0]) <= 0.0) or np.any(np.diff(arrays[3]) <= 0.0):
        raise ValueError("command and state header timestamps must increase")
    return arrays


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
    indices = np.searchsorted(
        command_times, sample_times - delay, side="right"
    ) - 1
    indices = np.clip(indices, 0, len(commands) - 1)
    return commands[indices]


def error_metrics(errors):
    absolute = np.abs(errors)
    return {
        "rmse": np.sqrt(np.mean(np.square(errors), axis=0)),
        "mae": np.mean(absolute, axis=0),
        "p95": np.percentile(absolute, 95.0, axis=0),
        "maximum": np.max(absolute, axis=0),
        "bias": np.mean(errors, axis=0),
    }


def scalar_error_metrics(errors):
    absolute = np.abs(errors)
    return {
        "rmse_rad": float(np.sqrt(np.mean(np.square(errors)))),
        "mae_rad": float(np.mean(absolute)),
        "p95_rad": float(np.percentile(absolute, 95.0)),
        "maximum_rad": float(np.max(absolute)),
        "bias_rad": float(np.mean(errors)),
    }


def lagged_errors(command_times, commands, state_times, positions, delay):
    mask = (
        (state_times >= command_times[0] + delay)
        & (state_times <= command_times[-1] + delay)
    )
    desired = held_commands(command_times, commands, state_times[mask], delay)
    return mask, desired - positions[mask]


def find_best_lags(command_times, commands, state_times, positions, max_lag, lag_step):
    candidate_lags = np.arange(0.0, max_lag + 0.5 * lag_step, lag_step)
    aggregate_scores = []
    joint_scores = [[] for _ in range(NUMBER_OF_JOINTS)]
    for delay in candidate_lags:
        _, errors = lagged_errors(
            command_times, commands, state_times, positions, delay
        )
        if not len(errors):
            raise ValueError("no state samples remain while estimating delay")
        aggregate_scores.append(float(np.sqrt(np.mean(np.square(errors)))))
        for joint in range(NUMBER_OF_JOINTS):
            joint_scores[joint].append(
                float(np.sqrt(np.mean(np.square(errors[:, joint]))))
            )
    aggregate_index = int(np.argmin(aggregate_scores))
    joint_indices = [int(np.argmin(scores)) for scores in joint_scores]
    return (
        float(candidate_lags[aggregate_index]),
        [float(candidate_lags[index]) for index in joint_indices],
    )


def serializable_metrics(metrics, joint):
    return {
        name + "_rad": float(values[joint])
        for name, values in metrics.items()
    }


def save_tracking_plot(
    output_path, command_times, commands, state_times, positions, dynamic_end
):
    start = command_times[0]
    command_mask = command_times <= dynamic_end
    state_mask = (state_times >= start) & (state_times <= dynamic_end)
    figure, axes = plt.subplots(NUMBER_OF_JOINTS, 1, figsize=(13, 16), sharex=True)
    for joint, axis in enumerate(axes):
        axis.step(
            command_times[command_mask] - start,
            commands[command_mask, joint],
            where="post",
            label="desired",
            linewidth=1.2,
        )
        axis.plot(
            state_times[state_mask] - start,
            positions[state_mask, joint],
            label="measured",
            linewidth=1.0,
        )
        axis.set_ylabel("J{} [rad]".format(joint + 1))
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time from first command [s]")
    figure.suptitle("Joint reproduction: desired and measured positions")
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def save_error_plot(output_path, times, aligned_errors, lag_times, lag_errors, lag):
    start = times[0]
    figure, axes = plt.subplots(NUMBER_OF_JOINTS, 1, figsize=(13, 16), sharex=True)
    for joint, axis in enumerate(axes):
        axis.plot(
            times - start,
            np.degrees(aligned_errors[:, joint]),
            label="timestamp aligned",
            linewidth=1.0,
        )
        axis.plot(
            lag_times - start,
            np.degrees(lag_errors[:, joint]),
            label="compensated by {:.3f} s".format(lag),
            linewidth=1.0,
        )
        axis.set_ylabel("J{} [deg]".format(joint + 1))
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time from first measured sample [s]")
    figure.suptitle("Joint reproduction errors")
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def analyze(arguments):
    trajectory_commands = load_trajectory(arguments.trajectory_file)
    (
        command_times,
        command_bag_times,
        commands,
        state_times,
        state_bag_times,
        positions,
        velocities,
    ) = load_bag(arguments.bag, arguments.command_topic, arguments.state_topic)

    dynamic_count = len(trajectory_commands)
    if len(commands) < dynamic_count:
        raise ValueError(
            "bag contains {} commands but trajectory contains {} rows; start recording "
            "before reproduction".format(len(commands), dynamic_count)
        )
    if not np.allclose(
        commands[:dynamic_count], trajectory_commands, rtol=0.0, atol=1e-8
    ):
        maximum_difference = float(
            np.max(np.abs(commands[:dynamic_count] - trajectory_commands))
        )
        raise ValueError(
            "recorded commands do not match the trajectory (maximum difference "
            "{:.6g} rad)".format(maximum_difference)
        )

    dynamic_times = command_times[:dynamic_count]
    dynamic_commands = commands[:dynamic_count]
    dynamic_end = dynamic_times[-1]
    aligned_mask, aligned_errors = lagged_errors(
        dynamic_times, dynamic_commands, state_times, positions, 0.0
    )
    aligned_metrics = error_metrics(aligned_errors)

    aggregate_lag, joint_lags = find_best_lags(
        dynamic_times,
        dynamic_commands,
        state_times,
        positions,
        arguments.max_lag,
        arguments.lag_step,
    )
    lag_mask, lag_errors = lagged_errors(
        dynamic_times,
        dynamic_commands,
        state_times,
        positions,
        aggregate_lag,
    )
    lag_metrics = error_metrics(lag_errors)
    joint_lag_metrics = []
    for joint, delay in enumerate(joint_lags):
        _, joint_errors = lagged_errors(
            dynamic_times, dynamic_commands, state_times, positions, delay
        )
        joint_lag_metrics.append(scalar_error_metrics(joint_errors[:, joint]))

    hold_end = min(state_times[-1], command_times[-1])
    hold_mask = (
        (state_times >= dynamic_end + arguments.settle_time)
        & (state_times <= hold_end)
    )
    hold_metrics = None
    if np.count_nonzero(hold_mask):
        hold_errors = dynamic_commands[-1] - positions[hold_mask]
        hold_metrics = error_metrics(hold_errors)

    output_dir = arguments.output_dir or Path(
        str(arguments.bag.with_suffix("")) + "_analysis"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for joint in range(NUMBER_OF_JOINTS):
        row = {
            "joint": joint + 1,
            "trajectory_range_rad": float(np.ptp(dynamic_commands[:, joint])),
            "individual_best_lag_s": joint_lags[joint],
        }
        row.update(
            {
                "aligned_" + name: value
                for name, value in serializable_metrics(
                    aligned_metrics, joint
                ).items()
            }
        )
        row.update(
            {
                "aggregate_lag_compensated_" + name: value
                for name, value in serializable_metrics(lag_metrics, joint).items()
            }
        )
        row.update(
            {
                "individual_lag_compensated_" + name: value
                for name, value in joint_lag_metrics[joint].items()
            }
        )
        if hold_metrics is not None:
            row.update(
                {
                    "hold_" + name: value
                    for name, value in serializable_metrics(
                        hold_metrics, joint
                    ).items()
                }
            )
        rows.append(row)

    csv_path = output_dir / "joint_metrics.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    aligned_norm = np.linalg.norm(aligned_errors, axis=1)
    lag_norm = np.linalg.norm(lag_errors, axis=1)
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
        "aggregate_best_lag_s": aggregate_lag,
        "aggregate_aligned_rmse_rad": float(np.sqrt(np.mean(np.square(aligned_errors)))),
        "aggregate_lag_compensated_rmse_rad": float(
            np.sqrt(np.mean(np.square(lag_errors)))
        ),
        "aligned_vector_norm_rmse_rad": float(
            np.sqrt(np.mean(np.square(aligned_norm)))
        ),
        "aligned_vector_norm_maximum_rad": float(np.max(aligned_norm)),
        "lag_compensated_vector_norm_rmse_rad": float(
            np.sqrt(np.mean(np.square(lag_norm)))
        ),
        "lag_compensated_vector_norm_maximum_rad": float(np.max(lag_norm)),
        "hold_samples": int(np.count_nonzero(hold_mask)),
        "maximum_measured_velocity_rad_s": np.max(
            np.abs(velocities[aligned_mask]), axis=0
        ).tolist(),
        "joints": rows,
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")

    save_tracking_plot(
        output_dir / "joint_tracking.png",
        command_times,
        commands,
        state_times,
        positions,
        dynamic_end,
    )
    save_error_plot(
        output_dir / "joint_errors.png",
        state_times[aligned_mask],
        aligned_errors,
        state_times[lag_mask],
        lag_errors,
        aggregate_lag,
    )

    print("Analyzed {} trajectory commands over {:.3f} s".format(
        dynamic_count, summary["dynamic_duration_s"]
    ))
    print("Aggregate best constant lag: {:.3f} s".format(aggregate_lag))
    print("Joint  aligned RMSE  aligned p95  best lag  hold bias")
    for row in rows:
        hold_bias = row.get("hold_bias_rad", float("nan"))
        print(
            "{:>5d}  {:>12.5f}  {:>11.5f}  {:>8.3f}  {:>9.5f}".format(
                row["joint"],
                row["aligned_rmse_rad"],
                row["aligned_p95_rad"],
                row["individual_best_lag_s"],
                hold_bias,
            )
        )
    print("Saved analysis to {}".format(output_dir.resolve()))


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="recorded reproduction bag")
    parser.add_argument(
        "trajectory_file", type=Path, help="15-column trajectory.txt used for replay"
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--max-lag", type=nonnegative_float, default=1.0)
    parser.add_argument("--lag-step", type=positive_float, default=0.001)
    parser.add_argument(
        "--settle-time",
        type=nonnegative_float,
        default=1.0,
        help="seconds after the final trajectory command before evaluating hold error",
    )
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
