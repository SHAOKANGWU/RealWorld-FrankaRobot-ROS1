#!/usr/bin/env python3

"""Convert a recorded demonstration to 15-column reproduction commands.

Rows are x y z qx qy qz qw joint1 ... joint7 gripper_action. The final
column is inferred from measured gripper width, not a recorded action goal.
"""

import argparse
from pathlib import Path

import numpy as np


def load_array(archive, name, expected_shape):
    if name not in archive:
        raise ValueError("trajectory is missing '{}'".format(name))
    array = np.asarray(archive[name], dtype=np.float64)
    if array.ndim != len(expected_shape) or any(
        expected is not None and actual != expected
        for actual, expected in zip(array.shape, expected_shape)
    ):
        raise ValueError(
            "'{}' has shape {}; expected {}".format(
                name, array.shape, expected_shape
            )
        )
    if not np.all(np.isfinite(array)):
        raise ValueError("'{}' contains a non-finite value".format(name))
    return array


def make_commands(archive, grasp_width_threshold):
    robot_time = load_array(archive, "robot_time", (None,))
    position = load_array(archive, "ee_position", (len(robot_time), 3))
    quaternion_wxyz = load_array(
        archive, "ee_quaternion_wxyz", (len(robot_time), 4)
    )
    joints = load_array(archive, "q", (len(robot_time), 7))
    gripper_time = load_array(archive, "gripper_time", (None,))
    gripper_width = load_array(
        archive, "gripper_width", (len(gripper_time),)
    )

    if not len(robot_time):
        raise ValueError("trajectory has no robot samples")
    if not len(gripper_time):
        raise ValueError("trajectory has no gripper samples")
    if np.any(np.diff(robot_time) < 0) or np.any(np.diff(gripper_time) < 0):
        raise ValueError("robot and gripper timestamps must be nondecreasing")
    if np.any(gripper_width < 0):
        raise ValueError("gripper_width contains a negative value")

    quaternion_norm = np.linalg.norm(quaternion_wxyz, axis=1)
    if np.any(quaternion_norm < 1e-6):
        raise ValueError("trajectory contains a zero quaternion")
    quaternion_wxyz = quaternion_wxyz / quaternion_norm[:, None]
    quaternion_xyzw = quaternion_wxyz[:, [1, 2, 3, 0]]

    # Hold the latest gripper observation for each robot sample. Before the
    # first gripper sample, use that first sample as the initial state.
    gripper_index = np.clip(
        np.searchsorted(gripper_time, robot_time, side="right") - 1,
        0,
        len(gripper_time) - 1,
    )
    grasp = (gripper_width[gripper_index] <= grasp_width_threshold).astype(int)

    return np.column_stack((position, quaternion_xyzw, joints, grasp))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        type=Path,
        help="recorded trajectory.npz or its episode directory",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="output text path (default: trajectory.txt beside trajectory.npz)",
    )
    parser.add_argument(
        "--grasp-width-threshold",
        type=float,
        default=0.04,
        help="width in metres at or below which action is 1 (default: 0.04)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace an existing output file",
    )
    args = parser.parse_args()

    if not np.isfinite(args.grasp_width_threshold) or args.grasp_width_threshold < 0:
        parser.error("--grasp-width-threshold must be finite and non-negative")

    input_path = args.input / "trajectory.npz" if args.input.is_dir() else args.input
    output_path = args.output or input_path.with_suffix(".txt")
    if output_path.resolve() == input_path.resolve():
        parser.error("output path must differ from input path")
    if output_path.exists() and not args.overwrite:
        parser.error("output already exists; use --overwrite to replace it")

    try:
        with np.load(input_path, allow_pickle=False) as archive:
            commands = make_commands(archive, args.grasp_width_threshold)
        with output_path.open("w" if args.overwrite else "x") as stream:
            np.savetxt(stream, commands, fmt=["%.10f"] * 14 + ["%d"])
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))

    print("Saved {} trajectory points to {}".format(len(commands), output_path))
    print(
        "Columns: x y z qx qy qz qw joint1 ... joint7 gripper_action"
    )
    print(
        "Inferred gripper action: 1 when observed width <= {:.4f} m; 0 otherwise."
        .format(args.grasp_width_threshold)
    )


if __name__ == "__main__":
    main()
