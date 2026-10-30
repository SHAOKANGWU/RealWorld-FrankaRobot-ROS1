#!/usr/bin/env python3

import argparse
import json
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np


# ============================================================
# Utility functions
# ============================================================

def check_time_stream(name, time_array):

    print(f"\n--- {name} timing ---")

    if len(time_array) < 2:
        print("Not enough samples.")
        return

    dt = np.diff(time_array)

    print(f"Samples: {len(time_array)}")

    print(
        f"Duration: "
        f"{time_array[-1] - time_array[0]:.3f} s"
    )

    if np.any(dt <= 0.0):

        bad_count = np.sum(dt <= 0.0)

        print(
            f"WARNING: {bad_count} "
            "non-increasing timestamps found."
        )

    else:

        print(
            "Timestamps strictly increasing: YES"
        )

    positive_dt = dt[dt > 0.0]

    if len(positive_dt) > 0:

        median_dt = np.median(
            positive_dt
        )

        mean_dt = np.mean(
            positive_dt
        )

        max_dt = np.max(
            positive_dt
        )

        frequency = (
            1.0 / median_dt
        )

        print(
            f"Median dt: {median_dt:.6f} s"
        )

        print(
            f"Mean dt:   {mean_dt:.6f} s"
        )

        print(
            f"Max gap:   {max_dt:.6f} s"
        )

        print(
            f"Approx. frequency: "
            f"{frequency:.2f} Hz"
        )


def check_nan(name, array):

    number_nan = np.isnan(
        array
    ).sum()

    number_inf = np.isinf(
        array
    ).sum()

    if number_nan == 0 and number_inf == 0:

        print(
            f"{name}: finite values OK"
        )

    else:

        print(
            f"WARNING: {name} contains "
            f"{number_nan} NaN and "
            f"{number_inf} Inf values."
        )


# ============================================================
# Validation
# ============================================================

def validate_episode(episode_dir):

    trajectory_path = os.path.join(
        episode_dir,
        "trajectory.npz"
    )

    metadata_path = os.path.join(
        episode_dir,
        "metadata.json"
    )

    if not os.path.exists(
        trajectory_path
    ):
        raise FileNotFoundError(
            trajectory_path
        )

    data = np.load(
        trajectory_path
    )

    print(
        "\n======================================"
    )

    print(
        " Kinesthetic demonstration validation"
    )

    print(
        "======================================"
    )

    # --------------------------------------------------------
    # Metadata
    # --------------------------------------------------------

    if os.path.exists(
        metadata_path
    ):

        with open(
            metadata_path,
            "r"
        ) as file:

            metadata = json.load(file)

        print(
            f"\nDuration: "
            f"{metadata['duration']:.3f} s"
        )

        print(
            f"Robot samples:   "
            f"{metadata['robot_samples']}"
        )

        print(
            f"Gripper samples: "
            f"{metadata['gripper_samples']}"
        )

        print(
            f"Image samples:   "
            f"{metadata['image_samples']}"
        )

    # --------------------------------------------------------
    # Load arrays
    # --------------------------------------------------------

    robot_time = data[
        "robot_time_relative"
    ]

    gripper_time = data[
        "gripper_time_relative"
    ]

    image_time = data[
        "image_time_relative"
    ]

    q = data["q"]

    dq = data["dq"]

    ee_position = data[
        "ee_position"
    ]

    quaternion = data[
        "ee_quaternion_wxyz"
    ]

    O_T_EE = data[
        "O_T_EE"
    ]

    gripper_joint_position = data[
        "gripper_joint_position"
    ]

    gripper_width = data[
        "gripper_width"
    ]

    image_paths = data[
        "image_paths"
    ]

    # --------------------------------------------------------
    # Shapes
    # --------------------------------------------------------

    print("\n--- Array shapes ---")

    print(
        "q:                  ",
        q.shape
    )

    print(
        "dq:                 ",
        dq.shape
    )

    print(
        "EE position:        ",
        ee_position.shape
    )

    print(
        "EE quaternion:      ",
        quaternion.shape
    )

    print(
        "O_T_EE:             ",
        O_T_EE.shape
    )

    print(
        "gripper joints:     ",
        gripper_joint_position.shape
    )

    print(
        "gripper width:      ",
        gripper_width.shape
    )

    print(
        "image paths:        ",
        image_paths.shape
    )

    # --------------------------------------------------------
    # Expected dimensions
    # --------------------------------------------------------

    if q.ndim != 2 or q.shape[1] != 7:
        print(
            "WARNING: q should have shape (N, 7)."
        )

    if dq.ndim != 2 or dq.shape[1] != 7:
        print(
            "WARNING: dq should have shape (N, 7)."
        )

    if (
        ee_position.ndim != 2
        or ee_position.shape[1] != 3
    ):
        print(
            "WARNING: EE position should "
            "have shape (N, 3)."
        )

    if (
        quaternion.ndim != 2
        or quaternion.shape[1] != 4
    ):
        print(
            "WARNING: quaternion should "
            "have shape (N, 4)."
        )

    # --------------------------------------------------------
    # Timestamps
    # --------------------------------------------------------

    check_time_stream(
        "Robot",
        robot_time
    )

    check_time_stream(
        "Gripper",
        gripper_time
    )

    check_time_stream(
        "RGB",
        image_time
    )

    # --------------------------------------------------------
    # Finite-value checks
    # --------------------------------------------------------

    print("\n--- Numerical checks ---")

    check_nan("q", q)

    check_nan("dq", dq)

    check_nan(
        "EE position",
        ee_position
    )

    check_nan(
        "EE quaternion",
        quaternion
    )

    check_nan(
        "gripper width",
        gripper_width
    )

    # --------------------------------------------------------
    # Quaternion validation
    # --------------------------------------------------------

    print(
        "\n--- Quaternion validation ---"
    )

    quaternion_norm = np.linalg.norm(
        quaternion,
        axis=1
    )

    norm_error = np.abs(
        quaternion_norm - 1.0
    )

    print(
        "Mean quaternion norm: "
        f"{np.mean(quaternion_norm):.8f}"
    )

    print(
        "Maximum norm error:   "
        f"{np.max(norm_error):.8e}"
    )

    if len(quaternion) > 1:

        consecutive_dot = np.sum(
            quaternion[:-1]
            * quaternion[1:],
            axis=1
        )

        print(
            "Minimum consecutive "
            "quaternion dot product: "
            f"{np.min(consecutive_dot):.6f}"
        )

        sign_flips = np.sum(
            consecutive_dot < 0.0
        )

        if sign_flips == 0:

            print(
                "Quaternion sign continuity: OK"
            )

        else:

            print(
                "WARNING: "
                f"{sign_flips} quaternion "
                "sign flips detected."
            )

    # --------------------------------------------------------
    # Verify position against raw O_T_EE
    # --------------------------------------------------------

    print(
        "\n--- O_T_EE validation ---"
    )

    # Franka O_T_EE:
    #
    # indices 12, 13, 14 correspond to
    # x, y, z because it is column-major.

    position_from_transform = (
        O_T_EE[:, [12, 13, 14]]
    )

    position_error = np.abs(
        position_from_transform
        - ee_position
    )

    print(
        "Max EE position extraction error: "
        f"{np.max(position_error):.8e} m"
    )

    # --------------------------------------------------------
    # Verify gripper width
    # --------------------------------------------------------

    print(
        "\n--- Gripper validation ---"
    )

    if (
        gripper_joint_position.ndim == 2
        and
        gripper_joint_position.shape[1] >= 2
    ):

        calculated_width = (
            gripper_joint_position[:, 0]
            + gripper_joint_position[:, 1]
        )

        width_error = np.abs(
            calculated_width
            - gripper_width
        )

        print(
            "Maximum gripper-width error: "
            f"{np.max(width_error):.8e} m"
        )

    print(
        "Gripper width range: "
        f"{np.min(gripper_width):.4f} "
        "to "
        f"{np.max(gripper_width):.4f} m"
    )

    # --------------------------------------------------------
    # Trajectory step sizes
    # --------------------------------------------------------

    print(
        "\n--- Trajectory continuity ---"
    )

    if len(q) > 1:

        joint_step = np.abs(
            np.diff(
                q,
                axis=0
            )
        )

        print(
            "Largest joint-position step: "
            f"{np.max(joint_step):.6f} rad"
        )

    if len(ee_position) > 1:

        position_step = np.linalg.norm(
            np.diff(
                ee_position,
                axis=0
            ),
            axis=1
        )

        print(
            "Largest EE translation step: "
            f"{np.max(position_step):.6f} m"
        )

    # --------------------------------------------------------
    # Check RGB files
    # --------------------------------------------------------

    print(
        "\n--- RGB file validation ---"
    )

    missing_images = []

    for relative_path in image_paths:

        image_path = os.path.join(
            episode_dir,
            str(relative_path)
        )

        if not os.path.exists(
            image_path
        ):

            missing_images.append(
                image_path
            )

    if len(missing_images) == 0:

        print(
            f"All {len(image_paths)} "
            "RGB files exist."
        )

    else:

        print(
            "WARNING: "
            f"{len(missing_images)} "
            "RGB files are missing."
        )

    return data


# ============================================================
# Plot trajectories
# ============================================================

def plot_trajectories(
    episode_dir,
    data
):

    validation_dir = os.path.join(
        episode_dir,
        "validation"
    )

    os.makedirs(
        validation_dir,
        exist_ok=True
    )

    robot_time = data[
        "robot_time_relative"
    ]

    gripper_time = data[
        "gripper_time_relative"
    ]

    q = data["q"]

    dq = data["dq"]

    ee_position = data[
        "ee_position"
    ]

    quaternion = data[
        "ee_quaternion_wxyz"
    ]

    gripper_width = data[
        "gripper_width"
    ]

    # --------------------------------------------------------
    # Joint positions
    # --------------------------------------------------------

    plt.figure(
        figsize=(11, 6)
    )

    for joint_index in range(7):

        plt.plot(
            robot_time,
            q[:, joint_index],
            label=(
                f"panda_joint"
                f"{joint_index + 1}"
            )
        )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Joint position [rad]"
    )

    plt.title(
        "Kinesthetic demonstration: "
        "joint positions"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            validation_dir,
            "joint_positions.png"
        ),
        dpi=150
    )

    plt.close()

    # --------------------------------------------------------
    # Joint velocities
    # --------------------------------------------------------

    plt.figure(
        figsize=(11, 6)
    )

    for joint_index in range(7):

        plt.plot(
            robot_time,
            dq[:, joint_index],
            label=(
                f"panda_joint"
                f"{joint_index + 1}"
            )
        )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Joint velocity [rad/s]"
    )

    plt.title(
        "Kinesthetic demonstration: "
        "joint velocities"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            validation_dir,
            "joint_velocities.png"
        ),
        dpi=150
    )

    plt.close()

    # --------------------------------------------------------
    # EE position
    # --------------------------------------------------------

    plt.figure(
        figsize=(11, 6)
    )

    plt.plot(
        robot_time,
        ee_position[:, 0],
        label="x"
    )

    plt.plot(
        robot_time,
        ee_position[:, 1],
        label="y"
    )

    plt.plot(
        robot_time,
        ee_position[:, 2],
        label="z"
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Position [m]"
    )

    plt.title(
        "End-effector position"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            validation_dir,
            "ee_position.png"
        ),
        dpi=150
    )

    plt.close()

    # --------------------------------------------------------
    # Quaternion
    # --------------------------------------------------------

    plt.figure(
        figsize=(11, 6)
    )

    labels = [
        "w",
        "x",
        "y",
        "z"
    ]

    for index in range(4):

        plt.plot(
            robot_time,
            quaternion[:, index],
            label=labels[index]
        )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Quaternion component"
    )

    plt.title(
        "End-effector orientation "
        "[w, x, y, z]"
    )

    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            validation_dir,
            "ee_quaternion.png"
        ),
        dpi=150
    )

    plt.close()

    # --------------------------------------------------------
    # Gripper width
    # --------------------------------------------------------

    plt.figure(
        figsize=(11, 5)
    )

    plt.plot(
        gripper_time,
        gripper_width
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Gripper width [m]"
    )

    plt.title(
        "Franka gripper opening"
    )

    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            validation_dir,
            "gripper_width.png"
        ),
        dpi=150
    )

    plt.close()

    print(
        "\nTrajectory plots saved to:"
    )

    print(
        validation_dir
    )


# ============================================================
# RGB contact sheet
# ============================================================

def create_rgb_contact_sheet(
    episode_dir,
    data,
    number_images=8
):

    image_paths = data[
        "image_paths"
    ]

    image_time = data[
        "image_time_relative"
    ]

    if len(image_paths) == 0:
        return

    number_images = min(
        number_images,
        len(image_paths)
    )

    indices = np.linspace(
        0,
        len(image_paths) - 1,
        number_images
    ).astype(int)

    frames = []

    target_width = 320

    for index in indices:

        image_path = os.path.join(
            episode_dir,
            str(image_paths[index])
        )

        image = cv2.imread(
            image_path
        )

        if image is None:
            continue

        height, width = (
            image.shape[:2]
        )

        scale = (
            target_width / width
        )

        target_height = int(
            height * scale
        )

        image = cv2.resize(
            image,
            (
                target_width,
                target_height
            )
        )

        label = (
            f"t={image_time[index]:.2f}s"
        )

        cv2.putText(
            image,
            label,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        frames.append(
            image
        )

    if len(frames) == 0:
        return

    # Four images per row.
    images_per_row = 4

    rows = []

    for start in range(
        0,
        len(frames),
        images_per_row
    ):

        row_frames = frames[
            start:
            start + images_per_row
        ]

        while (
            len(row_frames)
            < images_per_row
        ):

            blank = np.zeros_like(
                frames[0]
            )

            row_frames.append(
                blank
            )

        row = cv2.hconcat(
            row_frames
        )

        rows.append(
            row
        )

    contact_sheet = cv2.vconcat(
        rows
    )

    validation_dir = os.path.join(
        episode_dir,
        "validation"
    )

    os.makedirs(
        validation_dir,
        exist_ok=True
    )

    output_path = os.path.join(
        validation_dir,
        "rgb_samples.jpg"
    )

    cv2.imwrite(
        output_path,
        contact_sheet
    )

    print(
        "RGB contact sheet saved to:"
    )

    print(
        output_path
    )


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Validate and visualize a "
            "kinesthetic Panda demonstration."
        )
    )

    parser.add_argument(
        "episode",
        help=(
            "Episode directory, e.g. "
            "demo_0001"
        )
    )

    args = parser.parse_args()

    episode_dir = os.path.abspath(
        os.path.expanduser(
            args.episode
        )
    )

    data = validate_episode(
        episode_dir
    )

    plot_trajectories(
        episode_dir,
        data
    )

    create_rgb_contact_sheet(
        episode_dir,
        data
    )

    print(
        "\nValidation complete."
    )


if __name__ == "__main__":
    main()
