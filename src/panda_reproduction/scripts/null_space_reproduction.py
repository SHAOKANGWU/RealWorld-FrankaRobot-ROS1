#!/usr/bin/env python3

"""Replay synchronized Cartesian and joint targets through the null-space controller."""

import math
import os

import numpy as np
import rospy
from franka_msgs.msg import FrankaState
from panda_custom_controllers.msg import NullSpaceCommand
from tf.transformations import quaternion_from_matrix


NUMBER_OF_JOINTS = 7
POSE_SIZE = 7
ARM_COMMAND_SIZE = POSE_SIZE + NUMBER_OF_JOINTS
COMMAND_SIZE = ARM_COMMAND_SIZE + 1
QUATERNION_START = 3
MINIMUM_QUATERNION_NORM = 1e-6


class NullSpaceReproduction:
    def __init__(self):
        self.command_topic = rospy.get_param(
            "~command_topic",
            "/null_space_torque_controller/desired_null_space_command",
        )
        self.publish_rate = self._positive_float_param("~publish_rate", 10.0)
        self.wait_for_subscriber_timeout = self._nonnegative_float_param(
            "~wait_for_subscriber_timeout", 0.0
        )
        self.loop = bool(rospy.get_param("~loop", False))
        self.hold_final_command = bool(
            rospy.get_param("~hold_final_command", True)
        )
        self.base_frame = str(rospy.get_param("~base_frame", "panda_link0")).strip()
        if not self.base_frame:
            raise ValueError("~base_frame must not be empty")
        self.start_position_tolerance = self._positive_float_param(
            "~start_position_tolerance", 0.01
        )
        self.start_orientation_tolerance = self._positive_float_param(
            "~start_orientation_tolerance", 0.05
        )
        self.start_joint_tolerance = self._positive_float_param(
            "~start_joint_tolerance", 0.02
        )

        self.joint_names = rospy.get_param(
            "~joint_names",
            ["panda_joint{}".format(index) for index in range(1, 8)],
        )
        if len(self.joint_names) != NUMBER_OF_JOINTS:
            raise ValueError("~joint_names must contain exactly seven names")
        if len(set(self.joint_names)) != NUMBER_OF_JOINTS:
            raise ValueError("~joint_names must not contain duplicate names")

        self.trajectory = self._load_trajectory()
        self.sequence_id = 0
        self.publisher = rospy.Publisher(
            self.command_topic,
            NullSpaceCommand,
            queue_size=1,
        )

    @staticmethod
    def _positive_float_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and greater than zero".format(name))
        return value

    @staticmethod
    def _nonnegative_float_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("{} must be finite and non-negative".format(name))
        return value

    def _load_trajectory(self):
        inline_trajectory = rospy.get_param("~trajectory", [])
        trajectory_file = str(rospy.get_param("~trajectory_file", "")).strip()

        if inline_trajectory and trajectory_file:
            raise ValueError("set only one of ~trajectory and ~trajectory_file")

        if trajectory_file:
            commands = self._read_trajectory_file(trajectory_file)
        else:
            commands = inline_trajectory

        if not isinstance(commands, (list, tuple)):
            raise ValueError("~trajectory must be a list of fifteen-value rows")
        if not commands:
            raise ValueError(
                "no trajectory configured; set ~trajectory or ~trajectory_file"
            )

        return [
            self._validate_command(command, index)
            for index, command in enumerate(commands)
        ]

    @staticmethod
    def _read_trajectory_file(path):
        expanded_path = os.path.abspath(os.path.expanduser(path))
        commands = []
        try:
            with open(expanded_path, "r") as trajectory_stream:
                for line_number, line in enumerate(trajectory_stream, start=1):
                    content = line.split("#", 1)[0].strip()
                    if not content:
                        continue
                    fields = content.replace(",", " ").split()
                    try:
                        row = [float(field) for field in fields]
                    except ValueError as error:
                        raise ValueError(
                            "{}:{} contains a non-numeric value".format(
                                expanded_path, line_number
                            )
                        ) from error
                    if len(row) != COMMAND_SIZE:
                        raise ValueError(
                            "{}:{} must contain exactly {} values: pose (7), "
                            "joints (7), gripper (1)".format(
                                expanded_path, line_number, COMMAND_SIZE
                            )
                        )
                    if not all(math.isfinite(value) for value in row):
                        raise ValueError(
                            "{}:{} contains a non-finite value".format(
                                expanded_path, line_number
                            )
                        )
                    commands.append(row)
        except OSError as error:
            raise ValueError(
                "cannot read trajectory file '{}': {}".format(expanded_path, error)
            ) from error

        return commands

    @staticmethod
    def _validate_command(command, index):
        if not isinstance(command, (list, tuple)):
            raise ValueError("trajectory command {} must be a list".format(index))
        if len(command) != COMMAND_SIZE:
            raise ValueError(
                "trajectory command {} must contain exactly fifteen values: "
                "[x, y, z, qx, qy, qz, qw, joint1, ..., joint7, gripper]".format(
                    index
                )
            )

        values = []
        for element_index, value in enumerate(command[:ARM_COMMAND_SIZE]):
            try:
                value = float(value)
            except (TypeError, ValueError) as error:
                raise ValueError(
                    "trajectory command {}, element {} is not numeric".format(
                        index, element_index + 1
                    )
                ) from error
            if not math.isfinite(value):
                raise ValueError(
                    "trajectory command {}, element {} is not finite".format(
                        index, element_index + 1
                    )
                )
            values.append(value)

        quaternion_norm = math.sqrt(
            sum(value * value for value in values[QUATERNION_START:POSE_SIZE])
        )
        if quaternion_norm < MINIMUM_QUATERNION_NORM:
            raise ValueError(
                "trajectory command {} has a zero quaternion".format(index)
            )
        values[QUATERNION_START:POSE_SIZE] = [
            value / quaternion_norm
            for value in values[QUATERNION_START:POSE_SIZE]
        ]

        try:
            gripper_value = float(command[-1])
        except (TypeError, ValueError) as error:
            raise ValueError(
                "trajectory command {} has a non-numeric gripper value".format(index)
            ) from error
        if not math.isfinite(gripper_value) or gripper_value not in (0.0, 1.0):
            raise ValueError(
                "trajectory command {} gripper value must be 0 or 1".format(index)
            )
        return values + [bool(gripper_value)]

    def _wait_for_controller(self):
        start_time = rospy.get_time()
        wait_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if (
                self.wait_for_subscriber_timeout > 0.0
                and rospy.get_time() - start_time >= self.wait_for_subscriber_timeout
            ):
                raise RuntimeError(
                    "no subscriber connected to '{}' within {:.1f} seconds".format(
                        self.command_topic, self.wait_for_subscriber_timeout
                    )
                )
            rospy.loginfo_throttle(
                5.0,
                "Waiting for the null-space torque controller on %s",
                self.command_topic,
            )
            wait_rate.sleep()

    def _check_start_state(self):
        try:
            state = rospy.wait_for_message(
                "/franka_state_controller/franka_states",
                FrankaState,
                timeout=5.0,
            )
        except rospy.ROSException as error:
            raise RuntimeError("no Franka state received before replay") from error

        if state.header.stamp != rospy.Time():
            age = (rospy.Time.now() - state.header.stamp).to_sec()
            if abs(age) > 1.0:
                raise RuntimeError(
                    "Franka state timestamp is not current ({:.2f} s)".format(age)
                )

        if len(state.q) != NUMBER_OF_JOINTS or not all(
            math.isfinite(value) for value in state.q
        ):
            raise RuntimeError("Franka state contains invalid joint positions")

        transform = np.asarray(state.O_T_EE, dtype=float).reshape((4, 4), order="F")
        rotation = transform[:3, :3]
        if (not np.all(np.isfinite(transform))
                or not np.allclose(transform[3, :], [0, 0, 0, 1], atol=1e-3)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)):
            raise RuntimeError("Franka state contains an invalid end-effector pose")

        first_command = self.trajectory[0]
        distance = float(np.linalg.norm(transform[:3, 3] - first_command[:3]))
        actual_quaternion = quaternion_from_matrix(transform)
        quaternion_dot = abs(float(np.dot(actual_quaternion, first_command[3:POSE_SIZE])))
        angle = 2.0 * math.acos(min(1.0, quaternion_dot))

        target_joints = first_command[POSE_SIZE:ARM_COMMAND_SIZE]
        joint_errors = [
            abs(actual - desired)
            for actual, desired in zip(state.q, target_joints)
        ]
        maximum_joint_error = max(joint_errors)

        if (distance > self.start_position_tolerance
                or angle > self.start_orientation_tolerance
                or maximum_joint_error > self.start_joint_tolerance):
            raise RuntimeError(
                "robot is not at the recorded null-space start state: {:.4f} m, "
                "{:.4f} rad orientation, {:.4f} rad maximum joint error "
                "(limits {:.4f} m, {:.4f} rad, {:.4f} rad); run "
                "move_to_start_joint first".format(
                    distance,
                    angle,
                    maximum_joint_error,
                    self.start_position_tolerance,
                    self.start_orientation_tolerance,
                    self.start_joint_tolerance,
                )
            )

    def _publish(self, command):
        message = NullSpaceCommand()
        message.header.seq = self.sequence_id
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.base_frame

        message.pose.position.x = command[0]
        message.pose.position.y = command[1]
        message.pose.position.z = command[2]
        message.pose.orientation.x = command[3]
        message.pose.orientation.y = command[4]
        message.pose.orientation.z = command[5]
        message.pose.orientation.w = command[6]
        message.joint_positions = command[POSE_SIZE:ARM_COMMAND_SIZE]
        message.gripper_grasp = command[-1]

        self.publisher.publish(message)
        self.sequence_id = (self.sequence_id + 1) & 0xFFFFFFFF

    def run(self):
        self._wait_for_controller()
        if rospy.is_shutdown():
            return
        self._check_start_state()

        rospy.loginfo(
            "Replaying %d synchronized arm/gripper commands at %.3f Hz in frame %s on %s",
            len(self.trajectory),
            self.publish_rate,
            self.base_frame,
            self.command_topic,
        )
        rospy.loginfo("Joint order: %s", ", ".join(self.joint_names))

        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            for command in self.trajectory:
                if rospy.is_shutdown():
                    return
                self._publish(command)
                rate.sleep()

            if not self.loop:
                break

        if self.hold_final_command:
            rospy.loginfo("Trajectory complete; holding the final synchronized command")
            final_command = self.trajectory[-1]
            while not rospy.is_shutdown():
                self._publish(final_command)
                rate.sleep()
        else:
            rospy.loginfo(
                "Trajectory complete; the controller retains the final command"
            )


def main():
    rospy.init_node("null_space_reproduction")
    try:
        reproduction = NullSpaceReproduction()
        reproduction.run()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("Null-space reproduction failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
