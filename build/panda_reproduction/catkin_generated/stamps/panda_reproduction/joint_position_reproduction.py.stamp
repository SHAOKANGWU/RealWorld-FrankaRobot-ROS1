#!/usr/bin/env python3

"""Replay joint-position waypoints through the joint torque controller."""

import math
import os

import rospy
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState
from panda_custom_controllers.msg import JointPositionCommand


NUMBER_OF_JOINTS = 7
COMMAND_SIZE = NUMBER_OF_JOINTS + 1
POSE_SIZE = 7
TRAJECTORY_ROW_SIZE = POSE_SIZE + NUMBER_OF_JOINTS + 1
MINIMUM_QUATERNION_NORM = 1e-6


class JointPositionReproduction:
    def __init__(self):
        self.command_topic = rospy.get_param(
            "~command_topic",
            "/joint_position_torque_controller/desired_joint_position_command",
        )
        self.controller_name = rospy.get_param(
            "~controller_name", "joint_position_torque_controller"
        )
        self.publish_rate = self._positive_float_param("~publish_rate", 10.0)
        self.wait_for_subscriber_timeout = self._nonnegative_float_param(
            "~wait_for_subscriber_timeout", 0.0
        )
        self.loop = bool(rospy.get_param("~loop", False))
        self.hold_final_command = bool(
            rospy.get_param("~hold_final_command", True)
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
            JointPositionCommand,
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
            waypoints = self._read_trajectory_file(trajectory_file)
        else:
            waypoints = inline_trajectory

        if not isinstance(waypoints, (list, tuple)):
            raise ValueError("~trajectory must be a list of eight-value rows")
        if not waypoints:
            raise ValueError(
                "no trajectory configured; set ~trajectory or ~trajectory_file"
            )

        return [
            self._validate_waypoint(waypoint, index)
            for index, waypoint in enumerate(waypoints)
        ]

    @staticmethod
    def _read_trajectory_file(path):
        expanded_path = os.path.abspath(os.path.expanduser(path))
        waypoints = []
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
                    if len(row) != TRAJECTORY_ROW_SIZE:
                        raise ValueError(
                            "{}:{} must contain exactly {} values: pose (7), "
                            "joints (7), gripper (1)".format(
                                expanded_path, line_number, TRAJECTORY_ROW_SIZE
                            )
                        )
                    if not all(math.isfinite(value) for value in row):
                        raise ValueError(
                            "{}:{} contains a non-finite value".format(
                                expanded_path, line_number
                            )
                        )
                    quaternion_norm = math.sqrt(
                        sum(value * value for value in row[3:POSE_SIZE])
                    )
                    if quaternion_norm < MINIMUM_QUATERNION_NORM:
                        raise ValueError(
                            "{}:{} has a zero quaternion".format(
                                expanded_path, line_number
                            )
                        )
                    waypoints.append(
                        row[POSE_SIZE:POSE_SIZE + NUMBER_OF_JOINTS] + [row[-1]]
                    )
        except OSError as error:
            raise ValueError(
                "cannot read trajectory file '{}': {}".format(expanded_path, error)
            ) from error

        return waypoints

    @staticmethod
    def _validate_waypoint(waypoint, index):
        if not isinstance(waypoint, (list, tuple)):
            raise ValueError("trajectory waypoint {} must be a list".format(index))
        if len(waypoint) != COMMAND_SIZE:
            raise ValueError(
                "trajectory waypoint {} must contain seven positions followed by "
                "one gripper value".format(index)
            )

        positions = []
        for joint_index, value in enumerate(waypoint[:NUMBER_OF_JOINTS]):
            try:
                position = float(value)
            except (TypeError, ValueError) as error:
                raise ValueError(
                    "trajectory waypoint {}, joint {} is not numeric".format(
                        index, joint_index + 1
                    )
                ) from error
            if not math.isfinite(position):
                raise ValueError(
                    "trajectory waypoint {}, joint {} is not finite".format(
                        index, joint_index + 1
                    )
                )
            positions.append(position)

        try:
            gripper_value = float(waypoint[-1])
        except (TypeError, ValueError) as error:
            raise ValueError(
                "trajectory waypoint {} has a non-numeric gripper value".format(index)
            ) from error
        if not math.isfinite(gripper_value) or gripper_value not in (0.0, 1.0):
            raise ValueError(
                "trajectory waypoint {} gripper value must be 0 or 1".format(index)
            )
        return positions + [bool(gripper_value)]

    def _wait_for_controller(self):
        start_time = rospy.get_time()
        wait_rate = rospy.Rate(10.0)
        service_name = "/controller_manager/list_controllers"
        list_controllers = rospy.ServiceProxy(service_name, ListControllers)
        while not rospy.is_shutdown():
            connected = self.publisher.get_num_connections() > 0
            running = False
            if connected:
                try:
                    rospy.wait_for_service(service_name, timeout=0.1)
                    running = any(
                        controller.name == self.controller_name
                        and controller.state == "running"
                        for controller in list_controllers().controller
                    )
                except (rospy.ROSException, rospy.ServiceException):
                    pass
            if connected and running:
                return
            if (
                self.wait_for_subscriber_timeout > 0.0
                and rospy.get_time() - start_time >= self.wait_for_subscriber_timeout
            ):
                raise RuntimeError(
                    "controller '{}' is not running and connected to '{}' after {:.1f} seconds".format(
                        self.controller_name,
                        self.command_topic,
                        self.wait_for_subscriber_timeout,
                    )
                )
            rospy.loginfo_throttle(
                5.0,
                "Waiting for controller %s to run and subscribe to %s",
                self.controller_name,
                self.command_topic,
            )
            wait_rate.sleep()

    def _check_start_joints(self):
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

        target = self.trajectory[0][:NUMBER_OF_JOINTS]
        errors = [abs(actual - desired) for actual, desired in zip(state.q, target)]
        maximum_error = max(errors)
        if maximum_error > self.start_joint_tolerance:
            joint_index = errors.index(maximum_error) + 1
            raise RuntimeError(
                "robot is not at the recorded start joints: joint {} differs by "
                "{:.4f} rad (limit {:.4f} rad); run move_to_start_joint first".format(
                    joint_index, maximum_error, self.start_joint_tolerance
                )
            )

    def _publish(self, command):
        message = JointPositionCommand()
        message.header.seq = self.sequence_id
        message.header.stamp = rospy.Time.now()
        message.joint_positions = command[:NUMBER_OF_JOINTS]
        message.gripper_grasp = command[-1]
        self.publisher.publish(message)
        self.sequence_id = (self.sequence_id + 1) & 0xFFFFFFFF

    def run(self):
        self._wait_for_controller()
        if rospy.is_shutdown():
            return
        self._check_start_joints()

        rospy.loginfo(
            "Replaying %d joint/gripper commands at %.3f Hz on %s",
            len(self.trajectory),
            self.publish_rate,
            self.command_topic,
        )
        rospy.loginfo("Waypoint order: %s", ", ".join(self.joint_names))

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
            rospy.loginfo("Trajectory complete; holding the final arm/gripper command")
            final_command = self.trajectory[-1]
            while not rospy.is_shutdown():
                self._publish(final_command)
                rate.sleep()
        else:
            rospy.loginfo(
                "Trajectory complete; the controller retains the final command"
            )


def main():
    rospy.init_node("joint_position_reproduction")
    try:
        reproduction = JointPositionReproduction()
        reproduction.run()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("Joint-position reproduction failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
