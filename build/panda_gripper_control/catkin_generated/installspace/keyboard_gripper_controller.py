#!/usr/bin/env python3

import select
import sys
import termios
import threading
import tty

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    MoveAction,
    MoveGoal,
    StopAction,
    StopGoal,
)


class KeyboardGripperController:
    def __init__(self):
        self.grasp_action_name = rospy.get_param("~grasp_action_name")
        self.move_action_name = rospy.get_param("~move_action_name")
        self.stop_action_name = rospy.get_param("~stop_action_name")

        self.open_width = float(rospy.get_param("~open_width"))
        self.grasp_width = float(rospy.get_param("~grasp_width"))
        self.speed = float(rospy.get_param("~speed"))
        self.force = float(rospy.get_param("~force"))
        self.epsilon_inner = float(rospy.get_param("~epsilon_inner"))
        self.epsilon_outer = float(rospy.get_param("~epsilon_outer"))

        self.server_timeout = float(rospy.get_param("~server_timeout"))
        self.stop_timeout = float(rospy.get_param("~stop_timeout"))
        self.key_poll_period = float(rospy.get_param("~key_poll_period"))

        self.grasp_key = self._read_key_parameter("~grasp_key")
        self.release_key = self._read_key_parameter("~release_key")
        self.stop_key = self._read_key_parameter("~stop_key")
        self.quit_key = self._read_key_parameter("~quit_key")

        self._validate_parameters()

        self._command_lock = threading.Lock()
        self._active_command = None
        self._command_generation = 0

        self.grasp_client = actionlib.SimpleActionClient(
            self.grasp_action_name,
            GraspAction,
        )
        self.move_client = actionlib.SimpleActionClient(
            self.move_action_name,
            MoveAction,
        )
        self.stop_client = actionlib.SimpleActionClient(
            self.stop_action_name,
            StopAction,
        )

        self._wait_for_action_servers()

    @staticmethod
    def _read_key_parameter(parameter_name):
        value = str(rospy.get_param(parameter_name)).lower()
        if len(value) != 1:
            raise ValueError("{} must contain one character".format(parameter_name))
        return value

    def _validate_parameters(self):
        if not 0.0 <= self.grasp_width <= self.open_width:
            raise ValueError(
                "grasp_width must be between zero and open_width"
            )
        if self.speed <= 0.0:
            raise ValueError("speed must be positive")
        if self.force <= 0.0:
            raise ValueError("force must be positive")
        if self.epsilon_inner < 0.0 or self.epsilon_outer < 0.0:
            raise ValueError("grasp epsilon values must be non-negative")
        if self.server_timeout <= 0.0 or self.stop_timeout <= 0.0:
            raise ValueError("action timeouts must be positive")
        if self.key_poll_period <= 0.0:
            raise ValueError("key_poll_period must be positive")

        keys = {
            self.grasp_key,
            self.release_key,
            self.stop_key,
            self.quit_key,
        }
        if len(keys) != 4:
            raise ValueError("keyboard command keys must be distinct")

    def _wait_for_action_servers(self):
        servers = (
            (self.grasp_action_name, self.grasp_client),
            (self.move_action_name, self.move_client),
            (self.stop_action_name, self.stop_client),
        )

        rospy.loginfo("Waiting for Franka gripper action servers...")
        for action_name, client in servers:
            if not client.wait_for_server(rospy.Duration(self.server_timeout)):
                raise rospy.ROSException(
                    "Timed out waiting for action server {}".format(action_name)
                )

        rospy.loginfo("Connected to all Franka gripper action servers")

    def _begin_command(self, label):
        with self._command_lock:
            if self._active_command is not None:
                rospy.logwarn(
                    "Cannot start %s while %s is active; press '%s' to stop it",
                    label,
                    self._active_command,
                    self.stop_key,
                )
                return None

            self._command_generation += 1
            generation = self._command_generation
            self._active_command = label
            return generation

    def _command_done(self, label, generation, state, result):
        with self._command_lock:
            if generation != self._command_generation:
                return
            self._active_command = None

        succeeded = (
            state == GoalStatus.SUCCEEDED
            and result is not None
            and result.success
        )
        if succeeded:
            rospy.loginfo("Gripper %s command succeeded", label)
        else:
            error = getattr(result, "error", "") if result is not None else ""
            rospy.logwarn(
                "Gripper %s command failed (action state %d): %s",
                label,
                state,
                error,
            )

    def grasp(self):
        generation = self._begin_command("grasp")
        if generation is None:
            return

        goal = GraspGoal()
        goal.width = self.grasp_width
        goal.speed = self.speed
        goal.force = self.force
        goal.epsilon.inner = self.epsilon_inner
        goal.epsilon.outer = self.epsilon_outer

        rospy.loginfo(
            "Grasping: width=%.3f m, speed=%.3f m/s, force=%.1f N",
            goal.width,
            goal.speed,
            goal.force,
        )
        self.grasp_client.send_goal(
            goal,
            done_cb=lambda state, result: self._command_done(
                "grasp", generation, state, result
            ),
        )

    def release(self):
        generation = self._begin_command("release")
        if generation is None:
            return

        goal = MoveGoal()
        goal.width = self.open_width
        goal.speed = self.speed

        rospy.loginfo(
            "Releasing: width=%.3f m, speed=%.3f m/s",
            goal.width,
            goal.speed,
        )
        self.move_client.send_goal(
            goal,
            done_cb=lambda state, result: self._command_done(
                "release", generation, state, result
            ),
        )

    def stop(self, wait_for_result=False):
        with self._command_lock:
            had_active_command = self._active_command is not None
            self._command_generation += 1
            generation = self._command_generation
            self._active_command = None

        self.grasp_client.cancel_all_goals()
        self.move_client.cancel_all_goals()

        if not had_active_command and not wait_for_result:
            rospy.loginfo("No gripper motion command is active")
            return

        with self._command_lock:
            self._active_command = "stop"

        rospy.loginfo("Stopping gripper motion")
        self.stop_client.send_goal(
            StopGoal(),
            done_cb=lambda state, result: self._command_done(
                "stop", generation, state, result
            ),
        )
        if wait_for_result:
            self.stop_client.wait_for_result(rospy.Duration(self.stop_timeout))

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "Keyboard input requires an interactive terminal. "
                "Run keyboard_gripper.launch in its own terminal."
            )

        controls = (
            "Keyboard gripper controls: "
            "[{0}] grasp, [{1}] release, [{2}] stop, [{3}] quit"
        ).format(
            self.grasp_key,
            self.release_key,
            self.stop_key,
            self.quit_key,
        )
        rospy.loginfo(controls)

        terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while not rospy.is_shutdown():
                readable, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    self.key_poll_period,
                )
                if not readable:
                    continue

                key = sys.stdin.read(1).lower()
                if key == self.grasp_key:
                    self.grasp()
                elif key == self.release_key:
                    self.release()
                elif key == self.stop_key:
                    self.stop()
                elif key == self.quit_key:
                    break
        finally:
            try:
                self.stop(wait_for_result=True)
            finally:
                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    terminal_settings,
                )


def main():
    rospy.init_node("keyboard_gripper_controller")

    try:
        controller = KeyboardGripperController()
        controller.run()
    except (KeyError, ValueError, RuntimeError, rospy.ROSException) as error:
        rospy.logfatal("Keyboard gripper controller failed: %s", error)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
