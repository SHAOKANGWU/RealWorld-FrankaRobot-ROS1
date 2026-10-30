#!/usr/bin/env python3

import rospy
import actionlib

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from panda_spacemouse_teleop.msg import GripperCommand

from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    MoveAction,
    MoveGoal,
)


class SpaceMouseMapper:
    def __init__(self):

        # ==================================================
        # SpaceMouse / arm parameters
        # ==================================================

        self.max_linear_velocity = rospy.get_param(
            "~max_linear_velocity",
            0.03
        )

        self.max_angular_velocity = rospy.get_param(
            "~max_angular_velocity",
            0.20
        )

        self.command_frame = rospy.get_param(
            "~command_frame",
            "panda_link0"
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate",
            50.0
        )

        self.input_timeout = rospy.get_param(
            "~input_timeout",
            0.2
        )

        self.joy_timeout = rospy.get_param("~joy_timeout", 0.5)
        self.require_deadman = rospy.get_param("~require_deadman", False)
        self.deadman_buttons = rospy.get_param("~deadman_buttons", [0, 1])
        self.twist_topic = rospy.get_param("~twist_topic", "/spacenav/twist")
        self.joy_topic = rospy.get_param("~joy_topic", "/spacenav/joy")
        self.command_topic = rospy.get_param(
            "~command_topic", "/teleop/cartesian_velocity_cmd"
        )
        self.gripper_command_topic = rospy.get_param(
            "~gripper_command_topic", "/teleop/gripper_command"
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.input_timeout <= 0.0 or self.joy_timeout <= 0.0:
            raise ValueError("input_timeout and joy_timeout must be positive")
        if self.max_linear_velocity < 0.0 or self.max_angular_velocity < 0.0:
            raise ValueError("velocity limits must be non-negative")
        if not isinstance(self.deadman_buttons, list):
            raise ValueError("deadman_buttons must be a YAML list")

        # ==================================================
        # Gripper parameters
        # ==================================================

        # Treat these two buttons as one combined trigger.
        self.gripper_button_1 = rospy.get_param(
            "~gripper_button_1",
            0
        )

        self.gripper_button_2 = rospy.get_param(
            "~gripper_button_2",
            1
        )

        # Gripper opening width [m].
        # Panda maximum opening is normally around 0.08 m.
        self.open_width = rospy.get_param(
            "~gripper_open_width",
            0.08
        )

        # Expected width of the object when grasped [m].
        # Change this according to your object.
        self.grasp_width = rospy.get_param(
            "~gripper_grasp_width",
            0.00
        )

        self.gripper_speed = rospy.get_param(
            "~gripper_speed",
            0.05
        )

        self.gripper_force = rospy.get_param(
            "~gripper_force",
            20.0
        )

        self.grasp_epsilon_inner = rospy.get_param(
            "~grasp_epsilon_inner",
            0.005
        )

        self.grasp_epsilon_outer = rospy.get_param(
            "~grasp_epsilon_outer",
            0.08
        )

        self.enable_gripper = rospy.get_param("~enable_gripper", True)

        # ==================================================
        # Internal state
        # ==================================================

        self.latest_twist = Twist()

        self.last_twist_time = rospy.Time(0)
        self.last_joy_time = rospy.Time(0)
        self.deadman_pressed = False
        self.gripper_gesture_active = False

        # False = gripper open
        # True  = gripper commanded to grasp
        self.gripper_closed = False

        # Used for rising-edge detection of:
        #
        # both buttons not pressed
        #          ↓
        # both buttons pressed
        #
        self.previous_both_pressed = False

        # ==================================================
        # SpaceMouse subscribers
        # ==================================================

        self.twist_sub = rospy.Subscriber(
            self.twist_topic,
            Twist,
            self.twist_callback,
            queue_size=1
        )

        self.joy_sub = rospy.Subscriber(
            self.joy_topic,
            Joy,
            self.joy_callback,
            queue_size=1
        )

        # ==================================================
        # Cartesian velocity publisher
        # ==================================================

        self.command_pub = rospy.Publisher(
            self.command_topic,
            TwistStamped,
            queue_size=1
        )

        # A processed, bag-friendly record of each requested gripper action.
        self.gripper_command_pub = rospy.Publisher(
            self.gripper_command_topic,
            GripperCommand,
            queue_size=10
        )

        # ==================================================
        # Franka gripper action clients
        # ==================================================

        self.grasp_client = None
        self.move_client = None
        self.gripper_available = False

        if self.enable_gripper:
            self.grasp_client = actionlib.SimpleActionClient(
                "/franka_gripper/grasp", GraspAction
            )
            self.move_client = actionlib.SimpleActionClient(
                "/franka_gripper/move", MoveAction
            )
            rospy.loginfo("Waiting for Franka gripper action servers...")
            grasp_available = self.grasp_client.wait_for_server(rospy.Duration(5.0))
            move_available = self.move_client.wait_for_server(rospy.Duration(5.0))
            self.gripper_available = grasp_available and move_available

        if self.gripper_available:
            rospy.loginfo(
                "Franka gripper action servers connected."
            )
        elif self.enable_gripper:
            rospy.logwarn(
                "Franka gripper action servers are not available. "
                "Arm teleoperation will still work."
            )

        # ==================================================
        # Fixed-rate arm command publisher
        # ==================================================

        self.timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.publish_rate
            ),
            self.publish_command
        )
        rospy.on_shutdown(self.publish_zero_command)

        rospy.loginfo(
            "SpaceMouse mapper initialized."
        )


    # ======================================================
    # SpaceMouse motion
    # ======================================================

    def twist_callback(self, msg):

        self.latest_twist = msg

        self.last_twist_time = rospy.Time.now()


    # ======================================================
    # SpaceMouse buttons
    # ======================================================

    def joy_callback(self, msg):

        self.last_joy_time = rospy.Time.now()
        self.gripper_gesture_active = False
        valid_deadman_buttons = [
            index for index in self.deadman_buttons
            if isinstance(index, int) and 0 <= index < len(msg.buttons)
        ]
        self.deadman_pressed = any(
            msg.buttons[index] != 0 for index in valid_deadman_buttons
        )

        # Check that both requested button indices exist.
        if (
            self.gripper_button_1 < 0
            or self.gripper_button_2 < 0
            or self.gripper_button_1 >= len(msg.buttons)
            or self.gripper_button_2 >= len(msg.buttons)
        ):
            rospy.logwarn_throttle(
                2.0, "Gripper button index is not present in /spacenav/joy"
            )
            return

        button_1_pressed = (
            msg.buttons[self.gripper_button_1] != 0
        )

        button_2_pressed = (
            msg.buttons[self.gripper_button_2] != 0
        )

        # --------------------------------------------------
        # Treat both buttons as one combined button
        # --------------------------------------------------

        both_pressed = (
            button_1_pressed
            and button_2_pressed
        )

        self.gripper_gesture_active = both_pressed

        # A two-button gripper gesture must never enable arm motion.
        if both_pressed:
            self.deadman_pressed = False

        # --------------------------------------------------
        # Rising-edge detection
        #
        # False -> True
        #
        # This means the combined two-button command
        # triggers only once for each press.
        # --------------------------------------------------

        if (
            both_pressed
            and not self.previous_both_pressed
        ):

            self.toggle_gripper()

        self.previous_both_pressed = (
            both_pressed
        )


    # ======================================================
    # Gripper toggle
    # ======================================================

    def toggle_gripper(self):

        if not self.gripper_available:

            rospy.logwarn(
                "Cannot control gripper: "
                "gripper action server unavailable."
            )

            return

        if self.gripper_closed:

            self.open_gripper()

        else:

            self.grasp_object()


    # ======================================================
    # Grasp
    # ======================================================

    def grasp_object(self):

        rospy.loginfo(
            "Two-button command detected: GRASP"
        )

        goal = GraspGoal()

        goal.width = self.grasp_width
        goal.speed = self.gripper_speed
        goal.force = self.gripper_force

        goal.epsilon.inner = (
            self.grasp_epsilon_inner
        )

        goal.epsilon.outer = (
            self.grasp_epsilon_outer
        )

        self.publish_gripper_command(
            GripperCommand.GRASP,
            self.grasp_width,
            self.gripper_speed,
            self.gripper_force,
            self.grasp_epsilon_inner,
            self.grasp_epsilon_outer
        )

        self.grasp_client.send_goal(
            goal,
            done_cb=self.grasp_done_callback
        )


    def grasp_done_callback(
        self,
        state,
        result
    ):

        if result.success:

            self.gripper_closed = True

            rospy.loginfo(
                "Gripper grasp succeeded."
            )

        else:

            rospy.logwarn(
                "Gripper grasp failed."
            )


    # ======================================================
    # Open gripper
    # ======================================================

    def open_gripper(self):

        rospy.loginfo(
            "Two-button command detected: OPEN"
        )

        goal = MoveGoal()

        goal.width = self.open_width
        goal.speed = self.gripper_speed

        self.publish_gripper_command(
            GripperCommand.OPEN,
            self.open_width,
            self.gripper_speed,
            0.0,
            0.0,
            0.0
        )

        self.move_client.send_goal(
            goal,
            done_cb=self.open_done_callback
        )


    def open_done_callback(
        self,
        state,
        result
    ):

        if result.success:

            self.gripper_closed = False

            rospy.loginfo(
                "Gripper opened successfully."
            )

        else:

            rospy.logwarn(
                "Gripper opening failed."
            )

    def publish_gripper_command(
        self,
        command_type,
        width,
        speed,
        force,
        epsilon_inner,
        epsilon_outer
    ):
        """Publish the processed gripper intent for recording and learning."""
        command = GripperCommand()
        command.header.stamp = rospy.Time.now()
        command.command = command_type
        command.width = width
        command.speed = speed
        command.force = force
        command.epsilon_inner = epsilon_inner
        command.epsilon_outer = epsilon_outer
        self.gripper_command_pub.publish(command)


    # ======================================================
    # Cartesian velocity publisher
    # ======================================================

    def publish_command(self, event):

        command = TwistStamped()

        command.header.stamp = rospy.Time.now()

        command.header.frame_id = (
            self.command_frame
        )

        # --------------------------------------------------
        # Check whether SpaceMouse input is recent
        # --------------------------------------------------

        input_age = (
            rospy.Time.now()
            - self.last_twist_time
        ).to_sec()

        input_is_fresh = (
            input_age <= self.input_timeout
        )

        joy_is_fresh = (
            (rospy.Time.now() - self.last_joy_time).to_sec()
            <= self.joy_timeout
        )
        motion_enabled = (
            input_is_fresh
            and not self.gripper_gesture_active
            and (
                not self.require_deadman
                or (joy_is_fresh and self.deadman_pressed)
            )
        )

        # --------------------------------------------------
        # Convert normalized SpaceMouse input
        # to Cartesian velocity
        # --------------------------------------------------

        if motion_enabled:

            command.twist.linear.x = (
                self.max_linear_velocity
                * self.latest_twist.linear.x
            )

            command.twist.linear.y = (
                self.max_linear_velocity
                * self.latest_twist.linear.y
            )

            command.twist.linear.z = (
                self.max_linear_velocity
                * self.latest_twist.linear.z
            )

            command.twist.angular.x = (
                self.max_angular_velocity
                * self.latest_twist.angular.x
            )

            command.twist.angular.y = (
                self.max_angular_velocity
                * self.latest_twist.angular.y
            )

            command.twist.angular.z = (
                self.max_angular_velocity
                * self.latest_twist.angular.z
            )

        # If input is stale, TwistStamped remains zero.

        self.command_pub.publish(
            command
        )

    def publish_zero_command(self):
        """Best-effort stop command when this mapper shuts down."""
        command = TwistStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.command_frame
        for _ in range(3):
            self.command_pub.publish(command)
            rospy.sleep(0.01)


def main():

    rospy.init_node(
        "spacemouse_mapper"
    )

    SpaceMouseMapper()

    rospy.spin()


if __name__ == "__main__":
    main()
