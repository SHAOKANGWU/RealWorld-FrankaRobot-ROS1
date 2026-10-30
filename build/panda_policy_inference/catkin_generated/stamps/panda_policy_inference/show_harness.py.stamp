#!/usr/bin/env python3
"""ROS adaptation of Show-Harness's single-arm, zero-shot agent (Sec. 3, App. 7.4).

The VLM emits semantic units, never robot poses. This interpreter supplies all
metric increments. No model call or robot command runs in a sensor callback.
"""

import base64
from collections import deque
import json
import math
import os
import sys
import threading
import time
import uuid

import cv2
import numpy as np
import rospy
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from observation_receiver.msg import MultiCameraSynchronizedObservation
from tf.transformations import (quaternion_about_axis, quaternion_from_matrix, quaternion_matrix,
                                quaternion_multiply, quaternion_slerp)

PANDA_JOINTS = ["panda_joint{}".format(index) for index in range(1, 8)]


def finite_vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError("{} must contain {} finite numbers".format(name, size))
    return result


def rigid_transform(values, name):
    transform = finite_vector(values, 16, name).reshape(4, 4, order="F")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-3)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)):
        raise ValueError("{} is not a rigid transform".format(name))
    return transform


def pose_from_state(state):
    """libfranka O_T_EE is column-major and expressed in the robot base."""
    transform = finite_vector(state.O_T_EE, 16, "O_T_EE").reshape(4, 4, order="F")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-3)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)):
        raise ValueError("O_T_EE is not a rigid transform")
    return transform[:3, 3].copy(), quaternion_from_matrix(transform)


def matrix_from_pose(pose):
    transform = quaternion_matrix([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ])
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return transform


def pose_errors(actual, desired):
    position_error = float(np.linalg.norm(actual[:3, 3] - desired[:3, 3]))
    cosine = (np.trace(desired[:3, :3].T @ actual[:3, :3]) - 1.0) / 2.0
    orientation_error = math.acos(float(np.clip(cosine, -1.0, 1.0)))
    return position_error, orientation_error


def orientation_distance(a, b):
    return 2.0 * math.acos(float(np.clip(abs(np.dot(a, b)), 0.0, 1.0)))


def check_workspace(position, lower, upper):
    if not np.all(np.isfinite(position)) or np.any(position < lower) or np.any(position > upper):
        raise ValueError("position {} is outside the configured workspace".format(position))


def encode_image(message, bridge, max_side):
    # cv_bridge handles RGB/BGR, row padding, and supported ROS encodings.
    image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
    if image.size == 0:
        raise ValueError("empty camera image")
    height, width = image.shape[:2]
    scale = min(1.0, float(max_side) / max(height, width))
    if scale < 1.0:
        image = cv2.resize(image, (max(1, round(width * scale)), max(1, round(height * scale))),
                           interpolation=cv2.INTER_AREA)
    success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not success:
        raise ValueError("JPEG encoding failed")
    return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")


TRANSLATIONS = {
    "MV_FWD": (1, 0, 0), "MV_BACK": (-1, 0, 0),
    "MV_LEFT": (0, 1, 0), "MV_RIGHT": (0, -1, 0),
    "MV_UP": (0, 0, 1), "MV_DOWN": (0, 0, -1),
}
ROTATIONS = ("ROTATE_CW", "ROTATE_CCW")
ACTIONS = list(TRANSLATIONS) + list(ROTATIONS) + ["GRASP", "RELEASE", "DONE", "UNCERTAIN"]


def object_schema(properties):
    return {"type": "object", "properties": properties,
            "required": list(properties), "additionalProperties": False}


UNIT_SCHEMA = object_schema({
    "action": {"type": "string", "enum": ACTIONS},
    "axis": {"type": "string", "enum": ["NONE", "x", "y", "z"]},
})
DECISION_SCHEMA = object_schema({
    "actions": {"type": "array", "items": UNIT_SCHEMA},
    "target_visible_in_wrist": {"type": "boolean"},
    "confidence": {"type": "number"},
    "reasoning": {"type": "string"},
})
STAGE_FIELDS = ("id", "target", "affordance", "motion", "description", "completion")
PLAN_SCHEMA = object_schema({"subgoals": {
    "type": "array", "items": object_schema({name: {"type": "string"}
                                              for name in STAGE_FIELDS})}})

def semantic_rotation(values):
    """Columns map semantic (+forward, +left, +up) to robot base axes."""
    rotation = np.asarray(values, dtype=float)
    if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
        raise ValueError("semantic_to_base must be a proper 3x3 rotation")
    return rotation


def parse_response(response):
    if response.status != "completed":
        raise ValueError("model response incomplete: {}".format(response.status))
    for item in response.output:
        for part in getattr(item, "content", []):
            if getattr(part, "type", None) == "refusal":
                raise ValueError("model refused the request")
    return json.loads(response.output_text)


def validate_plan(plan, max_subgoals):
    if not isinstance(plan, dict) or set(plan) != {"subgoals"}:
        raise ValueError("invalid plan fields")
    stages = plan["subgoals"]
    if not isinstance(stages, list) or not 1 <= len(stages) <= max_subgoals:
        raise ValueError("invalid subgoal count")
    ids = set()
    for stage in stages:
        if (not isinstance(stage, dict) or set(stage) != set(STAGE_FIELDS)
                or any(not isinstance(stage[key], str) or not stage[key].strip()
                       for key in STAGE_FIELDS) or stage["id"] in ids):
            raise ValueError("invalid or duplicate subgoal")
        ids.add(stage["id"])
    return stages


def validate_decision(decision, max_chunk, enable_rotation):
    if not isinstance(decision, dict) or set(decision) != set(DECISION_SCHEMA["properties"]):
        raise ValueError("invalid decision fields")
    if not isinstance(decision["target_visible_in_wrist"], bool):
        raise ValueError("wrist visibility must be Boolean")
    confidence = decision["confidence"]
    if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not math.isfinite(confidence) or not 0 <= confidence <= 1):
        raise ValueError("invalid confidence")
    if not isinstance(decision["reasoning"], str) or not decision["reasoning"].strip():
        raise ValueError("missing visual reasoning")
    units = decision["actions"]
    if not isinstance(units, list) or not 1 <= len(units) <= max_chunk:
        raise ValueError("invalid chunk length")
    for unit in units:
        if not isinstance(unit, dict) or set(unit) != {"action", "axis"}:
            raise ValueError("invalid action unit")
        action, axis = unit["action"], unit["axis"]
        if not isinstance(action, str) or action not in ACTIONS:
            raise ValueError("unknown semantic action")
        if action in ROTATIONS:
            if not enable_rotation or axis not in ("x", "y", "z"):
                raise ValueError("rotation disabled or invalid axis")
        elif axis != "NONE":
            raise ValueError("non-rotation action requires axis NONE")
    if len(units) > 1 and (decision["target_visible_in_wrist"]
                          or any(unit["action"] not in TRANSLATIONS
                                 or unit["action"] == "MV_DOWN" for unit in units)):
        raise ValueError("chunks require a distant target and free-space translations")
    return decision


def gripper_width(message, joint_names):
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise ValueError("invalid gripper joint state")
    positions = dict(zip(message.name, message.position))
    if any(name not in positions for name in joint_names):
        raise ValueError("gripper state missing configured finger joints")
    fingers = finite_vector([positions[name] for name in joint_names], 2, "finger positions")
    if np.any(fingers < 0):
        raise ValueError("negative gripper finger position")
    return float(np.sum(fingers))


def pose_target(position, orientation, unit, translation_step, rotation_step,
                rotation, lower, upper, table_height, tip_offset, floor_clearance):
    """Eq. 4: local pose update, base-frame pre-multiplication, bounded sweep."""
    position = finite_vector(position, 3, "position")
    orientation = finite_vector(orientation, 4, "orientation")
    if not np.isclose(np.linalg.norm(orientation), 1.0, atol=1e-6):
        raise ValueError("orientation must be a unit quaternion")
    if not all(math.isfinite(step) and step > 0 for step in (translation_step, rotation_step)):
        raise ValueError("increments must be positive and finite")
    target, quaternion = position.copy(), orientation.copy()
    action = unit["action"]
    if action in TRANSLATIONS:
        target += translation_step * (rotation @ np.asarray(TRANSLATIONS[action]))
    elif action in ROTATIONS:
        axis = rotation[:, ("x", "y", "z").index(unit["axis"])]
        angle = rotation_step * (1 if action == "ROTATE_CCW" else -1)
        quaternion = quaternion_multiply(quaternion_about_axis(angle, axis), orientation)
        quaternion /= np.linalg.norm(quaternion)
    else:
        raise ValueError("pose_target requires a movement or rotation")
    # Includes start and intermediate tip heights, so tilted tools cannot sweep
    # below the table even when the final EE origin remains in its box.
    for fraction in np.linspace(0, 1, max(2, int(math.ceil(rotation_step / math.radians(1))) + 1)):
        point = position + fraction * (target - position)
        check_workspace(point, lower, upper)
        q = quaternion_slerp(orientation, quaternion, fraction)
        tip = point + quaternion_matrix(q)[:3, :3] @ tip_offset
        if tip[2] < table_height + floor_clearance:
            raise ValueError("tool tip sweep violates the calibrated table floor")
    return target, quaternion


class ShowHarness:
    """Independent VLM policy with MoveIt arm execution and Franka gripper I/O."""

    def __init__(self):
        self._configure_parameters()
        self.planner_prompt = self._text("planner_prompt")
        self.controller_prompt = self._text("controller_prompt")
        self.semantic_to_base = semantic_rotation(rospy.get_param("~semantic_to_base"))
        self.table_height = float(rospy.get_param("~table_height"))
        if not math.isfinite(self.table_height):
            raise ValueError("table_height must be finite")
        self.tip_offset = finite_vector(rospy.get_param("~tip_offset_ee"), 3, "tip_offset_ee")
        self.floor_clearance = self._number("floor_clearance", 0.005, allow_zero=True)
        self.fine_step = self._number("fine_step_size", 0.02)
        self.coarse_step = self._number("coarse_step_size", 0.04)
        self.max_translation = self._number("max_translation_step", 0.04)
        self.rotation_step = math.radians(self._number("rotation_step_degrees", 15.0))
        self.max_rotation = math.radians(self._number("max_rotation_step_degrees", 15.0))
        if not (self.fine_step <= self.coarse_step <= self.max_translation
                and self.rotation_step <= self.max_rotation <= math.pi / 2):
            raise ValueError("invalid fine/coarse or rotation step limits")
        self.enable_rotation = self._boolean("enable_rotation", True)
        self.adaptive_step = self._boolean("adaptive_step", True)
        self.action_chunking = self._boolean("action_chunking", True)
        self.subtask_planning = self._boolean("subtask_planning", True)
        self.failure_recovery = self._boolean("failure_recovery", True)
        self.max_chunk = self._integer("max_chunk_length", 3) if self.action_chunking else 1
        self.max_actions = self._integer("max_actions", 150)
        self.max_subgoals = self._integer("max_subgoals", 20)
        self.max_recoveries = self._integer("max_recoveries", 3)
        self.reference_camera = str(rospy.get_param("~reference_camera"))
        self.wrist_camera = str(rospy.get_param("~wrist_camera"))
        self.camera_guidance = rospy.get_param("~camera_guidance")
        if (not isinstance(self.camera_guidance, dict) or not self.camera_guidance
                or self.reference_camera == self.wrist_camera
                or any(not isinstance(name, str) or not isinstance(text, str)
                       or not name.strip() or not text.strip()
                       for name, text in self.camera_guidance.items())
                or any(name not in self.camera_guidance
                       for name in (self.reference_camera, self.wrist_camera))):
            raise ValueError("provide camera_guidance for distinct reference and wrist cameras")
        self.finger_joints = rospy.get_param("~gripper_finger_joints", [
            "panda_finger_joint1", "panda_finger_joint2"])
        if (not isinstance(self.finger_joints, list) or len(self.finger_joints) != 2
                or any(not isinstance(name, str) or not name for name in self.finger_joints)
                or len(set(self.finger_joints)) != 2):
            raise ValueError("provide two distinct gripper finger joint names")
        self.open_width = self._number("gripper_open_width", 0.08)
        self.grasp_width = self._number("gripper_grasp_width", 0.0, allow_zero=True)
        self.empty_width = self._number("empty_grasp_width", 0.003)
        self.width_tolerance = self._number("gripper_width_tolerance", 0.003)
        self.gripper_speed = self._number("gripper_speed", 0.05)
        self.gripper_force = self._number("gripper_force", 20.0)
        self.epsilon_inner = self._number("gripper_epsilon_inner", 0.005, allow_zero=True)
        self.epsilon_outer = self._number("gripper_epsilon_outer", 0.08, allow_zero=True)
        self.gripper_timeout = self._number("gripper_timeout", 10.0)
        if not 0 <= self.grasp_width <= self.empty_width < self.open_width <= 0.08:
            raise ValueError("invalid gripper widths")
        self.plan = []
        self.stage_index = 0
        self.grasp_stage_index = None
        self.gripper_status = "unknown"
        self.holding = False
        self.recovery_note = ""
        self.recovery_count = 0
        self.executed_actions = 0
        self.api_calls = 0
        self.api_seconds = 0.0
        self.episode_id = uuid.uuid4().hex
        self.log_file = None
        self.grasp_client = self.move_client = self.stop_client = None
        self.active_gripper_client = None
        self.setpoint_position = self.setpoint_orientation = None
        self._initialize_interfaces()
        log_path = str(rospy.get_param("~episode_log", "")).strip()
        try:
            if log_path:
                # Exclusive creation protects earlier trial artifacts.
                self.log_file = open(os.path.expanduser(log_path), "x", buffering=1)
            if not self.dry_run:
                self._initialize_moveit()
                self._initialize_gripper()
        except Exception:
            self.client.close()
            if self.log_file is not None:
                self.log_file.close()
            if self.moveit_commander is not None:
                self.moveit_commander.roscpp_shutdown()
            raise
        rospy.on_shutdown(self._hold_after_error)

    def _configure_parameters(self):
        self.dry_run = rospy.get_param("~dry_run", True)
        if not isinstance(self.dry_run, bool):
            raise ValueError("~dry_run must be a Boolean")
        self.task = str(rospy.get_param("~task_description", "")).strip()
        if not self.task:
            raise ValueError("set ~task_description to the actual task")
        self.model = str(rospy.get_param("~model_name", "gpt-5.6-terra"))
        self.reasoning_effort = str(rospy.get_param("~reasoning_effort", ""))
        self.base_frame = str(rospy.get_param("~base_frame", "panda_link0"))
        self.planning_group = str(rospy.get_param("~planning_group", "panda_arm"))
        self.end_effector_link = str(rospy.get_param("~end_effector_link", "panda_link8"))
        self.controller_name = str(rospy.get_param(
            "~controller_name", "position_joint_trajectory_controller"))

        self.api_timeout = self._number("api_timeout", 45.0)
        self.min_confidence = self._number("min_confidence", 0.7, allow_zero=True)
        self.max_decisions = self._integer("max_decisions", 100)
        self.max_image_side = self._integer("max_image_side", 768)
        self.max_output_tokens = self._integer("max_output_tokens", 4096)
        self.history_size = self._integer("history_size", 5)
        self.observation_timeout = self._number("observation_timeout", 10.0)
        self.max_sensor_age = self._number("max_sensor_age", 0.5)
        self.max_sync_skew = self._number("max_sync_skew", 0.1)
        self.max_decision_age = self._number("max_decision_age", 60.0)
        self.max_pre_execution_drift = self._number("max_pre_execution_drift", 0.003)
        self.max_pre_execution_joint_drift = self._number(
            "max_pre_execution_joint_drift", 0.02)
        self.stationary_joint_speed = self._number("stationary_joint_speed", 0.03)
        self.position_tolerance = self._number("position_tolerance", 0.004)
        self.orientation_tolerance = self._number("orientation_tolerance", 0.04)
        self.moveit_state_tolerance = self._number("moveit_state_tolerance", 0.02)
        self.moveit_orientation_tolerance = self._number("moveit_orientation_tolerance", 0.10)
        self.settle_time = self._number("settle_time", 0.4)
        self.motion_timeout = self._number("motion_timeout", 8.0)
        self.settle_poll_rate = self._number("settle_poll_rate", 20.0)
        self.velocity_scaling = self._fraction("velocity_scaling", 0.05)
        self.acceleration_scaling = self._fraction("acceleration_scaling", 0.05)
        self.cartesian_eef_step = self._number("cartesian_eef_step", 0.002)
        self.min_cartesian_fraction = self._fraction("min_cartesian_fraction", 0.999)
        self.moveit_wait_timeout = self._number("moveit_wait_timeout", 30.0)
        self.controller_timeout = self._number("controller_timeout", 5.0)
        self.planning_time = self._number("planning_time", 5.0)
        self.planning_attempts = self._integer("planning_attempts", 5)
        self.lower = finite_vector(rospy.get_param("~workspace_min"), 3, "workspace_min")
        self.upper = finite_vector(rospy.get_param("~workspace_max"), 3, "workspace_max")
        if np.any(self.lower >= self.upper) or self.min_confidence > 1:
            raise ValueError("invalid workspace limits or confidence threshold")
        if not all((self.base_frame, self.planning_group, self.end_effector_link,
                    self.controller_name)):
            raise ValueError("MoveIt frame, group, link, and controller names cannot be empty")

    @staticmethod
    def _text(name):
        value = rospy.get_param("~" + name, "")
        if not isinstance(value, str) or not value.strip():
            raise ValueError("~{} must be a non-empty string in the policy config".format(name))
        return value.strip()

    @staticmethod
    def _boolean(name, default):
        value = rospy.get_param("~" + name, default)
        if not isinstance(value, bool):
            raise ValueError("~{} must be Boolean".format(name))
        return value

    @staticmethod
    def _number(name, default, allow_zero=False):
        raw = rospy.get_param("~" + name, default)
        value = float(raw)
        if isinstance(raw, bool) or not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
            raise ValueError("~{} must be {}finite".format(name, "nonnegative and " if allow_zero else "positive and "))
        return value

    @staticmethod
    def _integer(name, default):
        value = rospy.get_param("~" + name, default)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError("~{} must be a positive integer".format(name))
        return value

    @staticmethod
    def _fraction(name, default):
        value = ShowHarness._number(name, default)
        if value > 1:
            raise ValueError("~{} must be at most 1".format(name))
        return value

    def _initialize_interfaces(self):
        # Import lazily so offline tests need neither an API key nor the SDK.
        from openai import OpenAI
        self.client = OpenAI(timeout=self.api_timeout, max_retries=0)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = None
        self.history = deque(maxlen=self.history_size)
        self.robot = None
        self.group = None
        self.moveit_commander = None
        self.decision_publisher = rospy.Publisher("~decision", String, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~sensor_topic", "/multi_camera_synchronized_observation"),
            MultiCameraSynchronizedObservation, self._sensor_callback, queue_size=1, buff_size=16777216)
        self.controller_service = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers)

    def _initialize_moveit(self):
        import moveit_commander

        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_commander = moveit_commander
        try:
            self.robot = moveit_commander.RobotCommander()
            self.group = moveit_commander.MoveGroupCommander(
                self.planning_group, wait_for_servers=self.moveit_wait_timeout)
            if self.group.get_planning_frame().lstrip("/") != self.base_frame.lstrip("/"):
                raise RuntimeError("MoveIt planning frame must be {}".format(self.base_frame))
            # Link enumeration is provided by RobotCommander in the ROS
            # Noetic Python API (MoveGroupCommander has no get_link_names()).
            group_links = self.robot.get_link_names(group=self.planning_group)
            if self.end_effector_link not in group_links:
                raise RuntimeError("{} is not in MoveIt group {}".format(
                    self.end_effector_link, self.planning_group))
            active_joints = set(self.group.get_active_joints())
            if active_joints != set(PANDA_JOINTS):
                raise RuntimeError("MoveIt group {} has unexpected active joints".format(
                    self.planning_group))
            self.group.set_end_effector_link(self.end_effector_link)
            self.group.set_planning_time(self.planning_time)
            self.group.set_num_planning_attempts(self.planning_attempts)
            self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
            self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
            self.group.set_goal_position_tolerance(self.position_tolerance)
            self.group.set_goal_orientation_tolerance(self.orientation_tolerance)
        except Exception:
            moveit_commander.roscpp_shutdown()
            self.moveit_commander = None
            raise

    def _initialize_gripper(self):
        import actionlib
        from franka_gripper.msg import GraspAction, MoveAction, StopAction
        for label, action_type in (("grasp", GraspAction), ("move", MoveAction), ("stop", StopAction)):
            name = rospy.get_param("~gripper_{}_action".format(label), "/franka_gripper/" + label)
            client = actionlib.SimpleActionClient(name, action_type)
            if not client.wait_for_server(rospy.Duration(self.controller_timeout)):
                raise RuntimeError("gripper server unavailable: " + name)
            setattr(self, label + "_client", client)

    def _sensor_callback(self, observation):
        with self.lock:
            self.latest = observation
            self.received_at = time.monotonic()

    def _observation_images(self, observation):
        names = list(observation.camera_names)
        if (not names or len(names) != len(observation.images) or len(set(names)) != len(names)
                or set(names) != set(self.camera_guidance)):
            raise ValueError("observation camera names must exactly match camera_guidance")
        return observation.images

    def _snapshot(self):
        with self.lock:
            observation = self.latest
            received_at = self.received_at
        if observation is None or received_at is None:
            raise RuntimeError("no synchronized observation received")
        if time.monotonic() - received_at > self.max_sensor_age:
            raise RuntimeError("synchronized observation stream is not fresh")

        stamps = [image.header.stamp for image in self._observation_images(observation)] + [observation.state.header.stamp,
                  observation.gripper_state.header.stamp]
        seconds = [stamp.to_sec() for stamp in stamps]
        if any(value <= 0 for value in seconds):
            raise RuntimeError("observation contains a zero sensor timestamp")
        now = rospy.Time.now().to_sec()
        if any(abs(now - value) > self.max_sensor_age for value in seconds):
            raise RuntimeError("observation sensor timestamp is stale or in the future")
        if max(seconds) - min(seconds) > self.max_sync_skew:
            raise RuntimeError("observation sensor timestamps are not synchronized")
        # franka_state_controller commonly leaves this Header frame empty. The
        # libfranka O_T_EE field is nevertheless defined in robot frame O,
        # which is panda_link0 for this configuration. Preserve rejection of
        # an explicitly conflicting frame instead of treating it as O.
        state_frame = observation.state.header.frame_id.strip().lstrip("/")
        expected_frame = self.base_frame.strip().lstrip("/")
        if state_frame and state_frame != expected_frame:
            raise RuntimeError(
                "Franka state frame {!r} does not match configured base frame {!r}"
                .format(observation.state.header.frame_id, self.base_frame))
        if not state_frame:
            rospy.logwarn_throttle(
                30.0,
                "FrankaState.header.frame_id is empty; interpreting O_T_EE in %s "
                "according to the libfranka frame convention",
                self.base_frame)

        state = observation.state
        position, orientation = pose_from_state(state)
        check_workspace(position, self.lower, self.upper)
        finite_vector(state.q, 7, "joint positions")
        finite_vector(state.dq, 7, "joint velocities")
        if not self.dry_run and state.robot_mode != FrankaState.ROBOT_MODE_MOVE:
            raise RuntimeError("robot must be in Franka MOVE mode")
        width = gripper_width(observation.gripper_state, self.finger_joints)
        if width > self.open_width + self.width_tolerance:
            raise ValueError("measured gripper width exceeds configured maximum")
        for name in ("cartesian_contact", "cartesian_collision", "O_F_ext_hat_K"):
            finite_vector(getattr(observation.state, name), 6, name)
        if not self.dry_run and any(observation.state.cartesian_collision):
            raise RuntimeError("Franka reports Cartesian collision")
        tip = position + quaternion_matrix(orientation)[:3, :3] @ self.tip_offset
        if tip[2] < self.table_height + self.floor_clearance:
            raise RuntimeError("measured tool tip is below the calibrated floor")
        return observation, position, orientation

    def _wait_for_start(self):
        deadline = time.monotonic() + self.observation_timeout
        last_error = "no observation"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                observation, position, orientation = self._snapshot()
                if not self.dry_run:
                    rigid_transform(observation.state.F_T_EE, "F_T_EE")
                self.setpoint_position = position.copy()
                self.setpoint_orientation = orientation.copy()
                return observation, position, orientation
            except (RuntimeError, ValueError) as error:
                last_error = str(error)
                rospy.logwarn_throttle(1.0, "Waiting for valid observation: %s", last_error)
                time.sleep(0.05)
        raise RuntimeError("no valid synchronized observation: {}".format(last_error))

    def _check_controller(self):
        """Require MoveIt to be connected to the real Panda trajectory controller."""
        manager = rospy.get_param("/move_group/moveit_controller_manager", "")
        allowed = (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_ros_control_interface::MoveItControllerManager",
        )
        if not manager or "FakeControllerManager" in manager:
            raise RuntimeError("MoveIt is configured for fake or missing trajectory execution")
        if manager not in allowed:
            raise RuntimeError("unsupported MoveIt controller manager: {}".format(manager))

        if manager == allowed[0]:
            configured = rospy.get_param("/move_group/controller_list", [])
            matching = [
                entry for entry in configured
                if entry.get("name", "").strip("/") == self.controller_name
                and entry.get("action_ns") == "follow_joint_trajectory"
                and entry.get("type") == "FollowJointTrajectory"
                and set(entry.get("joints", [])) == set(PANDA_JOINTS)
            ]
            if not matching:
                raise RuntimeError("MoveIt controller_list does not map the Panda arm to {}"
                                   .format(self.controller_name))

        try:
            rospy.wait_for_service(
                "/controller_manager/list_controllers", timeout=self.controller_timeout)
            controllers = self.controller_service().controller
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise RuntimeError("controller manager is unavailable: {}".format(error)) from error

        conflicts = []
        for controller in controllers:
            if controller.state != "running" or controller.name == self.controller_name:
                continue
            claimed = set()
            for resource in getattr(controller, "claimed_resources", []):
                claimed.update(resource.resources)
            if claimed.intersection(PANDA_JOINTS):
                conflicts.append(controller.name)
        if conflicts:
            raise RuntimeError("conflicting Panda arm controller(s) are running: {}".format(
                ", ".join(conflicts)))

        selected = next((controller for controller in controllers
                         if controller.name == self.controller_name), None)
        if selected is None or selected.state != "running":
            raise RuntimeError("{} is not running".format(self.controller_name))
        if (getattr(selected, "type", "")
                and selected.type != "position_controllers/JointTrajectoryController"):
            raise RuntimeError("{} has unexpected type {}".format(
                self.controller_name, selected.type))

    def _verify_moveit_state(self, state):
        measured_flange = (rigid_transform(state.O_T_EE, "O_T_EE")
                           @ np.linalg.inv(rigid_transform(state.F_T_EE, "F_T_EE")))
        moveit_pose = self.group.get_current_pose(self.end_effector_link)
        if moveit_pose.header.frame_id.lstrip("/") != self.base_frame.lstrip("/"):
            raise RuntimeError("MoveIt current pose is not expressed in {}".format(
                self.base_frame))
        position_error, angle_error = pose_errors(
            matrix_from_pose(moveit_pose.pose), measured_flange)
        if (position_error > self.moveit_state_tolerance
                or angle_error > self.moveit_orientation_tolerance):
            raise RuntimeError(
                "MoveIt state disagrees with Franka state ({:.4f} m, {:.4f} rad)"
                .format(position_error, angle_error))

    def _plan(self, observation, target, target_orientation):
        """Plan a straight, collision-checked flange path for an O_T_EE target."""
        self._verify_moveit_state(observation.state)
        target_ee = quaternion_matrix(target_orientation)
        target_ee[:3, 3] = target
        tool_transform = rigid_transform(observation.state.F_T_EE, "F_T_EE")
        target_flange = target_ee @ np.linalg.inv(tool_transform)

        goal = PoseStamped()
        goal.header.frame_id = self.base_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target_flange[:3, 3]
        quaternion = quaternion_from_matrix(target_flange)
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = quaternion

        self.group.set_start_state_to_current_state()
        trajectory, fraction = self.group.compute_cartesian_path(
            [goal.pose], self.cartesian_eef_step, avoid_collisions=True)
        if (not math.isfinite(fraction) or fraction < self.min_cartesian_fraction
                or not trajectory.joint_trajectory.points):
            raise RuntimeError("MoveIt found only {:.1%} of the requested Cartesian path"
                               .format(fraction))
        if set(trajectory.joint_trajectory.joint_names) != set(PANDA_JOINTS):
            raise RuntimeError("MoveIt planned a trajectory for unexpected joints")

        trajectory = self.group.retime_trajectory(
            self.robot.get_current_state(), trajectory,
            velocity_scaling_factor=self.velocity_scaling,
            acceleration_scaling_factor=self.acceleration_scaling)
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt failed to time-parameterize the Cartesian path")
        duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        if not math.isfinite(duration) or duration <= 0:
            raise RuntimeError("MoveIt returned an invalid trajectory duration")
        return trajectory, duration

    def _execute(self, observation, start, target, target_orientation):
        self._check_controller()
        current_observation, current, current_orientation = self._snapshot()
        if np.linalg.norm(current - start) > self.max_pre_execution_drift:
            raise RuntimeError("robot moved before MoveIt planning")
        if orientation_distance(
                current_orientation, pose_from_state(observation.state)[1]) > self.orientation_tolerance:
            raise RuntimeError("robot rotated before MoveIt planning")
        if np.max(np.abs(current_observation.state.dq)) > self.stationary_joint_speed:
            raise RuntimeError("robot is moving before MoveIt planning")

        trajectory, duration = self._plan(current_observation, target, target_orientation)
        rospy.loginfo("Executing %.2f s MoveIt Cartesian trajectory to %s", duration, target)

        self._check_controller()
        latest_observation, latest, latest_orientation = self._snapshot()
        if np.linalg.norm(latest - current) > self.max_pre_execution_drift:
            raise RuntimeError("robot moved after MoveIt planning; refusing stale trajectory")
        if orientation_distance(
                latest_orientation, current_orientation) > self.orientation_tolerance:
            raise RuntimeError("robot rotated after MoveIt planning; refusing stale trajectory")
        if (np.max(np.abs(np.asarray(latest_observation.state.q)
                          - np.asarray(current_observation.state.q)))
                > self.max_pre_execution_joint_drift):
            raise RuntimeError("robot joints moved after MoveIt planning; refusing stale trajectory")
        if np.max(np.abs(latest_observation.state.dq)) > self.stationary_joint_speed:
            raise RuntimeError("robot is moving after MoveIt planning")
        if not np.allclose(latest_observation.state.F_T_EE,
                           current_observation.state.F_T_EE, atol=1e-6):
            raise RuntimeError("end-effector transform changed after MoveIt planning")

        try:
            if not self.group.execute(trajectory, wait=True):
                raise RuntimeError("MoveIt failed to execute the Cartesian trajectory")
        finally:
            self.group.stop()
        return self._wait_until_settled(target, target_orientation)

    def _wait_until_settled(self, target, target_orientation):
        deadline = time.monotonic() + self.motion_timeout
        settled_since = None
        position_error = joint_speed = math.inf
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            observation, position, orientation = self._snapshot()
            position_error = float(np.linalg.norm(position - target))
            joint_speed = float(np.max(np.abs(finite_vector(
                observation.state.dq, 7, "joint velocities"))))
            angle_ok = (orientation_distance(orientation, target_orientation) <= self.orientation_tolerance)
            if (position_error <= self.position_tolerance and angle_ok
                    and joint_speed <= self.stationary_joint_speed):
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= self.settle_time:
                    return observation, position
            else:
                settled_since = None
            time.sleep(1.0 / self.settle_poll_rate)
        raise RuntimeError(
            "robot did not settle at target (position error {:.4f} m, max joint speed "
            "{:.4f} rad/s)".format(position_error, joint_speed))

    def _context(self, observation, position, orientation):
        width = gripper_width(observation.gripper_state, self.finger_joints)
        tip = position + quaternion_matrix(orientation)[:3, :3] @ self.tip_offset
        return {
            "task": self.task, "base_frame": self.base_frame,
            "active_subgoal": self.plan[self.stage_index] if self.plan else None,
            "remaining_subgoals": self.plan[self.stage_index:],
            "reference_camera": self.reference_camera, "wrist_camera": self.wrist_camera,
            "camera_guidance": self.camera_guidance,
            "semantic_to_base_rotation": self.semantic_to_base.tolist(),
            "ee_position_m": position.tolist(), "ee_orientation_xyzw": orientation.tolist(),
            "joint_positions_rad": list(observation.state.q),
            "joint_velocities_rad_s": list(observation.state.dq),
            "gripper_width_m": width, "gripper_status": self.gripper_status,
            "holding_candidate": self.holding,
            "grasp_caveat": "Width and action success suggest a hold; verify it visually.",
            "tool_tip_height_above_table_m": float(tip[2] - self.table_height),
            "cartesian_contact": list(observation.state.cartesian_contact),
            "external_wrench_base_N_Nm": list(observation.state.O_F_ext_hat_K),
            "fine_step_m": self.fine_step, "coarse_step_m": self.coarse_step,
            "step_rule": "fine when wrist target visible; coarse otherwise" if self.adaptive_step else "always fine",
            "fine_action_deltas_base_m": {name: (self.fine_step * self.semantic_to_base @ np.asarray(vector)).tolist()
                                         for name, vector in TRANSLATIONS.items()},
            "rotation_step_degrees": math.degrees(self.rotation_step),
            "rotation_enabled": self.enable_rotation, "max_chunk_length": self.max_chunk,
            "setpoint_position_m": self.setpoint_position.tolist(),
            "setpoint_orientation_xyzw": self.setpoint_orientation.tolist(),
            "workspace_min_m": self.lower.tolist(), "workspace_max_m": self.upper.tolist(),
            "floor_clearance_m": self.floor_clearance,
            "recent_actions_newest_first": list(reversed(self.history)),
            "recovery": self.recovery_note,
        }

    def _query(self, instructions, schema, context, observation, role):
        content = [{"type": "input_text", "text": json.dumps(context, allow_nan=False)}]
        for name, image in zip(observation.camera_names, observation.images):
            content.extend([
                {"type": "input_text", "text": "Camera {} ({}), stamp {:.6f}".format(
                    name, image.header.frame_id, image.header.stamp.to_sec())},
                {"type": "input_image", "image_url": encode_image(image, self.bridge, self.max_image_side),
                 "detail": "high"},
            ])
        kwargs = {"model": self.model, "store": False, "instructions": instructions,
                  "input": [{"role": "user", "content": content}],
                  "max_output_tokens": self.max_output_tokens,
                  "text": {"format": {"type": "json_schema", "name": role,
                                      "strict": True, "schema": schema}}}
        if self.reasoning_effort:
            kwargs["reasoning"] = {"effort": self.reasoning_effort}
        start = time.monotonic()
        response = self.client.responses.create(**kwargs)
        elapsed = time.monotonic() - start
        self.api_calls += 1
        self.api_seconds += elapsed
        if elapsed > self.max_decision_age:
            raise RuntimeError("model response too old to use")
        result = parse_response(response)
        self._record({"outcome": "model_response", "role": role, "api_seconds": elapsed,
                      "usage": response.usage.model_dump() if response.usage else None,
                      "response": result, "observation": context,
                      "sensor_stamps": [image.header.stamp.to_sec() for image in observation.images]})
        return result

    def _record(self, event):
        event.update({"episode_id": self.episode_id, "backend": "moveit",
                      "stage_index": self.stage_index, "executed_actions": self.executed_actions})
        event.update({"stamp": rospy.Time.now().to_sec(), "dry_run": self.dry_run})
        payload = json.dumps(event, allow_nan=False)
        self.decision_publisher.publish(String(data=payload))
        rospy.loginfo("Show-Harness: %s", payload)
        if self.log_file is not None:
            self.log_file.write(json.dumps(event, allow_nan=False) + "\n")

    def _fresh_stationary(self, observed, check_gripper=True):
        latest, position, orientation = self._snapshot()
        _, original_position, original_orientation = observed
        if (np.linalg.norm(position - original_position) > self.max_pre_execution_drift
                or orientation_distance(orientation, original_orientation) > self.orientation_tolerance
                or np.max(np.abs(np.asarray(latest.state.q) - np.asarray(observed[0].state.q)))
                > self.max_pre_execution_joint_drift
                or np.max(np.abs(latest.state.dq)) > self.stationary_joint_speed
                or (check_gripper and abs(gripper_width(latest.gripper_state, self.finger_joints)
                       - gripper_width(observed[0].gripper_state, self.finger_joints)) > self.width_tolerance)):
            raise RuntimeError("robot or gripper changed during model inference/planning")
        if not np.allclose(latest.state.F_T_EE, observed[0].state.F_T_EE, atol=1e-6):
            raise RuntimeError("tool transform changed during inference/planning")
        return latest, position, orientation

    def _target(self, position, orientation, unit, step):
        return pose_target(position, orientation, unit, step, self.rotation_step,
                           self.semantic_to_base, self.lower, self.upper,
                           self.table_height, self.tip_offset, self.floor_clearance)

    def _check_setpoint_tracking(self, observed):
        if (np.linalg.norm(observed[1] - self.setpoint_position) > self.position_tolerance
                or orientation_distance(observed[2], self.setpoint_orientation) > self.orientation_tolerance):
            raise RuntimeError("measured pose drifted from the settled interpreter setpoint")

    def _execute_pose(self, observed, target, quaternion):
        self._check_controller()
        latest, position, _ = self._fresh_stationary(observed)
        return self._execute(latest, position, target, quaternion)

    def _gripper_command(self, action):
        from actionlib_msgs.msg import GoalStatus
        from franka_gripper.msg import GraspGoal, MoveGoal
        if action == "GRASP":
            goal, client = GraspGoal(), self.grasp_client
            goal.width, goal.speed, goal.force = self.grasp_width, self.gripper_speed, self.gripper_force
            goal.epsilon.inner, goal.epsilon.outer = self.epsilon_inner, self.epsilon_outer
        else:
            goal, client = MoveGoal(), self.move_client
            goal.width, goal.speed = self.open_width, self.gripper_speed
        arm_start = self._snapshot()
        self._fresh_stationary(arm_start)
        self.active_gripper_client = client
        command_stamp = rospy.Time.now().to_sec()
        client.send_goal(goal)
        deadline = time.monotonic() + self.gripper_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._fresh_stationary(arm_start, check_gripper=False)
            if client.wait_for_result(rospy.Duration(0.05)):
                break
        else:
            client.cancel_goal()
            raise RuntimeError("gripper command timed out or shutdown requested")
        result = client.get_result()
        status = client.get_state()
        self.active_gripper_client = None
        if result is None or status in (GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.LOST):
            raise RuntimeError("gripper execution interrupted or unavailable")
        succeeded = status == GoalStatus.SUCCEEDED and bool(result.success)
        # Require new, stable width measurements AFTER the action finishes.
        command_stamp = max(command_stamp, rospy.Time.now().to_sec())
        stable_since, previous_width, previous_stamp = None, None, None
        deadline = time.monotonic() + self.gripper_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            observation, position, _ = self._fresh_stationary(arm_start, check_gripper=False)
            stamp = observation.gripper_state.header.stamp.to_sec()
            width = gripper_width(observation.gripper_state, self.finger_joints)
            if stamp > command_stamp and stamp != previous_stamp:
                if previous_width is None or abs(width - previous_width) > self.width_tolerance:
                    stable_since = time.monotonic()
                previous_width, previous_stamp = width, stamp
                if stable_since is not None and time.monotonic() - stable_since >= self.settle_time:
                    if action == "RELEASE" and (not succeeded or abs(width - self.open_width) > self.width_tolerance):
                        raise RuntimeError("gripper release failed: " + getattr(result, "error", ""))
                    return observation, position, width, succeeded, getattr(result, "error", "")
            time.sleep(1.0 / self.settle_poll_rate)
        raise RuntimeError("gripper width did not settle on fresh measurements")

    def _recover_grasp(self, reason):
        self.holding = False
        self.gripper_status = reason
        self.recovery_note = "{}: recenter the object body and confirm depth before closing again".format(reason)
        if not self.failure_recovery:
            raise RuntimeError("grasp failed with recovery disabled: " + reason)
        if self.recovery_count >= self.max_recoveries:
            raise RuntimeError("grasp recovery budget exhausted")
        if self.executed_actions >= self.max_actions:
            raise RuntimeError("no action budget remains for recovery reopen")
        self.recovery_count += 1
        self._check_controller()
        _, _, width, _, _ = self._gripper_command("RELEASE")
        self.gripper_status = "open_after_" + reason
        if self.grasp_stage_index is not None:
            self.stage_index = self.grasp_stage_index
        self.history.append({"action": "RELEASE", "outcome": "recovery_reopen", "reason": reason,
                             "gripper_width_m": width})
        self.executed_actions += 1
        self._record({"outcome": "grasp_recovery", "reason": reason,
                      "recovery_count": self.recovery_count})

    def _hold_after_error(self):
        if self.group is not None:
            try:
                self.group.stop()
            except Exception as error:  # Preserve the original error while reporting stop failure.
                rospy.logerr("Failed to stop MoveIt motion after error: %s", error)
        if self.active_gripper_client is not None:
            self.active_gripper_client.cancel_goal()
            if self.stop_client is not None:
                from franka_gripper.msg import StopGoal
                self.stop_client.send_goal(StopGoal())
            self.active_gripper_client = None

    def run(self):
        termination = "aborted"
        started = time.monotonic()
        try:
            observed = self._wait_for_start()
            if np.max(np.abs(observed[0].state.dq)) > self.stationary_joint_speed:
                raise RuntimeError("robot must be stationary at episode start")
            if not self.dry_run:
                self._check_controller()
            self._record({"outcome": "episode_start", "task": self.task, "model": self.model,
                          "reasoning_effort": self.reasoning_effort,
                          "prompts": {"planner": self.planner_prompt, "controller": self.controller_prompt},
                          "fine_step_m": self.fine_step, "coarse_step_m": self.coarse_step,
                          "semantic_to_base": self.semantic_to_base.tolist(),
                          "plugins": {"subtask_planning": self.subtask_planning,
                                      "adaptive_step": self.adaptive_step,
                                      "action_chunking": self.action_chunking,
                                      "failure_recovery": self.failure_recovery}})
            if self.subtask_planning:
                self.plan = validate_plan(self._query(
                    self.planner_prompt, PLAN_SCHEMA, self._context(*observed), observed[0], "subgoal_plan"),
                    self.max_subgoals)
            else:
                self.plan = [dict(zip(STAGE_FIELDS, ["task", self.task, "task-relevant region", "TASK",
                                                   self.task, "The entire requested task is visibly complete"]))]
            self._record({"outcome": "plan", "subgoals": self.plan})
            for index in range(self.max_decisions):
                if rospy.is_shutdown():
                    termination = "shutdown"
                    return
                if self.executed_actions >= self.max_actions:
                    termination = "action_budget"
                    return
                observed = self._snapshot()
                self._check_setpoint_tracking(observed)
                width = gripper_width(observed[0].gripper_state, self.finger_joints)
                if not self.dry_run and self.holding and width <= self.empty_width:
                    self._recover_grasp("lost_grasp")
                    continue
                if self.plan[self.stage_index]["motion"].upper() == "GRASP":
                    self.grasp_stage_index = self.stage_index
                decision = validate_decision(self._query(
                    self.controller_prompt, DECISION_SCHEMA, self._context(*observed), observed[0], "semantic_action"),
                    self.max_chunk, self.enable_rotation)
                units = decision["actions"]
                event = {"index": index, "decision": decision,
                         "observation_stamp": observed[0].state.header.stamp.to_sec()}
                if units[0]["action"] == "UNCERTAIN" or decision["confidence"] < self.min_confidence:
                    termination = "uncertain"
                    self._record(dict(event, outcome="stopped_without_action"))
                    return
                if units[0]["action"] == "DONE":
                    self._record(dict(event, outcome="subgoal_done_by_model"))
                    if self.dry_run:
                        termination = "dry_run_preview"
                        return
                    self._fresh_stationary(observed)
                    if self.stage_index == len(self.plan) - 1:
                        termination = "model_done"
                        return
                    self.stage_index += 1
                    continue
                step = self.coarse_step if self.adaptive_step and not decision["target_visible_in_wrist"] else self.fine_step
                # Validate the ENTIRE proposed chunk before its first command.
                position, quaternion = self.setpoint_position.copy(), self.setpoint_orientation.copy()
                previews = []
                for unit in units:
                    if unit["action"] in TRANSLATIONS or unit["action"] in ROTATIONS:
                        position, quaternion = self._target(position, quaternion, unit, step)
                    previews.append({"unit": unit, "target_position_m": position.tolist(),
                                     "target_orientation_xyzw": quaternion.tolist()})
                self._record(dict(event, outcome="proposed", step_m=step, targets=previews))
                if self.dry_run:
                    termination = "dry_run_preview"
                    self._record(dict(event, outcome="dry_run_preview", targets=previews))
                    return
                self._fresh_stationary(observed)
                for unit in units:
                    if rospy.is_shutdown():
                        termination = "shutdown"
                        return
                    if self.executed_actions >= self.max_actions:
                        termination = "action_budget"
                        return
                    latest = self._snapshot()
                    self._check_setpoint_tracking(latest)
                    self._check_controller()
                    if self.holding and gripper_width(latest[0].gripper_state, self.finger_joints) <= self.empty_width:
                        self._recover_grasp("lost_grasp")
                        break
                    action = unit["action"]
                    if action == "MV_DOWN" and any(latest[0].state.cartesian_contact):
                        raise RuntimeError("descent blocked by measured contact")
                    if len(units) > 1 and any(latest[0].state.cartesian_contact):
                        self._record({"outcome": "chunk_interrupted_by_contact"})
                        break
                    if action in TRANSLATIONS or action in ROTATIONS:
                        target, target_q = self._target(self.setpoint_position, self.setpoint_orientation, unit, step)
                        if (np.linalg.norm(target - latest[1]) > self.max_translation + self.position_tolerance
                                or orientation_distance(target_q, latest[2]) > self.max_rotation + self.orientation_tolerance):
                            raise RuntimeError("measured pose is too far from bounded setpoint step")
                        # Check the measured-start sweep too, including tracking error.
                        self._target(latest[1], latest[2], unit, step)
                        after, actual = self._execute_pose(latest, target, target_q)
                        self.setpoint_position, self.setpoint_orientation = target.copy(), target_q.copy()
                        outcome = {"action": action, "axis": unit["axis"], "step_m": step,
                                   "start_position_m": latest[1].tolist(),
                                   "target_position_m": target.tolist(), "actual_position_m": actual.tolist(),
                                   "target_orientation_xyzw": target_q.tolist(),
                                   "actual_orientation_xyzw": pose_from_state(after.state)[1].tolist(),
                                   "outcome": "executed_and_settled"}
                    else:
                        if action == "GRASP":
                            self.grasp_stage_index = self.stage_index
                        after, actual, width, success, error = self._gripper_command(action)
                        self.holding = action == "GRASP" and success and width > self.empty_width
                        self.gripper_status = "holding_candidate" if self.holding else "empty" if action == "GRASP" else "open"
                        outcome = {"action": action, "outcome": self.gripper_status,
                                   "gripper_width_m": width, "action_success": success, "error": error}
                    self.executed_actions += 1
                    self.history.append(outcome)
                    self._record(dict(outcome))
                    if action == "GRASP" and not self.holding:
                        self._recover_grasp("empty_grasp" if width <= self.empty_width else "unverified_grasp")
                        break
                    if action == "GRASP" and self.holding:
                        self.recovery_note = ""
            termination = "decision_budget"
        except Exception as error:
            self._hold_after_error()
            self._record({"outcome": "aborted", "error": str(error)})
            raise
        finally:
            if rospy.is_shutdown():
                self._hold_after_error()
            self._record({"outcome": "episode_end", "termination": termination,
                          "model_claimed_success": termination == "model_done",
                          "independent_success": None, "api_calls": self.api_calls,
                          "api_seconds": self.api_seconds,
                          "episode_seconds": time.monotonic() - started})
            self.client.close()
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None
            if self.moveit_commander is not None:
                self.moveit_commander.roscpp_shutdown()


def main():
    rospy.init_node("show_harness")
    try:
        ShowHarness().run()
    except rospy.ROSInterruptException:
        return 0
    except Exception as error:
        rospy.logfatal("Show-Harness stopped: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
