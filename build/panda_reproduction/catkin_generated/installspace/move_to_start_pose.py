#!/usr/bin/env python3

"""Move to the first recorded end-effector pose without using recorded joints."""

import argparse
import math
from pathlib import Path
import sys
import time

from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
import numpy as np
import rospy
from tf.transformations import quaternion_from_matrix, quaternion_matrix


JOINT_NAMES = ["panda_joint{}".format(index) for index in range(1, 8)]
CONTROLLER_NAME = "position_joint_trajectory_controller"
STATE_TOPIC = "/franka_state_controller/franka_states"
BASE_FRAME = "panda_link0"
FLANGE_LINK = "panda_link8"
STILLNESS_INTERVAL = 0.2
STILLNESS_POSITION_TOLERANCE = 0.003
STILLNESS_ORIENTATION_TOLERANCE = 0.02


def read_first_point(path):
    """Read only the pose from a 7-value pose row or 15-value trajectory row."""
    path = Path(path).expanduser()
    try:
        with path.open("r") as stream:
            for line_number, line in enumerate(stream, start=1):
                content = line.split("#", 1)[0].strip()
                if not content:
                    continue
                fields = content.replace(",", " ").split()
                if len(fields) not in (7, 15):
                    raise ValueError("{}:{} must contain 7 pose values or 15 trajectory values"
                                     .format(path, line_number))
                try:
                    pose = [float(value) for value in fields[:7]]
                except ValueError as error:
                    raise ValueError("{}:{} contains a non-numeric pose value".format(
                        path, line_number)) from error
                if not all(math.isfinite(value) for value in pose):
                    raise ValueError("{}:{} contains a non-finite pose value".format(
                        path, line_number))
                norm = math.hypot(*pose[3:7])
                if norm < 1e-6:
                    raise ValueError("{}:{} has a zero quaternion".format(path, line_number))
                return pose[:3], [value / norm for value in pose[3:7]]
    except OSError as error:
        raise ValueError("cannot read '{}': {}".format(path, error)) from error
    raise ValueError("'{}' contains no trajectory points".format(path))


def transform_matrix(values):
    """Convert a Franka column-major transform and validate its rigid geometry."""
    transform = np.asarray(values, dtype=float).reshape((4, 4), order="F")
    rotation = transform[:3, :3]
    if (not np.all(np.isfinite(transform))
            or not np.allclose(transform[3, :], [0, 0, 0, 1], atol=1e-3)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)):
        raise RuntimeError("Franka state contains an invalid pose transform")
    return transform


def pose_matrix(position, orientation):
    transform = quaternion_matrix(orientation)
    transform[:3, 3] = position
    return transform


def pose_from_matrix(transform):
    return transform[:3, 3].tolist(), quaternion_from_matrix(transform).tolist()


def transform_errors(actual, desired):
    distance = np.linalg.norm(actual[:3, 3] - desired[:3, 3])
    cosine = (np.trace(desired[:3, :3].T @ actual[:3, :3]) - 1.0) / 2.0
    angle = math.acos(max(-1.0, min(1.0, cosine)))
    return float(distance), angle


def pose_errors(state, position, orientation):
    """Compare the measured O_T_EE with a pose in panda_link0."""
    return transform_errors(transform_matrix(state.O_T_EE),
                            pose_matrix(position, orientation))


def get_robot_state(timeout=5.0):
    try:
        state = rospy.wait_for_message(STATE_TOPIC, FrankaState, timeout=timeout)
    except rospy.ROSException as error:
        raise RuntimeError("no Franka state received on {}".format(STATE_TOPIC)) from error
    if len(state.O_T_EE) != 16 or len(state.F_T_EE) != 16:
        raise RuntimeError("Franka state has an invalid end-effector transform")
    transform_matrix(state.O_T_EE)
    transform_matrix(state.F_T_EE)
    if state.header.stamp != rospy.Time():
        age = (rospy.Time.now() - state.header.stamp).to_sec()
        if abs(age) > 1.0:
            raise RuntimeError("Franka state timestamp is not current ({:.2f} s)".format(age))
    return state


def require_still_pose(state):
    """Measure end-effector motion over a short interval before planning."""
    start = transform_matrix(state.O_T_EE)
    rospy.sleep(STILLNESS_INTERVAL)
    latest = get_robot_state()
    distance, angle = transform_errors(transform_matrix(latest.O_T_EE), start)
    if (distance > STILLNESS_POSITION_TOLERANCE
            or angle > STILLNESS_ORIENTATION_TOLERANCE):
        raise RuntimeError("end effector is still moving ({:.4f} m, {:.4f} rad)"
                           .format(distance, angle))
    return latest


def require_real_controller():
    """Reject fake MoveIt execution and require the configured arm controller."""
    manager = rospy.get_param("/move_group/moveit_controller_manager", "")
    if not manager or "FakeControllerManager" in manager:
        raise RuntimeError(
            "MoveIt must run with a real controller manager; demo.launch defaults "
            "to fake execution. Use panda_moveit_config/franka_control.launch "
            "for the real robot")
    if manager not in (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_ros_control_interface::MoveItControllerManager"):
        raise RuntimeError("unsupported MoveIt controller manager: {}".format(manager))
    if manager == "moveit_simple_controller_manager/MoveItSimpleControllerManager":
        configured = rospy.get_param("/move_group/controller_list", [])
        matching = [
            entry for entry in configured
            if entry.get("name", "").strip("/") == CONTROLLER_NAME
            and entry.get("action_ns") == "follow_joint_trajectory"
            and entry.get("type") == "FollowJointTrajectory"
            and set(entry.get("joints", [])) == set(JOINT_NAMES)
        ]
        if not matching:
            raise RuntimeError(
                "MoveIt's /move_group/controller_list does not map the Panda arm "
                "to " + CONTROLLER_NAME)
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=5.0)
        controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers)().controller
    except (rospy.ROSException, rospy.ServiceException) as error:
        raise RuntimeError("controller manager is unavailable: {}".format(error)) from error
    conflicts = [
        controller.name for controller in controllers
        if controller.state == "running" and controller.name != CONTROLLER_NAME
        and any(set(resource.resources).intersection(JOINT_NAMES)
                for resource in controller.claimed_resources)
    ]
    if conflicts:
        raise RuntimeError("stop the active arm controller(s) before resetting: {}"
                           .format(", ".join(conflicts)))
    selected = next((controller for controller in controllers
                     if controller.name == CONTROLLER_NAME), None)
    if selected is None or selected.state != "running":
        raise RuntimeError(
            "{} must be running before MoveIt can execute; start it through "
            "panda_moveit_config/franka_control.launch".format(CONTROLLER_NAME))
    if selected.type != "position_controllers/JointTrajectoryController":
        raise RuntimeError("{} has unexpected type {}".format(
            CONTROLLER_NAME, selected.type))


def plan_with_moveit(position, orientation, state, args):
    """Plan a flange goal equivalent to the recorded end-effector pose."""
    group = moveit_commander.MoveGroupCommander("panda_arm", wait_for_servers=30.0)
    if group.get_planning_frame().lstrip("/") != BASE_FRAME:
        raise RuntimeError("MoveIt planning frame must be {}".format(BASE_FRAME))

    tool_transform = transform_matrix(state.F_T_EE)
    measured_flange = transform_matrix(state.O_T_EE) @ np.linalg.inv(tool_transform)
    model_pose = group.get_current_pose(FLANGE_LINK)
    if model_pose.header.frame_id.lstrip("/") != BASE_FRAME:
        raise RuntimeError("MoveIt flange pose is not expressed in {}".format(BASE_FRAME))
    model_flange = pose_matrix(
        [model_pose.pose.position.x, model_pose.pose.position.y, model_pose.pose.position.z],
        [model_pose.pose.orientation.x, model_pose.pose.orientation.y,
         model_pose.pose.orientation.z, model_pose.pose.orientation.w])
    frame_distance, frame_angle = transform_errors(model_flange, measured_flange)
    if frame_distance > 0.02 or frame_angle > 0.10:
        raise RuntimeError(
            "MoveIt flange pose disagrees with Franka state ({:.3f} m, {:.3f} rad); "
            "check robot frames and tool configuration".format(frame_distance, frame_angle))

    target_flange = pose_matrix(position, orientation) @ np.linalg.inv(tool_transform)
    goal_position, goal_orientation = pose_from_matrix(target_flange)
    goal = PoseStamped()
    goal.header.frame_id = BASE_FRAME
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = goal_position
    (goal.pose.orientation.x, goal.pose.orientation.y,
     goal.pose.orientation.z, goal.pose.orientation.w) = goal_orientation

    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)
    group.set_goal_position_tolerance(args.position_tolerance)
    group.set_goal_orientation_tolerance(args.orientation_tolerance)
    group.set_pose_target(goal, end_effector_link=FLANGE_LINK)
    try:
        success, plan, _, error = group.plan()
    finally:
        group.clear_pose_targets()
    if not success or not plan.joint_trajectory.points:
        raise RuntimeError("MoveIt could not plan to the start pose (code {})".format(
            error.val))
    duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
    if not math.isfinite(duration) or duration <= 0:
        raise RuntimeError("MoveIt returned an invalid trajectory duration")
    return group, plan, duration


def execute_plan(group, plan, start_state):
    """Execute only while the end effector and tool still match the planned start."""
    require_real_controller()
    state = require_still_pose(get_robot_state())
    if not np.allclose(state.F_T_EE, start_state.F_T_EE, atol=1e-6):
        raise RuntimeError("end-effector frame changed after planning; run the script again")
    start_position, start_orientation = pose_from_matrix(
        transform_matrix(start_state.O_T_EE))
    distance, angle = pose_errors(state, start_position, start_orientation)
    if distance > 0.01 or angle > 0.05:
        raise RuntimeError("end effector moved after planning; run the script again")
    try:
        if not group.execute(plan, wait=True):
            raise RuntimeError("MoveIt failed to execute the planned move")
    finally:
        group.stop()


def wait_for_target(position, orientation, args):
    deadline = time.monotonic() + 3.0
    distance = angle = math.inf
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        state = get_robot_state(timeout=remaining)
        distance, angle = pose_errors(state, position, orientation)
        if (distance <= args.position_tolerance
                and angle <= args.orientation_tolerance):
            return distance, angle
    raise RuntimeError(
        "start pose not reached: position error {:.4f} m, orientation error {:.4f} rad"
        .format(distance, angle))


def run():
    parser = argparse.ArgumentParser(
        description="Move the Panda to the first recorded end-effector pose")
    parser.add_argument("trajectory_file", nargs="?",
                        help="path to trajectory.txt (or set ~trajectory_file)")
    parser.add_argument("--velocity-scaling", type=float,
                        help="MoveIt joint velocity scaling in (0, 1] (default: 0.05)")
    parser.add_argument("--acceleration-scaling", type=float,
                        help="MoveIt joint acceleration scaling in (0, 1] (default: 0.05)")
    parser.add_argument("--max-position-distance", type=float,
                        help="largest start-pose translation in m (default: 0.30)")
    parser.add_argument("--max-orientation-distance", type=float,
                        help="largest start-pose rotation in rad (default: 1.57)")
    parser.add_argument("--position-tolerance", type=float,
                        help="final position error in m (default: 0.01)")
    parser.add_argument("--orientation-tolerance", type=float,
                        help="final rotation error in rad (default: 0.05)")
    parser.add_argument("--dry-run", action="store_true", default=None,
                        help="plan and check the path without moving")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_to_start_pose")
    args.trajectory_file = args.trajectory_file or rospy.get_param("~trajectory_file", "")
    if not isinstance(args.trajectory_file, str) or not args.trajectory_file.strip():
        parser.error("provide a trajectory_file argument or ~trajectory_file ROS parameter")
    defaults = {
        "velocity_scaling": 0.05,
        "acceleration_scaling": 0.05,
        "max_position_distance": 0.30,
        "max_orientation_distance": 1.57,
        "position_tolerance": 0.01,
        "orientation_tolerance": 0.05,
        "dry_run": False,
    }
    for name, default in defaults.items():
        if getattr(args, name) is None:
            setattr(args, name, rospy.get_param("~" + name, default))
    for name in ("max_position_distance", "max_orientation_distance",
                 "position_tolerance", "orientation_tolerance"):
        value = getattr(args, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            parser.error("--{} must be finite and positive".format(name.replace("_", "-")))
    for name in ("velocity_scaling", "acceleration_scaling"):
        value = getattr(args, name)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 1:
            parser.error("--{} must be in (0, 1]".format(name.replace("_", "-")))
    if not isinstance(args.dry_run, bool):
        parser.error("~dry_run must be true or false")

    position, orientation = read_first_point(args.trajectory_file)
    state = require_still_pose(get_robot_state(timeout=30.0))
    distance, angle = pose_errors(state, position, orientation)
    if distance <= args.position_tolerance and angle <= args.orientation_tolerance:
        rospy.loginfo("Robot is already at the recorded start pose")
        return
    if distance > args.max_position_distance or angle > args.max_orientation_distance:
        raise ValueError(
            "start pose is {:.3f} m / {:.3f} rad away; limits are {:.3f} m / {:.3f} rad"
            .format(distance, angle, args.max_position_distance,
                    args.max_orientation_distance))

    group, plan, duration = plan_with_moveit(position, orientation, state, args)
    if args.dry_run:
        rospy.loginfo("Dry run: MoveIt planned %.2f s to the start pose", duration)
        return

    rospy.loginfo("Executing MoveIt plan to recorded start pose (%.2f s)", duration)
    execute_plan(group, plan, state)
    distance, angle = wait_for_target(position, orientation, args)
    rospy.loginfo("Reached start pose: %.4f m, %.4f rad error", distance, angle)


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        return 1
    except (RuntimeError, ValueError, rospy.ROSException,
            MoveItCommanderException) as error:
        if rospy.core.is_initialized():
            rospy.logerr("Move to start failed: %s", error)
        else:
            print("Move to start failed: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
