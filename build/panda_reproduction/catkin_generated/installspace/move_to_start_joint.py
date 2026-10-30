#!/usr/bin/env python3

"""Move to the first recorded joint configuration in trajectory.txt."""

import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
import rospy


JOINT_NAMES = ["panda_joint{}".format(index) for index in range(1, 8)]
CONTROLLER_NAME = "position_joint_trajectory_controller"
STATE_TOPIC = "/franka_state_controller/franka_states"
ROW_LENGTH = 15
MAX_START_JOINT_SPEED = 0.05  # rad/s


def read_first_point(path):
    """Read the first nonempty, noncomment 15-value trajectory row."""
    path = Path(path).expanduser()
    try:
        with path.open("r") as stream:
            for line_number, line in enumerate(stream, start=1):
                content = line.split("#", 1)[0].strip()
                if not content:
                    continue
                try:
                    row = [float(value) for value in content.replace(",", " ").split()]
                except ValueError as error:
                    raise ValueError("{}:{} contains a non-numeric value".format(
                        path, line_number)) from error
                if len(row) != ROW_LENGTH or not all(math.isfinite(x) for x in row):
                    raise ValueError("{}:{} must contain 15 finite values".format(
                        path, line_number))
                if row[-1] not in (0.0, 1.0):
                    raise ValueError("{}:{} gripper action must be 0 or 1".format(
                        path, line_number))
                orientation = row[3:7]
                norm = math.hypot(*orientation)
                if norm < 1e-6:
                    raise ValueError("{}:{} has a zero quaternion".format(
                        path, line_number))
                return row[:3], [value / norm for value in orientation], row[7:14]
    except OSError as error:
        raise ValueError("cannot read '{}': {}".format(path, error)) from error
    raise ValueError("'{}' contains no trajectory points".format(path))


def check_joint_limits(target, margin):
    """Use the active robot's URDF limits rather than copied numeric limits."""
    try:
        description = rospy.get_param("/robot_description")
        root = ET.fromstring(description)
    except (KeyError, ET.ParseError) as error:
        raise RuntimeError("/robot_description is unavailable or invalid") from error
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for name, value in zip(JOINT_NAMES, target):
        joint = joints.get(name)
        limit = joint.find("limit") if joint is not None else None
        if (limit is None or limit.get("lower") is None
                or limit.get("upper") is None):
            raise RuntimeError("/robot_description has no limits for {}".format(name))
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        if not all(math.isfinite(x) for x in (lower, upper)):
            raise RuntimeError("/robot_description has invalid limits for {}".format(name))
        if not lower + margin <= value <= upper - margin:
            raise ValueError("{} target {:.4f} rad is outside [{:.4f}, {:.4f}] rad"
                             .format(name, value, lower + margin, upper - margin))


def get_robot_state(timeout=5.0):
    try:
        state = rospy.wait_for_message(STATE_TOPIC, FrankaState, timeout=timeout)
    except rospy.ROSException as error:
        raise RuntimeError("no Franka state received on {}".format(STATE_TOPIC)) from error
    if len(state.q) != len(JOINT_NAMES) or not all(math.isfinite(q) for q in state.q):
        raise RuntimeError("Franka state has invalid joint positions")
    if len(state.dq) != len(JOINT_NAMES) or not all(math.isfinite(dq) for dq in state.dq):
        raise RuntimeError("Franka state has invalid joint velocities")
    if len(state.O_T_EE) != 16 or not all(math.isfinite(x) for x in state.O_T_EE):
        raise RuntimeError("Franka state has an invalid end-effector transform")
    if state.header.stamp != rospy.Time():
        age = (rospy.Time.now() - state.header.stamp).to_sec()
        if abs(age) > 1.0:
            raise RuntimeError("Franka state timestamp is not current ({:.2f} s)".format(age))
    return state


def require_still_arm(state):
    speed = max(abs(dq) for dq in state.dq)
    if speed > MAX_START_JOINT_SPEED:
        raise RuntimeError("arm is still moving at {:.3f} rad/s; wait until it stops".format(
            speed))


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


def plan_with_moveit(target, current, args):
    """Plan to the recorded joints and check the complete planned path."""
    group = moveit_commander.MoveGroupCommander("panda_arm", wait_for_servers=30.0)
    if set(group.get_active_joints()) != set(JOINT_NAMES):
        raise RuntimeError("MoveIt planning group panda_arm does not contain the Panda arm joints")
    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)
    group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
    success, plan, _, error = group.plan()
    if not success or not plan.joint_trajectory.points:
        raise RuntimeError("MoveIt could not plan to the start point (code {})".format(
            error.val))
    trajectory = plan.joint_trajectory
    if len(trajectory.joint_names) != len(JOINT_NAMES) or set(trajectory.joint_names) != set(JOINT_NAMES):
        raise RuntimeError("MoveIt returned a trajectory for unexpected joints")
    indices = [trajectory.joint_names.index(name) for name in JOINT_NAMES]
    for point in trajectory.points:
        if len(point.positions) != len(JOINT_NAMES) or not all(
                math.isfinite(value) for value in point.positions):
            raise RuntimeError("MoveIt returned an invalid trajectory point")
        excursion = max(abs(point.positions[index] - start)
                        for index, start in zip(indices, current))
        if excursion > args.max_joint_distance:
            raise RuntimeError(
                "planned path moves {:.3f} rad from the starting position on one joint; "
                "limit is {:.3f} rad".format(excursion, args.max_joint_distance))
    first = [trajectory.points[0].positions[index] for index in indices]
    last = [trajectory.points[-1].positions[index] for index in indices]
    if max(abs(actual - expected) for actual, expected in zip(first, current)) > 0.02:
        raise RuntimeError("MoveIt's planned start differs from the measured robot state")
    if max(abs(actual - expected) for actual, expected in zip(last, target)) > args.joint_tolerance:
        raise RuntimeError("MoveIt's planned goal does not reach the recorded joints")
    duration = trajectory.points[-1].time_from_start.to_sec()
    if not math.isfinite(duration) or duration <= 0:
        raise RuntimeError("MoveIt returned an invalid trajectory duration")
    return group, plan, first, duration


def execute_plan(group, plan, planned_start):
    """Execute only while the arm is still at the plan's starting point."""
    require_real_controller()
    state = get_robot_state()
    require_still_arm(state)
    if max(abs(actual - expected) for actual, expected in zip(state.q, planned_start)) > 0.02:
        raise RuntimeError("robot moved after planning; run the script again")
    try:
        if not group.execute(plan, wait=True):
            raise RuntimeError("MoveIt failed to execute the planned move")
    finally:
        group.stop()


def wait_for_target(target, tolerance):
    deadline = time.monotonic() + 3.0
    joint_error = math.inf
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        state = get_robot_state(timeout=remaining)
        joint_error = max(abs(a - b) for a, b in zip(state.q, target))
        if joint_error <= tolerance:
            return joint_error
    raise RuntimeError("start joints not reached: maximum error {:.4f} rad".format(
        joint_error))


def run():
    parser = argparse.ArgumentParser(
        description="Move the Panda to the first joint configuration in trajectory.txt")
    parser.add_argument("trajectory_file", nargs="?",
                        help="path to trajectory.txt (or set ~trajectory_file)")
    parser.add_argument("--velocity-scaling", type=float,
                        help="MoveIt joint velocity scaling in (0, 1] (default: 0.05)")
    parser.add_argument("--acceleration-scaling", type=float,
                        help="MoveIt joint acceleration scaling in (0, 1] (default: 0.05)")
    parser.add_argument("--max-joint-distance", type=float,
                        help="largest planned joint excursion in rad (default: 1.0)")
    parser.add_argument("--joint-tolerance", type=float,
                        help="final joint error tolerance in rad (default: 0.02)")
    parser.add_argument("--joint-limit-margin", type=float,
                        help="clearance from URDF joint limits in rad (default: 0.01)")
    parser.add_argument("--dry-run", action="store_true", default=None,
                        help="plan and check the path without moving")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_to_start_joint")
    args.trajectory_file = args.trajectory_file or rospy.get_param("~trajectory_file", "")
    if not isinstance(args.trajectory_file, str) or not args.trajectory_file.strip():
        parser.error("provide a trajectory_file argument or ~trajectory_file ROS parameter")
    defaults = {
        "velocity_scaling": 0.05,
        "acceleration_scaling": 0.05,
        "max_joint_distance": 1.0,
        "joint_tolerance": 0.02,
        "joint_limit_margin": 0.01,
        "dry_run": False,
    }
    for name, default in defaults.items():
        if getattr(args, name) is None:
            setattr(args, name, rospy.get_param("~" + name, default))
    for name in ("max_joint_distance", "joint_tolerance"):
        value = getattr(args, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            parser.error("--{} must be finite and positive".format(name.replace("_", "-")))
    if (type(args.joint_limit_margin) not in (int, float)
            or not math.isfinite(args.joint_limit_margin)
            or args.joint_limit_margin < 0):
        parser.error("--joint-limit-margin must be finite and nonnegative")
    for name in ("velocity_scaling", "acceleration_scaling"):
        value = getattr(args, name)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 1:
            parser.error("--{} must be in (0, 1]".format(name.replace("_", "-")))
    if not isinstance(args.dry_run, bool):
        parser.error("~dry_run must be true or false")

    _, _, target = read_first_point(args.trajectory_file)
    check_joint_limits(target, args.joint_limit_margin)
    state = get_robot_state(timeout=30.0)
    require_still_arm(state)
    errors = [abs(a - b) for a, b in zip(state.q, target)]
    if max(errors) <= args.joint_tolerance:
        rospy.loginfo("Robot is already at the recorded start joints")
        return
    if max(errors) > args.max_joint_distance:
        raise ValueError("start point is {:.3f} rad away on one joint; limit is {:.3f} rad"
                         .format(max(errors), args.max_joint_distance))

    group, plan, planned_start, duration = plan_with_moveit(target, state.q, args)
    if args.dry_run:
        rospy.loginfo("Dry run: MoveIt planned %.2f s to the start joints", duration)
        return

    rospy.loginfo("Executing MoveIt plan to start joints (%.2f s)", duration)
    execute_plan(group, plan, planned_start)
    joint_error = wait_for_target(target, args.joint_tolerance)
    rospy.loginfo("Reached start joints: maximum error %.4f rad", joint_error)


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
