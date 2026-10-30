"""Offline checks with real ROS messages; never connect to an API or robot."""
import copy
import ast
import json
import math
from pathlib import Path
import sys
import types
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image, JointState
from observation_receiver.msg import MultiCameraSynchronizedObservation
from tf.transformations import quaternion_about_axis, quaternion_matrix

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
import show_harness as harness
import evaluate_show_harness as evaluation


def decision(action="MV_LEFT", visible=True, axis="NONE"):
    return {"actions": [{"action": action, "axis": axis}],
            "target_visible_in_wrist": visible, "confidence": 0.95,
            "reasoning": "The target lies left of the fingers."}


def stage(stage_id="grasp", motion="GRASP"):
    return dict(zip(harness.STAGE_FIELDS, [stage_id, "red block", "body", motion,
                                           "Align and grasp the block", "Block visibly held"]))


def response(value):
    return types.SimpleNamespace(status="completed", output=[], output_text=json.dumps(value), usage=None)


def observation(stamp=100.0):
    value = MultiCameraSynchronizedObservation()
    value.camera_names = ["camera2", "camera1"]  # Intentionally not config order.
    for name in value.camera_names:
        image = Image()
        image.header.stamp = rospy.Time.from_sec(stamp)
        image.header.frame_id = name + "_optical"
        image.width = image.height = 2
        image.encoding = "rgb8"
        image.step = 6
        image.data = bytes([128] * 12)
        value.images.append(image)
    value.state.header.stamp = rospy.Time.from_sec(stamp)
    value.state.header.frame_id = "panda_link0"
    transform = np.eye(4)
    transform[:3, 3] = [0.5, 0, 0.3]
    value.state.O_T_EE = transform.flatten(order="F").tolist()
    value.state.F_T_EE = np.eye(4).flatten(order="F").tolist()
    value.state.q = [0] * 7
    value.state.dq = [0] * 7
    value.state.robot_mode = value.state.ROBOT_MODE_MOVE
    value.gripper_state = JointState()
    value.gripper_state.header.stamp = rospy.Time.from_sec(stamp)
    value.gripper_state.name = ["panda_finger_joint2", "panda_finger_joint1"]
    value.gripper_state.position = [0.04, 0.04]
    return value


class InterpreterTests(unittest.TestCase):
    def target(self, unit, **overrides):
        kwargs = dict(position=[0.5, 0, 0.3], orientation=[0, 0, 0, 1], unit=unit,
                      translation_step=0.02, rotation_step=math.radians(15),
                      rotation=np.diag([-1, -1, 1]), lower=np.array([0.35, -0.3, 0.12]),
                      upper=np.array([0.7, 0.3, 0.55]), table_height=0,
                      tip_offset=np.zeros(3), floor_clearance=0.005)
        kwargs.update(overrides)
        return harness.pose_target(**kwargs)

    def test_all_calibrated_translations(self):
        for action, vector in harness.TRANSLATIONS.items():
            position, quaternion = self.target({"action": action, "axis": "NONE"})
            np.testing.assert_allclose(position, [0.5, 0, 0.3] + 0.02 * (np.diag([-1, -1, 1]) @ vector))
            np.testing.assert_allclose(quaternion, [0, 0, 0, 1])

    def test_rotations_are_reference_frame_not_tool_frame(self):
        initial = quaternion_about_axis(math.pi / 2, [1, 0, 0])
        for axis_index, axis in enumerate(("x", "y", "z")):
            for action, sign in (("ROTATE_CCW", 1), ("ROTATE_CW", -1)):
                position, q = self.target({"action": action, "axis": axis}, orientation=initial)
                expected = quaternion_matrix(quaternion_about_axis(
                    sign * math.radians(15), np.diag([-1, -1, 1])[:, axis_index])) @ quaternion_matrix(initial)
                np.testing.assert_allclose(quaternion_matrix(q), expected, atol=1e-9)
                np.testing.assert_allclose(position, [0.5, 0, 0.3])

    def test_full_rotation_composition(self):
        q = np.array([0, 0, 0, 1])
        for _ in range(6):
            _, q = self.target({"action": "ROTATE_CCW", "axis": "z"}, orientation=q)
        np.testing.assert_allclose(quaternion_matrix(q), quaternion_matrix(
            quaternion_about_axis(math.pi / 2, [0, 0, 1])), atol=1e-9)

    def test_outside_workspace_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "workspace"):
            self.target({"action": "MV_BACK", "axis": "NONE"}, position=[0.69, 0, 0.3])

    def test_rotation_tip_sweep_checks_between_safe_endpoints(self):
        with self.assertRaisesRegex(ValueError, "tip sweep"):
            self.target({"action": "ROTATE_CCW", "axis": "y"},
                        rotation=np.eye(3), orientation=quaternion_about_axis(math.pi / 4, [0, 1, 0]),
                        rotation_step=math.pi / 2, position=[0.5, 0, 0.9],
                        upper=np.array([0.7, 0.3, 1.0]), tip_offset=np.array([0.5, 0, 0]),
                        table_height=0.5)

    def test_non_rigid_semantic_maps_rejected(self):
        for matrix in (np.diag([-1, 1, 1]), np.ones((3, 3)), [[1]], [[math.nan] * 3] * 3):
            with self.assertRaises(ValueError):
                harness.semantic_rotation(matrix)

    def test_gripper_width_uses_names(self):
        msg = JointState(name=["irrelevant", "right", "left"], position=[99, 0.01, 0.02])
        self.assertAlmostEqual(harness.gripper_width(msg, ["left", "right"]), 0.03)
        msg.name = ["left", "left", "right"]
        with self.assertRaises(ValueError):
            harness.gripper_width(msg, ["left", "right"])


class ContractTests(unittest.TestCase):
    def test_invalid_decisions(self):
        invalid = []
        for field, value in (("confidence", True), ("confidence", math.nan),
                             ("confidence", 1.1), ("reasoning", ""),
                             ("target_visible_in_wrist", "YES"), ("actions", [])):
            value_decision = decision()
            value_decision[field] = value
            invalid.append(value_decision)
        invalid += [decision("NOT_AN_ACTION"), decision(axis="x"), decision("ROTATE_CW"),
                    dict(decision(), unexpected="extra")]
        for value in invalid:
            with self.assertRaises(ValueError):
                harness.validate_decision(value, 3, True)

    def test_chunks_only_allow_distant_free_space_translations(self):
        value = decision(visible=False)
        value["actions"] *= 3
        harness.validate_decision(value, 3, True)
        for visible, action in ((True, "MV_LEFT"), (False, "GRASP"), (False, "MV_DOWN"), (False, "DONE")):
            modified = copy.deepcopy(value)
            modified["target_visible_in_wrist"] = visible
            modified["actions"][1]["action"] = action
            with self.assertRaises(ValueError):
                harness.validate_decision(modified, 3, True)

    def test_rotation_toggle(self):
        value = decision("ROTATE_CCW", axis="z")
        harness.validate_decision(value, 3, True)
        with self.assertRaises(ValueError):
            harness.validate_decision(value, 3, False)

    def test_plan_validation(self):
        harness.validate_plan({"subgoals": [stage()]}, 20)
        for value in ({"subgoals": []}, {"subgoals": [stage(), stage()]},
                      {"subgoals": [dict(stage(), completion="")]}, {"subgoals": [stage()], "extra": 0}):
            with self.assertRaises(ValueError):
                harness.validate_plan(value, 20)

    def test_refusal_and_incomplete_output(self):
        with self.assertRaises(ValueError):
            harness.parse_response(types.SimpleNamespace(status="incomplete"))
        with self.assertRaises(ValueError):
            harness.parse_response(types.SimpleNamespace(status="completed", output=[
                types.SimpleNamespace(content=[types.SimpleNamespace(type="refusal")])]))


class LoopTests(unittest.TestCase):
    def setUp(self):
        self.params = yaml.safe_load((PACKAGE / "config/show_harness.yaml").read_text())
        self.params["task_description"] = "Place the red block on the plate"
        patches = [patch.object(rospy, "get_param", side_effect=lambda key, default=None:
                                self.params.get(key.lstrip("~"), default)),
                   patch.object(rospy, "Publisher"), patch.object(rospy, "Subscriber"),
                   patch.object(rospy, "ServiceProxy"), patch.object(rospy, "on_shutdown"),
                   patch.object(rospy, "is_shutdown", return_value=False),
                   patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(100)),
                   patch.object(rospy, "loginfo"), patch.object(rospy, "logwarn_throttle"),
                   patch("openai.OpenAI")]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.node = harness.ShowHarness()
        self.node._sensor_callback(observation())
        self.node._execute_pose = Mock()
        self.node._gripper_command = Mock()
        self.node._gripper_command.return_value = (self.node.latest, np.array([0.5, 0, 0.3]), 0.08, True, "")
        self.node._check_controller = Mock()
        self.events = []
        self.node._record = lambda event: self.events.append(copy.deepcopy(event))

    def responses(self, *values):
        self.node.client.responses.create.side_effect = [response(value) for value in values]

    def test_dry_run_planner_and_multiview_api(self):
        self.responses({"subgoals": [stage()]}, decision())
        self.node.run()
        self.node._execute_pose.assert_not_called()
        self.node._gripper_command.assert_not_called()
        self.assertEqual(self.events[-1]["termination"], "dry_run_preview")
        calls = self.node.client.responses.create.call_args_list
        self.assertEqual(len(calls), 2)
        for call in calls:
            content = call.kwargs["input"][0]["content"]
            images = [part for part in content if part["type"] == "input_image"]
            self.assertEqual(len(images), 2)
            self.assertTrue(all(part["image_url"].startswith("data:image/jpeg;base64,") for part in images))
            self.assertTrue(call.kwargs["text"]["format"]["strict"])
            self.assertFalse(call.kwargs["store"])
        self.assertEqual(calls[0].kwargs["instructions"], self.params["planner_prompt"].strip())
        self.assertEqual(calls[1].kwargs["instructions"], self.params["controller_prompt"].strip())
        controller_context = json.loads(calls[1].kwargs["input"][0]["content"][0]["text"])
        self.assertEqual(controller_context["gripper_width_m"], 0.08)
        self.assertEqual(controller_context["ee_orientation_xyzw"], [0, 0, 0, 1])

    def test_far_target_selects_coarse_step(self):
        self.responses({"subgoals": [stage()]}, decision(visible=False))
        self.node.run()
        preview = next(item for item in self.events if item["outcome"] == "proposed")
        self.assertEqual(preview["step_m"], 0.04)
        np.testing.assert_allclose(preview["targets"][0]["target_position_m"], [0.5, 0.04, 0.3])

    def test_camera2_only_profile_plans_and_queries_with_one_image(self):
        self.params = yaml.safe_load((PACKAGE / "config/show_harness_camera2.yaml").read_text())
        self.params["task_description"] = "Move above the green marker"
        self.node = harness.ShowHarness()
        one_view = observation()
        one_view.camera_names = ["camera2"]
        one_view.images = one_view.images[:1]
        self.node._sensor_callback(one_view)
        self.node._record = lambda event: self.events.append(copy.deepcopy(event))
        self.responses({"subgoals": [stage()]}, decision("MV_DOWN"))
        with patch.object(self.node, "_execute_pose") as execute, \
                patch.object(self.node, "_gripper_command") as gripper:
            self.node.run()
        execute.assert_not_called()
        gripper.assert_not_called()
        self.assertEqual(self.events[-1]["termination"], "dry_run_preview")
        for call in self.node.client.responses.create.call_args_list:
            content = call.kwargs["input"][0]["content"]
            self.assertEqual(sum(part["type"] == "input_image" for part in content), 1)
            context = json.loads(content[0]["text"])
            self.assertEqual(context["available_cameras"], ["camera2"])
            self.assertTrue(context["single_camera"])
            self.assertEqual(context["reference_camera"], "camera2")
            self.assertEqual(context["wrist_camera"], "camera2")
        preview = next(event for event in self.events if event["outcome"] == "proposed")
        np.testing.assert_allclose(preview["targets"][0]["target_position_m"], [0.5, 0, 0.28])
        # Sharing camera roles does not allow a mismatched sensor name.
        self.node.latest.camera_names = ["camera1"]
        with self.assertRaisesRegex(ValueError, "camera names"):
            self.node._snapshot()

    def test_stale_secondary_camera_rejected(self):
        self.node.latest.images[1].header.stamp = rospy.Time.from_sec(99)
        with self.assertRaisesRegex(RuntimeError, "timestamp"):
            self.node._snapshot()

    def test_idle_wait_recovers_only_after_a_fresh_observation(self):
        self.node.received_at = time.monotonic() - 1.0
        with patch.object(harness.time, "sleep", side_effect=lambda _: self.node._sensor_callback(observation())):
            result = self.node._wait_for_fresh_observation()
        self.assertEqual(result[0].camera_names, ["camera2", "camera1"])
        self.assertEqual([event["outcome"] for event in self.events],
                         ["waiting_for_fresh_observation", "fresh_observation_resumed"])
        self.node._execute_pose.assert_not_called()

    def test_idle_wait_timeout_keeps_robot_stationary(self):
        self.node.received_at = time.monotonic() - 1.0
        self.node.observation_timeout = 0.001
        with self.assertRaisesRegex(harness.ObservationUnavailable, "fresh observation timeout"):
            self.node._wait_for_fresh_observation()
        self.node._execute_pose.assert_not_called()
        self.node._gripper_command.assert_not_called()

    def test_pose_drift_is_still_rejected_after_observation_wait(self):
        before = copy.deepcopy(self.node._snapshot())
        self.node.received_at = time.monotonic() - 1.0
        def receive_changed_pose(_):
            value = observation()
            transform = harness.rigid_transform(value.state.O_T_EE, "O_T_EE")
            transform[0, 3] += 0.01
            value.state.O_T_EE = transform.flatten(order="F").tolist()
            self.node._sensor_callback(value)
        with patch.object(harness.time, "sleep", side_effect=receive_changed_pose):
            with self.assertRaisesRegex(RuntimeError, "changed during"):
                self.node._fresh_stationary(before, wait_for_fresh=True)

    def test_motion_feedback_does_not_retry_stale_observations(self):
        self.node.received_at = time.monotonic() - 1.0
        with self.assertRaisesRegex(harness.ObservationUnavailable, "stream is not fresh"):
            self.node._wait_until_settled(np.array([0.5, 0, 0.3]), np.array([0, 0, 0, 1]))
        self.assertEqual(self.events, [])

    def test_expired_decision_cannot_be_revived_by_fresh_observations(self):
        self.node.decision_deadline = time.monotonic() - 1.0
        with self.assertRaisesRegex(RuntimeError, "decision expired"):
            self.node._fresh_stationary(self.node._snapshot(), wait_for_fresh=True)

    def test_planner_latency_followed_by_observation_gap_recovers(self):
        def api_response(**kwargs):
            if kwargs["text"]["format"]["name"] == "subgoal_plan":
                self.node.received_at = time.monotonic() - 1.0
                return response({"subgoals": [stage()]})
            return response(decision())
        self.node.client.responses.create.side_effect = api_response
        with patch.object(harness.time, "sleep", side_effect=lambda _: self.node._sensor_callback(observation())):
            self.node.run()
        self.assertEqual(self.events[-1]["termination"], "dry_run_preview")
        self.assertIn("fresh_observation_resumed", [event["outcome"] for event in self.events])
        self.assertEqual(self.node.client.responses.create.call_count, 2)

    def test_unknown_camera_name_rejected(self):
        self.node.latest.camera_names[0] = "unknown"
        with self.assertRaisesRegex(ValueError, "camera names"):
            self.node._snapshot()

    def test_orientation_drift_during_inference_rejected(self):
        self.node._wait_for_start()
        before = copy.deepcopy(self.node._snapshot())
        rotated = quaternion_matrix(quaternion_about_axis(0.2, [0, 0, 1]))
        rotated[:3, 3] = before[1]
        self.node.latest.state.O_T_EE = rotated.flatten(order="F").tolist()
        with self.assertRaisesRegex(RuntimeError, "changed"):
            self.node._fresh_stationary(before)

    def test_live_done_advances_subgoals_without_motion(self):
        self.node.dry_run = False
        self.responses({"subgoals": [stage(), stage("lift", "LIFT")]}, decision("DONE"), decision("DONE"))
        self.node.run()
        self.assertEqual(self.node.stage_index, 1)
        self.assertEqual(self.events[-1]["termination"], "model_done")
        self.assertIsNone(self.events[-1]["independent_success"])
        self.node._execute_pose.assert_not_called()

    def test_empty_grasp_reopens_and_rolls_back(self):
        self.node.grasp_stage_index = 0
        self.node.stage_index = 2
        self.node.executed_actions = 1
        self.node._recover_grasp("empty_grasp")
        self.node._gripper_command.assert_called_once_with("RELEASE")
        self.assertEqual(self.node.stage_index, 0)
        self.assertEqual(self.node.executed_actions, 2)
        self.assertFalse(self.node.holding)
        self.assertEqual(self.node.recovery_count, 1)

    def test_recovery_limit_prevents_additional_commands(self):
        self.node.recovery_count = self.node.max_recoveries
        with self.assertRaisesRegex(RuntimeError, "recovery budget"):
            self.node._recover_grasp("empty_grasp")
        self.node._gripper_command.assert_not_called()

    def test_chunk_workspace_rejection_before_any_motion(self):
        self.node.upper = np.array([0.7, 0.07, 0.55])
        value = decision(visible=False)
        value["actions"] *= 3
        self.responses({"subgoals": [stage()]}, value)
        with self.assertRaisesRegex(ValueError, "workspace"):
            self.node.run()
        self.node._execute_pose.assert_not_called()

    def test_live_rotation_reaches_orientation_and_updates_setpoint(self):
        self.node.dry_run = False
        self.node.max_decisions = 1
        self.responses({"subgoals": [stage()]}, decision("ROTATE_CCW", axis="z"))
        def execute(observed, target, quaternion):
            transform = quaternion_matrix(quaternion)
            transform[:3, 3] = target
            self.node.latest.state.O_T_EE = transform.flatten(order="F").tolist()
            return self.node.latest, target
        self.node._execute_pose.side_effect = execute
        self.node.run()
        self.node._execute_pose.assert_called_once()
        self.assertAlmostEqual(harness.orientation_distance(
            self.node.setpoint_orientation, [0, 0, 0, 1]), math.radians(15))
        self.assertEqual(self.events[-1]["executed_actions"] if "executed_actions" in self.events[-1]
                         else self.node.executed_actions, 1)

    def test_gripper_waits_for_result_then_measures_named_width(self):
        from actionlib_msgs.msg import GoalStatus
        self.node.grasp_client = Mock()
        self.node.grasp_client.get_state.return_value = GoalStatus.SUCCEEDED
        self.node.grasp_client.get_result.return_value = types.SimpleNamespace(success=True, error="")
        self.node.grasp_client.wait_for_result.return_value = True
        self.node.settle_time = 0.001
        counter = [101.0]
        def snapshot():
            counter[0] += 0.01
            value = observation(counter[0])
            value.gripper_state.position = [0.012, 0.012]
            return value, np.array([0.5, 0, 0.3]), np.array([0, 0, 0, 1])
        self.node._snapshot = snapshot
        _, _, width, success, _ = harness.ShowHarness._gripper_command(self.node, "GRASP")
        self.assertAlmostEqual(width, 0.024)
        self.assertTrue(success)
        goal = self.node.grasp_client.send_goal.call_args[0][0]
        self.assertEqual(goal.width, 0.0)
        self.assertEqual(goal.force, 20.0)
        self.assertEqual(goal.epsilon.outer, 0.08)

    def test_gripper_changes_allowed_during_command_but_not_inference(self):
        self.node._wait_for_start()
        before = copy.deepcopy(self.node._snapshot())
        self.node.latest.gripper_state.position = [0.01, 0.01]
        with self.assertRaisesRegex(RuntimeError, "changed"):
            self.node._fresh_stationary(before)
        self.node._fresh_stationary(before, check_gripper=False)

    def test_moveit_controller_accepts_real_trajectory_execution(self):
        self.params["/move_group/moveit_controller_manager"] = "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        self.params["/move_group/controller_list"] = [{
            "name": "position_joint_trajectory_controller", "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory", "joints": harness.PANDA_JOINTS}]
        controller = types.SimpleNamespace(name="position_joint_trajectory_controller", state="running",
                                           type="position_controllers/JointTrajectoryController")
        self.node.controller_service.return_value.controller = [controller]
        with patch.object(rospy, "wait_for_service"):
            harness.ShowHarness._check_controller(self.node)

    def test_moveit_fake_controller_is_rejected(self):
        self.params["/move_group/moveit_controller_manager"] = "moveit_fake_controller_manager/MoveItFakeControllerManager"
        with self.assertRaisesRegex(RuntimeError, "fake"):
            harness.ShowHarness._check_controller(self.node)

    def test_empty_close_result_triggers_recovery_in_live_loop(self):
        self.node.dry_run = False
        self.node.max_decisions = 1
        self.responses({"subgoals": [stage()]}, decision("GRASP"))
        self.node._gripper_command.side_effect = [
            (self.node.latest, np.array([0.5, 0, 0.3]), 0.0, True, ""),
            (self.node.latest, np.array([0.5, 0, 0.3]), 0.08, True, "")]
        self.node.run()
        self.assertEqual([call.args[0] for call in self.node._gripper_command.call_args_list], ["GRASP", "RELEASE"])
        self.assertEqual(self.node.executed_actions, 2)
        self.assertEqual(self.node.recovery_count, 1)

    def test_api_exception_ends_episode_without_robot_command(self):
        self.node.client.responses.create.side_effect = RuntimeError("API unavailable")
        with self.assertRaisesRegex(RuntimeError, "API unavailable"):
            self.node.run()
        self.assertEqual(self.events[-1]["termination"], "aborted")
        self.node._execute_pose.assert_not_called()
        self.node._gripper_command.assert_not_called()

    def test_moveit_settling_requires_orientation(self):
        self.node._wait_for_start()
        self.node.motion_timeout = 0.02
        self.node.settle_poll_rate = 1000
        self.node.settle_time = 0.001
        with self.assertRaisesRegex(RuntimeError, "did not settle"):
            self.node._wait_until_settled(np.array([0.5, 0, 0.3]),
                                        quaternion_about_axis(math.radians(15), [0, 0, 1]))

    def test_moveit_tool_frame_conversion_for_rotation(self):
        self.node.group = Mock()
        self.node.robot = Mock()
        self.node._verify_moveit_state = Mock()
        trajectory = Mock()
        trajectory.joint_trajectory.points = [Mock()]
        trajectory.joint_trajectory.points[-1].time_from_start.to_sec.return_value = 1.0
        trajectory.joint_trajectory.joint_names = harness.PANDA_JOINTS
        self.node.group.compute_cartesian_path.return_value = (trajectory, 1.0)
        self.node.group.retime_trajectory.return_value = trajectory
        value = observation()
        tool = np.eye(4)
        tool[2, 3] = 0.1
        value.state.F_T_EE = tool.flatten(order="F").tolist()
        q = quaternion_about_axis(math.pi / 2, [1, 0, 0])
        self.node._plan(value, np.array([0.5, 0, 0.3]), q)
        goal = self.node.group.compute_cartesian_path.call_args[0][0][0]
        expected_ee = quaternion_matrix(q)
        expected_ee[:3, 3] = [0.5, 0, 0.3]
        np.testing.assert_allclose(harness.matrix_from_pose(goal), expected_ee @ np.linalg.inv(tool), atol=1e-9)

    def test_incomplete_cartesian_plan_rejected(self):
        self.node.group = Mock()
        self.node._verify_moveit_state = Mock()
        self.node.group.compute_cartesian_path.return_value = (Mock(), 0.5)
        with self.assertRaisesRegex(RuntimeError, "requested Cartesian path"):
            self.node._plan(observation(), np.array([0.5, 0, 0.3]), np.array([0, 0, 0, 1]))
        self.node.group.execute.assert_not_called()

    def test_pose_drift_after_planning_prevents_execution(self):
        self.node.group = Mock()
        before = self.node._snapshot()
        self.node._plan = Mock(return_value=(Mock(), 1.0))
        after_observation = copy.deepcopy(before[0])
        matrix = harness.rigid_transform(after_observation.state.O_T_EE, "O_T_EE")
        matrix[0, 3] += 0.01
        after_observation.state.O_T_EE = matrix.flatten(order="F").tolist()
        after = (after_observation, matrix[:3, 3], before[2])
        self.node._snapshot = Mock(side_effect=[before, after])
        with self.assertRaisesRegex(RuntimeError, "after MoveIt planning"):
            self.node._execute(before[0], before[1], before[1], before[2])
        self.node.group.execute.assert_not_called()

    def test_prompts_can_be_changed_in_config(self):
        self.params["planner_prompt"] = "Custom planner instructions"
        self.params["controller_prompt"] = "Custom controller instructions"
        node = harness.ShowHarness()
        self.assertEqual(node.planner_prompt, self.params["planner_prompt"])
        self.assertEqual(node.controller_prompt, self.params["controller_prompt"])

    def test_invalid_prompts_fail_at_startup(self):
        for prompt_name in ("planner_prompt", "controller_prompt"):
            original = self.params[prompt_name]
            for value in ("", "   ", None, 123):
                self.params[prompt_name] = value
                with self.assertRaisesRegex(ValueError, prompt_name):
                    harness.ShowHarness()
            self.params[prompt_name] = original

    def test_script_has_no_baseline_or_custom_controller_dependency(self):
        source = (PACKAGE / "scripts/show_harness.py").read_text()
        tree = ast.parse(source)
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        imports.extend(alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names)
        self.assertFalse(any(name.startswith(("gpt_discrete_inference", "panda_custom_controllers")) for name in imports))
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ShowHarness")
        self.assertEqual(cls.bases, [])


class EvaluationTests(unittest.TestCase):
    def log(self, directory, episode_id, dry_run, model_success):
        common = {"episode_id": episode_id, "dry_run": dry_run}
        events = [dict(common, outcome="episode_start"),
                  dict(common, outcome="model_response", api_seconds=2.0,
                       usage={"input_tokens": 10, "output_tokens": 5, "total_tokens": 15}),
                  dict(common, outcome="episode_end", termination="model_done" if model_success else "decision_budget",
                       model_claimed_success=model_success, executed_actions=4, api_calls=2, episode_seconds=8.0)]
        path = Path(directory) / (episode_id + ".jsonl")
        path.write_text("\n".join(json.dumps(event) for event in events))
        return path

    def test_success_labels_and_dry_runs_are_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = [self.log(directory, "live1", False, True),
                     self.log(directory, "live2", False, False),
                     self.log(directory, "preview", True, True)]
            result = evaluation.summarize(paths, {"live1": False, "live2": True, "preview": True})
            self.assertEqual(result["live_episodes"], 2)
            self.assertEqual(result["dry_run_previews"], 1)
            self.assertEqual(result["independent_success_rate"], 0.5)
            self.assertEqual(result["model_done_rate"], 0.5)
            self.assertEqual(result["tokens_all"]["total_tokens"], 45)

    def test_unlabeled_success_is_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            result = evaluation.summarize([self.log(directory, "live", False, True)])
            self.assertIsNone(result["independent_success_rate"])


if __name__ == "__main__":
    unittest.main()
