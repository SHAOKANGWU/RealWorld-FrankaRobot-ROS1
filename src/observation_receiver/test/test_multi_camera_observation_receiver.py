"""Run after generating messages: python3 -m unittest discover -s <package>/test.

Uses the real ROS message types and synchronizer without contacting a ROS master.
"""

import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import message_filters
import rospy
import yaml

from franka_msgs.msg import FrankaState
from observation_receiver.msg import MultiCameraSynchronizedObservation
from sensor_msgs.msg import Image, JointState


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "multi_camera_receiver", PACKAGE / "scripts/multi_camera_observation_receiver_node.py"
)
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


class MultiCameraReceiverTest(unittest.TestCase):
    def setUp(self):
        with (PACKAGE / "config/multi_camera_observation_receiver.yaml").open() as config:
            self.params = yaml.safe_load(config)
        for target, options in [
            ("get_param", {"side_effect": lambda name, default=None: self.params.get(name.lstrip("~"), default)}),
            ("resolve_name", {"side_effect": lambda name: "/" + name.lstrip("/")}),
            ("Publisher", {"side_effect": lambda *args, **kwargs: Mock()}),
            ("loginfo", {}),
            ("loginfo_throttle", {}),
            ("logwarn", {}),
            ("Timer", {}),
        ]:
            patcher = patch.object(rospy, target, **options)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = patch.object(rospy.Time, "now", return_value=rospy.Time(100))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = patch.object(
            message_filters, "Subscriber",
            side_effect=lambda *args, **kwargs: message_filters.SimpleFilter(),
        )
        self.subscriber = patcher.start()
        self.addCleanup(patcher.stop)

    @staticmethod
    def stamped(message_type, seconds, frame=""):
        message = message_type()
        message.header.stamp = rospy.Time.from_sec(seconds)
        message.header.frame_id = frame
        return message

    def test_camera_counts_order_and_serialization(self):
        for count in (1, 2, 3):
            with self.subTest(camera_count=count):
                self.params["cameras"] = [
                    {"name": "camera_{}".format(i), "image_topic": "/camera_{}/image".format(i)}
                    for i in range(count)
                ]
                receiver = NODE.MultiCameraObservationReceiver()
                images = [self.stamped(Image, 10 + i * 0.005, "optical_{}".format(i))
                          for i in range(count)]
                state = self.stamped(FrankaState, 10.015)
                gripper = self.stamped(JointState, 10.02)
                # Reverse arrival order must not change configured image ordering.
                receiver.gripper_subscriber.signalMessage(gripper)
                receiver.state_subscriber.signalMessage(state)
                for index in reversed(range(count)):
                    receiver.image_subscribers[index].signalMessage(images[index])
                receiver.publisher.publish.assert_called_once()
                observation = receiver.publisher.publish.call_args[0][0]
                self.assertEqual(observation.camera_names, receiver.camera_names)
                self.assertEqual(observation.images, images)
                self.assertEqual(observation.header, images[0].header)
                self.assertEqual(observation.state, state)
                self.assertEqual(observation.gripper_state, gripper)
                buffer = io.BytesIO()
                observation.serialize(buffer)
                restored = MultiCameraSynchronizedObservation()
                restored.deserialize(buffer.getvalue())
                self.assertEqual(restored, observation)

    def test_missing_camera_waits_until_matching_image_arrives(self):
        receiver = NODE.MultiCameraObservationReceiver()
        receiver.image_subscribers[0].signalMessage(self.stamped(Image, 10))
        receiver.state_subscriber.signalMessage(self.stamped(FrankaState, 10.01))
        receiver.gripper_subscriber.signalMessage(self.stamped(JointState, 10.02))
        receiver.publisher.publish.assert_not_called()
        # An image outside the synchronization window must not complete the set.
        receiver.image_subscribers[1].signalMessage(self.stamped(Image, 10.2))
        receiver.publisher.publish.assert_not_called()
        receiver.image_subscribers[1].signalMessage(self.stamped(Image, 10.03))
        receiver.publisher.publish.assert_called_once()

    def test_invalid_camera_configuration_fails_before_subscribing(self):
        camera = {"name": "front", "image_topic": "/front/image"}
        for cameras in (
            [], "front", [None], [{}], [{"name": "", "image_topic": "/image"}],
            [{"name": "front", "image_topic": ""}],
            [camera, {"name": "front", "image_topic": "/other/image"}],
            [camera, {"name": "other", "image_topic": "front/image"}],
        ):
            with self.subTest(cameras=cameras):
                self.params["cameras"] = cameras
                with self.assertRaises(ValueError):
                    NODE.MultiCameraObservationReceiver()
        self.subscriber.assert_not_called()

    def test_main_starts_the_new_node(self):
        with patch.object(rospy, "init_node") as init, \
                patch.object(NODE, "MultiCameraObservationReceiver") as receiver, \
                patch.object(rospy, "spin") as spin:
            NODE.main()
        init.assert_called_once_with("multi_camera_observation_receiver")
        receiver.assert_called_once_with()
        spin.assert_called_once_with()

    def test_diagnostics_identify_missing_camera(self):
        receiver = NODE.MultiCameraObservationReceiver()
        receiver.image_subscribers[0].signalMessage(self.stamped(Image, 100))
        receiver.state_subscriber.signalMessage(self.stamped(FrankaState, 100))
        receiver.gripper_subscriber.signalMessage(self.stamped(JointState, 100))
        receiver._report_health(None)
        report = json.loads(rospy.logwarn.call_args[0][-1])
        self.assertEqual(report["missing_or_stale_inputs"], ["camera2"])
        self.assertEqual(report["inputs"]["camera1"]["received_messages"], 1)
        self.assertIsNone(report["output_receive_age_seconds"])

    def test_diagnostics_distinguish_synchronization_from_missing_sources(self):
        receiver = NODE.MultiCameraObservationReceiver()
        receiver.image_subscribers[0].signalMessage(self.stamped(Image, 99.7))
        receiver.image_subscribers[1].signalMessage(self.stamped(Image, 99.9))
        receiver.state_subscriber.signalMessage(self.stamped(FrankaState, 100))
        receiver.gripper_subscriber.signalMessage(self.stamped(JointState, 100))
        receiver._report_health(None)
        report = json.loads(rospy.logwarn.call_args[0][-1])
        self.assertEqual(report["missing_or_stale_inputs"], [])
        self.assertIn("synchronized output", rospy.logwarn.call_args[0][0])
        receiver.publisher.publish.assert_not_called()

    def test_diagnostics_measure_synchronized_publications(self):
        receiver = NODE.MultiCameraObservationReceiver()
        receiver.image_subscribers[0].signalMessage(self.stamped(Image, 100))
        receiver.image_subscribers[1].signalMessage(self.stamped(Image, 100))
        receiver.state_subscriber.signalMessage(self.stamped(FrankaState, 100))
        receiver.gripper_subscriber.signalMessage(self.stamped(JointState, 100))
        receiver._report_health(None)
        report = json.loads(rospy.loginfo.call_args[0][-1])
        self.assertEqual(report["missing_or_stale_inputs"], [])
        self.assertGreater(report["output_rate_hz"], 0)
        self.assertEqual(receiver.publication_count, 1)


if __name__ == "__main__":
    unittest.main()
