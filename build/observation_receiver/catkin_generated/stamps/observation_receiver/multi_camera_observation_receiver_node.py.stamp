#!/usr/bin/env python3

import message_filters
import rospy

from observation_receiver.msg import MultiCameraSynchronizedObservation
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import Image, JointState


class MultiCameraObservationReceiver:
    def __init__(self):
        cameras = rospy.get_param("~cameras")
        if not isinstance(cameras, list) or not cameras:
            raise ValueError("~cameras must be a non-empty list of named image topics")

        self.camera_names = []
        image_topics = []
        for camera in cameras:
            if not isinstance(camera, dict):
                raise ValueError("Each ~cameras entry must contain name and image_topic")
            name = camera.get("name")
            topic = camera.get("image_topic")
            if not isinstance(name, str) or not name.strip():
                raise ValueError("Each camera must have a non-empty string name")
            if not isinstance(topic, str) or not topic.strip():
                raise ValueError("Each camera must have a non-empty string image_topic")
            if name in self.camera_names:
                raise ValueError("Duplicate camera name: {}".format(name))
            topic = rospy.resolve_name(topic)
            if topic in image_topics:
                raise ValueError("Duplicate camera image topic: {}".format(topic))
            self.camera_names.append(name)
            image_topics.append(topic)

        state_topic = rospy.get_param("~state_topic")
        gripper_topic = rospy.get_param("~gripper_topic")
        output_topic = rospy.get_param("~output_topic")

        publisher_queue_size = rospy.get_param("~publisher_queue_size")
        image_queue_size = rospy.get_param("~image_queue_size")
        image_buffer_size = rospy.get_param("~image_buffer_size")
        state_queue_size = rospy.get_param("~state_queue_size")
        gripper_queue_size = rospy.get_param("~gripper_queue_size")

        sync_queue_size = rospy.get_param("~sync_queue_size")
        sync_slop = rospy.get_param("~sync_slop")

        self.publisher = rospy.Publisher(
            output_topic,
            MultiCameraSynchronizedObservation,
            queue_size=publisher_queue_size,
        )

        self.image_subscribers = [
            message_filters.Subscriber(
                image_topic,
                Image,
                queue_size=image_queue_size,
                buff_size=image_buffer_size,
            )
            for image_topic in image_topics
        ]

        self.state_subscriber = message_filters.Subscriber(
            state_topic,
            FrankaState,
            queue_size=state_queue_size,
        )

        self.gripper_subscriber = message_filters.Subscriber(
            gripper_topic,
            JointState,
            queue_size=gripper_queue_size,
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.image_subscribers + [self.state_subscriber, self.gripper_subscriber],
            queue_size=sync_queue_size,
            slop=sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.observation_callback)

        rospy.loginfo("Multi-camera observation receiver started")
        for name, topic in zip(self.camera_names, image_topics):
            rospy.loginfo("Camera %s image topic: %s", name, topic)
        rospy.loginfo("State topic: %s", state_topic)
        rospy.loginfo("Gripper topic: %s", gripper_topic)
        rospy.loginfo("Output topic: %s", output_topic)

    def observation_callback(self, *messages):
        # Synchronizer inputs follow the configured camera order, then state
        # and gripper. Every input is required before an observation is emitted.
        image_messages = list(messages[:-2])
        state_message, gripper_message = messages[-2:]
        observation = MultiCameraSynchronizedObservation()
        # Use the first configured camera as the observation's reference header.
        # Each image retains its own timestamp and optical frame.
        observation.header = image_messages[0].header
        observation.camera_names = list(self.camera_names)
        observation.images = image_messages
        observation.state = state_message
        observation.gripper_state = gripper_message
        self.publisher.publish(observation)

        stamps = [message.header.stamp for message in messages]
        time_span = (max(stamps) - min(stamps)).to_sec()
        rospy.loginfo_throttle(
            1.0,
            "Published multi-camera synchronized observation (timestamp span: %.4f s)",
            time_span,
        )


def main():
    rospy.init_node("multi_camera_observation_receiver")
    MultiCameraObservationReceiver()
    rospy.spin()


if __name__ == "__main__":
    main()
