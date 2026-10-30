#!/usr/bin/env python3

import message_filters
import rospy

from observation_receiver.msg import SynchronizedObservation
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import Image, JointState


class ObservationReceiver:
    def __init__(self):
        image_topic = rospy.get_param("~image_topic")
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
            SynchronizedObservation,
            queue_size=publisher_queue_size,
        )

        self.image_subscriber = message_filters.Subscriber(
            image_topic,
            Image,
            queue_size=image_queue_size,
            buff_size=image_buffer_size,
        )

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
            [
                self.image_subscriber,
                self.state_subscriber,
                self.gripper_subscriber,
            ],
            queue_size=sync_queue_size,
            slop=sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.observation_callback)

        rospy.loginfo("Observation receiver started")
        rospy.loginfo("Image topic: %s", image_topic)
        rospy.loginfo("State topic: %s", state_topic)
        rospy.loginfo("Gripper topic: %s", gripper_topic)
        rospy.loginfo("Output topic: %s", output_topic)

    def observation_callback(
        self,
        image_message,
        state_message,
        gripper_message,
    ):
        observation = SynchronizedObservation()
        observation.header = image_message.header
        observation.image = image_message
        observation.state = state_message
        observation.gripper_state = gripper_message
        self.publisher.publish(observation)

        stamps = [
            image_message.header.stamp,
            state_message.header.stamp,
            gripper_message.header.stamp,
        ]
        time_span = (max(stamps) - min(stamps)).to_sec()
        rospy.loginfo_throttle(
            1.0,
            "Published synchronized observation (timestamp span: %.4f s)",
            time_span,
        )


def main():
    rospy.init_node("observation_receiver")
    ObservationReceiver()
    rospy.spin()


if __name__ == "__main__":
    main()
