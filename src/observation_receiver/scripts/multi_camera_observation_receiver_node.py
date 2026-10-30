#!/usr/bin/env python3

import json
import math
import threading
import time

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
        self.diagnostic_period = float(rospy.get_param("~diagnostic_period", 5.0))
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.5))
        if any(not math.isfinite(value) or value <= 0
               for value in (self.diagnostic_period, self.max_input_age)):
            raise ValueError("diagnostic_period and max_input_age must be positive and finite")
        self.health_lock = threading.Lock()
        self.input_health = {}
        self.previous_counts = {}
        self.last_diagnostic_at = time.monotonic()
        self.last_publication_at = None
        self.publication_count = 0
        self.previous_publication_count = 0
        self.output_topic = output_topic

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

        # Record arrivals before the synchronizer consumes them, so a missing
        # match can be distinguished from a missing source topic.
        input_labels = self.camera_names + ["robot_state", "gripper_state"]
        input_topics = image_topics + [state_topic, gripper_topic]
        inputs = self.image_subscribers + [self.state_subscriber, self.gripper_subscriber]
        for label, topic, subscriber in zip(input_labels, input_topics, inputs):
            self.input_health[label] = {"topic": topic, "received_at": None, "stamp": None, "count": 0}
            self.previous_counts[label] = 0
            subscriber.registerCallback(self._input_callback, label)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            inputs,
            queue_size=sync_queue_size,
            slop=sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.observation_callback)
        self.health_timer = rospy.Timer(rospy.Duration(self.diagnostic_period), self._report_health)

        rospy.loginfo("Multi-camera observation receiver started")
        for name, topic in zip(self.camera_names, image_topics):
            rospy.loginfo("Camera %s image topic: %s", name, topic)
        rospy.loginfo("State topic: %s", state_topic)
        rospy.loginfo("Gripper topic: %s", gripper_topic)
        rospy.loginfo("Output topic: %s", output_topic)

    def _input_callback(self, message, label):
        with self.health_lock:
            entry = self.input_health[label]
            entry.update(received_at=time.monotonic(), stamp=message.header.stamp.to_sec(),
                         count=entry["count"] + 1)

    def _report_health(self, _event):
        now, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        with self.health_lock:
            elapsed = max(now - self.last_diagnostic_at, 1e-6)
            inputs = {}
            for name, entry in self.input_health.items():
                inputs[name] = {
                    "topic": entry["topic"], "received_messages": entry["count"],
                    "rate_hz": round((entry["count"] - self.previous_counts[name]) / elapsed, 2),
                    "receive_age_seconds": None if entry["received_at"] is None else round(now - entry["received_at"], 3),
                    "stamp_age_seconds": None if entry["stamp"] is None else round(ros_now - entry["stamp"], 3),
                }
                self.previous_counts[name] = entry["count"]
            output_age = None if self.last_publication_at is None else now - self.last_publication_at
            output_rate = (self.publication_count - self.previous_publication_count) / elapsed
            self.previous_publication_count = self.publication_count
            self.last_diagnostic_at = now
        missing_or_stale = [name for name, entry in inputs.items()
                            if entry["receive_age_seconds"] is None
                            or entry["receive_age_seconds"] > self.max_input_age
                            or abs(entry["stamp_age_seconds"]) > self.max_input_age]
        summary = {"inputs": inputs, "output_topic": self.output_topic,
                   "output_rate_hz": round(output_rate, 2),
                   "output_receive_age_seconds": None if output_age is None else round(output_age, 3),
                   "missing_or_stale_inputs": missing_or_stale}
        if missing_or_stale:
            rospy.logwarn("Observation inputs missing/stale: %s", json.dumps(summary))
        elif output_age is None or output_age > self.max_input_age:
            rospy.logwarn("Inputs arriving but synchronized output missing/stale; check stamp skew and sync queues: %s",
                          json.dumps(summary))
        else:
            rospy.loginfo("Observation stream health: %s", json.dumps(summary))

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
        with self.health_lock:
            self.last_publication_at = time.monotonic()
            self.publication_count += 1

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
