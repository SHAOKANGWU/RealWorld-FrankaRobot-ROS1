#!/usr/bin/env python3

import os
import threading
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState

from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

import numpy as np
from tf.transformations import quaternion_from_matrix

import json

import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError


class DemoRecorder:

    def __init__(self):

        # ------------------------------------------
        # Parameters
        # ------------------------------------------

        self.output_root = rospy.get_param(
            "~output_root",
            os.path.expanduser(
                "~/ros_franka_robotlearn/data/"
                "kinesthetic"
            )
        )

        # ------------------------------------------
        # Recording state
        # ------------------------------------------

        self.recording = False
        self.lock = threading.Lock()

        # Prevent stop_recording() from saving the episode
        # while an image callback is still writing a frame.
        self.image_processing_lock = threading.Lock()
        self.bridge = CvBridge()

        self.jpeg_quality = rospy.get_param(
            "~jpeg_quality",
            95
        )
        self.franka_state_topic = rospy.get_param(
            "~franka_state_topic", "/franka_state_controller/franka_states"
        )
        self.gripper_joint_states_topic = rospy.get_param(
            "~gripper_joint_states_topic", "/franka_gripper/joint_states"
        )
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/color/image_raw"
        )

        self.current_episode_dir = None
        self.current_image_dir = None

        self.recording_start_time = None
        self.recording_stop_time = None

        os.makedirs(
            self.output_root,
            exist_ok=True
        )

        # ------------------------------------------
        # Robot trajectory buffers
        # ------------------------------------------
        self.robot_time = []
        self.q = []
        self.dq = []
        self.ee_position = []
        self.ee_quaternion = []
        self.O_T_EE = []

        # ------------------------------------------
        # Gripper buffers
        # ------------------------------------------
        self.gripper_time = []
        self.gripper_joint_position = []
        self.gripper_width = []

        # ------------------------------------------
        # Image buffers
        # ------------------------------------------
        self.image_time = []
        self.image_paths = []
        self.image_index = 0

        # ------------------------------------------
        # Subscribers
        # ------------------------------------------

        self.franka_state_sub = rospy.Subscriber(
            self.franka_state_topic,
            FrankaState,
            self.franka_state_callback,
            queue_size=10
        )

        self.gripper_sub = rospy.Subscriber(
            self.gripper_joint_states_topic,
            JointState,
            self.gripper_callback,
            queue_size=10
        )

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        # ------------------------------------------
        # Services
        # ------------------------------------------

        self.start_service = rospy.Service(
            "/demo_recorder/start",
            Trigger,
            self.start_recording
        )

        self.stop_service = rospy.Service(
            "/demo_recorder/stop",
            Trigger,
            self.stop_recording
        )

        rospy.loginfo(
            "Kinesthetic demonstration recorder ready."
        )

    def reset_buffers(self):

        self.robot_time = []
        self.q = []
        self.dq = []
        self.ee_position = []
        self.ee_quaternion = []
        self.O_T_EE = []

        self.gripper_time = []
        self.gripper_joint_position = []
        self.gripper_width = []

        self.image_time = []
        self.image_paths = []
        self.image_index = 0

        
    def get_next_demo_index(self):

        index = 1
        while True:
            episode_name = (
                f"demo_{index:04d}"
            )
            episode_dir = os.path.join(
                self.output_root,
                episode_name
            )
            if not os.path.exists(
                episode_dir
            ):
                return index

            index += 1

            
    def franka_state_callback(self, msg):

        if not self.recording:
            return

        # ------------------------------------------
        # Timestamp
        # ------------------------------------------

        if msg.header.stamp.to_sec() > 0.0:
            timestamp = msg.header.stamp.to_sec()
        else:
            timestamp = rospy.Time.now().to_sec()

        # ------------------------------------------
        # Joint state
        # ------------------------------------------

        q = np.asarray(
            msg.q,
            dtype=np.float64
        )

        dq = np.asarray(
            msg.dq,
            dtype=np.float64
        )

        # ------------------------------------------
        # End-effector transform
        # Franka uses column-major storage.
        # ------------------------------------------

        O_T_EE = np.asarray(
            msg.O_T_EE,
            dtype=np.float64
        )

        T = O_T_EE.reshape(
            (4, 4),
            order="F"
        )

        position = T[:3, 3]

        # Returns [x, y, z, w]
        quaternion_xyzw = (
            quaternion_from_matrix(T)
        )

        # Convert to the same convention
        # you use in your Eigen C++ code:
        # [w, x, y, z]
        quaternion_wxyz = np.array(
            [
                quaternion_xyzw[3],
                quaternion_xyzw[0],
                quaternion_xyzw[1],
                quaternion_xyzw[2],
            ],
            dtype=np.float64
        )

        # ------------------------------------------
        # Append to buffers
        # ------------------------------------------

        with self.lock:

            # q and -q describe the same orientation. Keep each sample in the
            # same quaternion hemisphere as the preceding stored sample so
            # interpolation and finite differences do not see false jumps.
            if (
                self.ee_quaternion
                and np.dot(
                    self.ee_quaternion[-1],
                    quaternion_wxyz
                ) < 0.0
            ):

                quaternion_wxyz = (
                    -quaternion_wxyz
                )

            self.robot_time.append(
                timestamp
            )

            self.q.append(
                q.copy()
            )

            self.dq.append(
                dq.copy()
            )

            self.ee_position.append(
                position.copy()
            )

            self.ee_quaternion.append(
                quaternion_wxyz.copy()
            )

            self.O_T_EE.append(
                O_T_EE.copy()
            )


    def gripper_callback(self, msg):

        if not self.recording:
            return

        if msg.header.stamp.to_sec() > 0.0:
            timestamp = msg.header.stamp.to_sec()
        else:
            timestamp = rospy.Time.now().to_sec()

        joint_position = np.asarray(
            msg.position,
            dtype=np.float64
        )

        if len(joint_position) >= 2:

            width = (
                joint_position[0]
                + joint_position[1]
            )

        elif len(joint_position) == 1:

            width = joint_position[0]

        else:

            return

        with self.lock:

            self.gripper_time.append(
                timestamp
            )

            self.gripper_joint_position.append(
                joint_position.copy()
            )

            self.gripper_width.append(
                width
            )


    def image_callback(self, msg):

        # Only one image callback may be processed
        # at a time.
        with self.image_processing_lock:

            # --------------------------------------
            # Check recording state
            # --------------------------------------
            with self.lock:
                if not self.recording:
                    return

                image_index = self.image_index
                self.image_index += 1
                image_dir = (
                    self.current_image_dir
                )

            # --------------------------------------
            # Timestamp
            # --------------------------------------

            if msg.header.stamp.to_sec() > 0.0:
                timestamp = (
                    msg.header.stamp.to_sec()
                )
            else:
                timestamp = (
                    rospy.Time.now().to_sec()
                )

            # --------------------------------------
            # ROS Image -> OpenCV
            # --------------------------------------
            try:

                image = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8"
                )

            except CvBridgeError as error:

                rospy.logerr(
                    "Could not convert RGB image: %s",
                    str(error)
                )

                return

            # --------------------------------------
            # Image filename
            # --------------------------------------
            image_filename = (
                f"{image_index:06d}.jpg"
            )

            image_path = os.path.join(
                image_dir,
                image_filename
            )

            # --------------------------------------
            # Save image
            # --------------------------------------
            success = cv2.imwrite(
                image_path,
                image,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    self.jpeg_quality
                ]
            )

            if not success:
                rospy.logerr(
                    "Could not save image: %s",
                    image_path
                )
                return

            # --------------------------------------
            # Record timestamp + relative path
            # --------------------------------------

            relative_path = os.path.join(
                "images",
                image_filename
            )

            with self.lock:
                self.image_time.append(
                    timestamp
                )
                self.image_paths.append(
                    relative_path
                )


    def start_recording(self, request):

        with self.lock:

            if self.recording:

                return TriggerResponse(
                    success=False,
                    message=(
                        "A demonstration is "
                        "already being recorded."
                    )
                )

            # --------------------------------------
            # Determine episode number
            # --------------------------------------

            demo_index = (
                self.get_next_demo_index()
            )

            episode_name = (
                f"demo_{demo_index:04d}"
            )

            self.current_episode_dir = (
                os.path.join(
                    self.output_root,
                    episode_name
                )
            )

            self.current_image_dir = (
                os.path.join(
                    self.current_episode_dir,
                    "images"
                )
            )

            # --------------------------------------
            # Create directories
            # --------------------------------------

            os.makedirs(
                self.current_image_dir,
                exist_ok=False
            )

            # --------------------------------------
            # Clear previous demonstration
            # --------------------------------------

            self.reset_buffers()

            # --------------------------------------
            # Store start time
            # --------------------------------------

            self.recording_start_time = (
                rospy.Time.now().to_sec()
            )

            self.recording_stop_time = None

            # --------------------------------------
            # Enable callbacks
            # --------------------------------------

            self.recording = True


        rospy.loginfo(
            "Started recording %s",
            episode_name
        )

        return TriggerResponse(
            success=True,
            message=(
                f"Recording started: "
                f"{episode_name}"
            )
        )


    def stop_recording(self, request):

        # ------------------------------------------
        # Disable all new robot/gripper callbacks
        # ------------------------------------------

        with self.lock:

            if not self.recording:
                return TriggerResponse(
                    success=False,
                    message=(
                        "No demonstration is "
                        "currently being recorded."
                    )
                )

            self.recording = False

            self.recording_stop_time = (
                rospy.Time.now().to_sec()
            )

        # ------------------------------------------
        # Wait for any image currently being saved
        # ------------------------------------------
        with self.image_processing_lock:
            pass

        # ------------------------------------------
        # Save episode
        # ------------------------------------------

        try:
            self.save_episode()

        except Exception as error:

            rospy.logerr(
                "Failed to save demonstration: %s",
                str(error)
            )

            return TriggerResponse(
                success=False,
                message=(
                    "Recording stopped, but "
                    "saving failed: "
                    + str(error)
                )
            )

        episode_name = os.path.basename(
            self.current_episode_dir
        )

        rospy.loginfo(
            "Saved demonstration %s",
            episode_name
        )

        return TriggerResponse(
            success=True,
            message=(
                f"Saved {episode_name}"
            )
        )


    def save_episode(self):

        # ------------------------------------------
        # Copy buffers while holding the lock
        # ------------------------------------------

        with self.lock:

            robot_time = np.asarray(
                self.robot_time,
                dtype=np.float64
            )

            q = np.asarray(
                self.q,
                dtype=np.float64
            )

            dq = np.asarray(
                self.dq,
                dtype=np.float64
            )

            ee_position = np.asarray(
                self.ee_position,
                dtype=np.float64
            )

            ee_quaternion = np.asarray(
                self.ee_quaternion,
                dtype=np.float64
            )

            O_T_EE = np.asarray(
                self.O_T_EE,
                dtype=np.float64
            )

            gripper_time = np.asarray(
                self.gripper_time,
                dtype=np.float64
            )

            gripper_joint_position = np.asarray(
                self.gripper_joint_position,
                dtype=np.float64
            )

            gripper_width = np.asarray(
                self.gripper_width,
                dtype=np.float64
            )

            image_time = np.asarray(
                self.image_time,
                dtype=np.float64
            )

            image_paths = np.asarray(
                self.image_paths
            )

            start_time = (
                self.recording_start_time
            )

            stop_time = (
                self.recording_stop_time
            )

            episode_dir = (
                self.current_episode_dir
            )

        # ------------------------------------------
        # Relative timestamps
        # ------------------------------------------

        robot_time_relative = (
            robot_time - start_time
        )

        gripper_time_relative = (
            gripper_time - start_time
        )

        image_time_relative = (
            image_time - start_time
        )

        # ------------------------------------------
        # Save NumPy trajectory
        # ------------------------------------------

        trajectory_path = os.path.join(
            episode_dir,
            "trajectory.npz"
        )

        np.savez_compressed(

            trajectory_path,

            # Absolute ROS timestamps
            robot_time=robot_time,
            gripper_time=gripper_time,
            image_time=image_time,

            # Relative to /start
            robot_time_relative=
                robot_time_relative,

            gripper_time_relative=
                gripper_time_relative,

            image_time_relative=
                image_time_relative,

            # Robot state
            q=q,
            dq=dq,

            ee_position=
                ee_position,

            ee_quaternion_wxyz=
                ee_quaternion,

            O_T_EE=
                O_T_EE,

            # Gripper
            gripper_joint_position=
                gripper_joint_position,

            gripper_width=
                gripper_width,

            # RGB
            image_paths=
                image_paths,
        )

        # ------------------------------------------
        # Metadata
        # ------------------------------------------

        metadata = {

            "recording_start_time":
                start_time,

            "recording_stop_time":
                stop_time,

            "duration":
                stop_time - start_time,

            "robot_samples":
                int(len(robot_time)),

            "gripper_samples":
                int(len(gripper_time)),

            "image_samples":
                int(len(image_time)),

            "topics": {

                "robot_state":
                    self.franka_state_topic,

                "gripper_state":
                    self.gripper_joint_states_topic,

                "rgb":
                    self.image_topic,
            },

            "joint_order": [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],

            "ee_quaternion_order":
                "wxyz",

            "O_T_EE_storage":
                "Franka column-major 16-element array",

            "gripper_width_definition":
                "sum of the two finger joint positions",
        }

        metadata_path = os.path.join(
            episode_dir,
            "metadata.json"
        )

        with open(
            metadata_path,
            "w"
        ) as file:

            json.dump(
                metadata,
                file,
                indent=2
            )


def main():

    rospy.init_node(
        "demo_recorder"
    )

    DemoRecorder()

    rospy.spin()


if __name__ == "__main__":
    main()
