#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix


class ArucoPosePublisher:

    def __init__(self):
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.distortion_coefficients = None
        self.camera_frame = None
        self.camera_info_ready = False

        self.marker_id = rospy.get_param(
            "~marker_id",
            0
        )

        # This must equal the measured black-square side length.
        self.marker_size = rospy.get_param(
            "~marker_size",
            0.100
        )
        if (isinstance(self.marker_size, bool)
                or not np.isfinite(self.marker_size)
                or self.marker_size <= 0.0):
            raise ValueError("~marker_size must be a positive finite number")

        self.dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_50
        )

        self.detector_parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        # SUBPIX is a poor fit for this printed marker: it can move the outer
        # square corners enough to select the other planar-pose branch. Contour
        # refinement was verified against the rigid hand-to-marker constraint.
        refinement_name = str(rospy.get_param(
            "~corner_refinement", "contour")).strip().lower()
        refinements = {
            "none": cv2.aruco.CORNER_REFINE_NONE,
            "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
        }
        if hasattr(cv2.aruco, "CORNER_REFINE_CONTOUR"):
            refinements["contour"] = cv2.aruco.CORNER_REFINE_CONTOUR
        if refinement_name not in refinements:
            raise ValueError("~corner_refinement must be one of {}".format(
                ", ".join(sorted(refinements))))
        self.detector_parameters.cornerRefinementMethod = refinements[
            refinement_name]

        half_size = 0.5 * float(self.marker_size)
        # Required point ordering for SOLVEPNP_IPPE_SQUARE and ArUco corners:
        # top-left, top-right, bottom-right, bottom-left.
        self.marker_object_points = np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float64).reshape(4, 1, 3)
        self.pnp_method = getattr(
            cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster()
        )

        self.pose_publisher = rospy.Publisher(
            "/aruco_marker/pose",
            PoseStamped,
            queue_size=10
        )

        self.camera_info_subscriber = rospy.Subscriber(
            "/camera/color/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.image_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.window_name = "ArUco Pose Estimation"

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL
        )

        rospy.on_shutdown(self.shutdown_callback)

        rospy.loginfo(
            "ArUco pose publisher initialized: "
            "dictionary=DICT_4X4_50, ID=%d, size=%.4f m, "
            "corner_refinement=%s, pnp_method=%d",
            self.marker_id,
            self.marker_size,
            refinement_name,
            self.pnp_method
        )

    def camera_info_callback(self, camera_info_msg):
        self.camera_matrix = np.array(
            camera_info_msg.K,
            dtype=np.float64
        ).reshape(3, 3)

        self.distortion_coefficients = np.array(
            camera_info_msg.D,
            dtype=np.float64
        )

        self.camera_frame = (
            camera_info_msg.header.frame_id
        )

        self.camera_info_ready = True

        rospy.loginfo_once(
            "Camera calibration received. Frame: %s",
            self.camera_frame
        )

    def image_callback(self, image_msg):
        if not self.camera_info_ready:
            rospy.logwarn_throttle(
                1.0,
                "Waiting for camera calibration."
            )
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8"
            )

        except CvBridgeError as error:
            rospy.logerr("CV Bridge error: %s", error)
            return

        gray_image = cv2.cvtColor(
            color_image,
            cv2.COLOR_BGR2GRAY
        )

        corners, marker_ids, rejected = (
            cv2.aruco.detectMarkers(
                gray_image,
                self.dictionary,
                parameters=self.detector_parameters
            )
        )

        display_image = color_image.copy()

        marker_found = False

        if marker_ids is not None:
            marker_ids_flat = marker_ids.flatten()

            cv2.aruco.drawDetectedMarkers(
                display_image,
                corners,
                marker_ids
            )

            matching_indices = np.where(
                marker_ids_flat == self.marker_id
            )[0]

            if matching_indices.size > 0:
                marker_found = True
                marker_index = int(matching_indices[0])

                image_points = np.ascontiguousarray(
                    corners[marker_index], dtype=np.float64).reshape(4, 1, 2)
                success, rotation_vector, translation_vector = cv2.solvePnP(
                    self.marker_object_points,
                    image_points,
                    self.camera_matrix,
                    self.distortion_coefficients,
                    flags=self.pnp_method
                )
                if not success:
                    rospy.logwarn_throttle(
                        1.0, "PnP failed for marker ID %d", self.marker_id)
                    return
                rotation_vector = rotation_vector.reshape(3)
                translation_vector = translation_vector.reshape(3)

                rotation_matrix, _ = cv2.Rodrigues(
                    rotation_vector
                )

                homogeneous_rotation = np.eye(
                    4,
                    dtype=np.float64
                )

                homogeneous_rotation[:3, :3] = (
                    rotation_matrix
                )

                quaternion = quaternion_from_matrix(
                    homogeneous_rotation
                )

                self.publish_pose(
                    image_msg.header.stamp,
                    translation_vector,
                    quaternion
                )

                self.publish_transform(
                    image_msg.header.stamp,
                    translation_vector,
                    quaternion
                )

                self.draw_coordinate_axes(
                    display_image,
                    rotation_vector,
                    translation_vector
                )

                x, y, z = translation_vector

                rospy.loginfo_throttle(
                    1.0,
                    (
                        "Marker %d position in %s: "
                        "[%.3f, %.3f, %.3f] m"
                    ),
                    self.marker_id,
                    self.camera_frame,
                    x,
                    y,
                    z
                )

                position_text = (
                    f"Marker {self.marker_id}: "
                    f"[{x:.3f}, {y:.3f}, {z:.3f}] m"
                )

                cv2.putText(
                    display_image,
                    position_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        if not marker_found:
            rospy.logwarn_throttle(
                1.0,
                "Marker ID %d is not visible.",
                self.marker_id
            )

            cv2.putText(
                display_image,
                f"Marker {self.marker_id} not detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

        cv2.imshow(
            self.window_name,
            display_image
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rospy.signal_shutdown(
                "User requested shutdown."
            )

    def publish_pose(
        self,
        stamp,
        translation_vector,
        quaternion
    ):
        pose_msg = PoseStamped()

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = translation_vector[0]
        pose_msg.pose.position.y = translation_vector[1]
        pose_msg.pose.position.z = translation_vector[2]

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_msg)

    def publish_transform(
        self,
        stamp,
        translation_vector,
        quaternion
    ):
        transform_msg = TransformStamped()

        transform_msg.header.stamp = stamp
        transform_msg.header.frame_id = (
            self.camera_frame
        )

        transform_msg.child_frame_id = (
            "aruco_marker_frame"
        )

        transform_msg.transform.translation.x = (
            translation_vector[0]
        )

        transform_msg.transform.translation.y = (
            translation_vector[1]
        )

        transform_msg.transform.translation.z = (
            translation_vector[2]
        )

        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(
            transform_msg
        )

    def draw_coordinate_axes(
        self,
        image,
        rotation_vector,
        translation_vector
    ):
        axis_length = self.marker_size * 0.5

        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(
                image,
                self.camera_matrix,
                self.distortion_coefficients,
                rotation_vector,
                translation_vector,
                axis_length
            )

        elif hasattr(cv2.aruco, "drawAxis"):
            cv2.aruco.drawAxis(
                image,
                self.camera_matrix,
                self.distortion_coefficients,
                rotation_vector,
                translation_vector,
                axis_length
            )

    def shutdown_callback(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("aruco_pose_publisher")

    try:
        node = ArucoPosePublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
