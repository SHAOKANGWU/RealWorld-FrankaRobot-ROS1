#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo


class ManualRgbdLocalizer:

    def __init__(self):
        self.bridge = CvBridge()

        self.camera_model = PinholeCameraModel()
        self.camera_model_ready = False
        self.camera_frame = None

        # The currently selected RGB pixel.
        self.selected_pixel = None

        # RealSense 16UC1 depth is expressed in millimetres.
        self.depth_scale = rospy.get_param("~depth_scale", 0.001)

        self.window_name = "RGB-D Object Selection"

        # Create the graphical window and register the mouse callback.
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(
            self.window_name,
            self.mouse_callback
        )

        # Publish the selected point in the camera optical frame.
        self.point_publisher = rospy.Publisher(
            "/object_position/camera",
            PointStamped,
            queue_size=10
        )

        self.camera_info_subscriber = rospy.Subscriber(
            "/camera/color/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.color_subscriber = message_filters.Subscriber(
            "/camera/color/image_raw",
            Image
        )

        self.depth_subscriber = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.color_subscriber,
                self.depth_subscriber
            ],
            queue_size=10,
            slop=0.05
        )

        self.synchronizer.registerCallback(self.rgbd_callback)

        rospy.on_shutdown(self.shutdown_callback)

        rospy.loginfo("Manual RGB-D localizer initialized.")
        rospy.loginfo("Left-click an object to select it.")
        rospy.loginfo("Press 'c' to clear the selection.")
        rospy.loginfo("Press 'q' to stop the node.")

    def camera_info_callback(self, camera_info_msg):
        """
        Initialize the pinhole camera model.
        """

        self.camera_model.fromCameraInfo(camera_info_msg)
        self.camera_frame = camera_info_msg.header.frame_id
        self.camera_model_ready = True

        rospy.loginfo_once(
            "Camera model initialized. Optical frame: %s",
            self.camera_frame
        )

    def mouse_callback(self, event, x, y, flags, parameter):
        """
        Store the image pixel selected with the left mouse button.
        """

        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_pixel = (x, y)

            rospy.loginfo(
                "Selected pixel: u=%d, v=%d",
                x,
                y
            )

    def estimate_selected_point(
        self,
        depth_image,
        depth_msg,
        u,
        v
    ):
        """
        Estimate and publish the 3D position of the selected pixel.

        Returns:
            (x, y, z) in metres if successful, otherwise None.
        """

        depth_height, depth_width = depth_image.shape[:2]

        if not (
            0 <= u < depth_width
            and 0 <= v < depth_height
        ):
            rospy.logwarn_throttle(
                1.0,
                "Selected pixel is outside the depth image."
            )
            return None

        # Use a 7 x 7 region around the selected pixel.
        radius = 3

        u_min = max(0, u - radius)
        u_max = min(depth_width, u + radius + 1)
        v_min = max(0, v - radius)
        v_max = min(depth_height, v + radius + 1)

        depth_region = depth_image[
            v_min:v_max,
            u_min:u_max
        ]

        # Remove missing and invalid depth measurements.
        valid_depths = depth_region[
            np.isfinite(depth_region)
            & (depth_region > 0)
        ]

        if valid_depths.size == 0:
            rospy.logwarn_throttle(
                1.0,
                "No valid depth around pixel (%d, %d).",
                u,
                v
            )
            return None

        depth_raw = float(np.median(valid_depths))

        if depth_msg.encoding == "16UC1":
            depth_metres = depth_raw * self.depth_scale

        elif depth_msg.encoding == "32FC1":
            depth_metres = depth_raw

        else:
            rospy.logerr_throttle(
                1.0,
                "Unsupported depth encoding: %s",
                depth_msg.encoding
            )
            return None

        if (
            not np.isfinite(depth_metres)
            or depth_metres <= 0.0
        ):
            rospy.logwarn_throttle(
                1.0,
                "Invalid depth at pixel (%d, %d).",
                u,
                v
            )
            return None

        # Undistort the selected RGB pixel.
        rectified_pixel = self.camera_model.rectifyPoint(
            (float(u), float(v))
        )

        # Calculate its viewing ray.
        ray = self.camera_model.projectPixelTo3dRay(
            rectified_pixel
        )

        if abs(ray[2]) < 1e-9:
            rospy.logerr_throttle(
                1.0,
                "The projection ray has a zero Z component."
            )
            return None

        # Scale the ray using the measured optical-axis depth.
        scale = depth_metres / ray[2]

        x_camera = ray[0] * scale
        y_camera = ray[1] * scale
        z_camera = ray[2] * scale

        point_camera = PointStamped()

        point_camera.header.stamp = depth_msg.header.stamp
        point_camera.header.frame_id = self.camera_frame

        point_camera.point.x = x_camera
        point_camera.point.y = y_camera
        point_camera.point.z = z_camera

        self.point_publisher.publish(point_camera)

        rospy.loginfo_throttle(
            1.0,
            (
                "Pixel=(%d, %d), raw depth=%.1f, "
                "camera point=[%.3f, %.3f, %.3f] m"
            ),
            u,
            v,
            depth_raw,
            x_camera,
            y_camera,
            z_camera
        )

        return x_camera, y_camera, z_camera

    def rgbd_callback(self, color_msg, depth_msg):
        """
        Display the RGB image and process the selected pixel.
        """

        if not self.camera_model_ready:
            rospy.logwarn_throttle(
                1.0,
                "Waiting for camera calibration information."
            )
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding="bgr8"
            )

            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough"
            )

        except CvBridgeError as error:
            rospy.logerr("CV Bridge error: %s", error)
            return

        color_height, color_width = color_image.shape[:2]
        depth_height, depth_width = depth_image.shape[:2]

        if (
            color_width != depth_width
            or color_height != depth_height
        ):
            rospy.logerr_throttle(
                1.0,
                "Image size mismatch: RGB=%dx%d, depth=%dx%d",
                color_width,
                color_height,
                depth_width,
                depth_height
            )
            return

        display_image = color_image.copy()

        # Instructions displayed in the RGB window.
        cv2.putText(
            display_image,
            "Left-click: select | c: clear | q: quit",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        if self.selected_pixel is None:
            cv2.putText(
                display_image,
                "No pixel selected",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        else:
            u, v = self.selected_pixel

            # Draw a marker at the selected pixel.
            cv2.drawMarker(
                display_image,
                (u, v),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2
            )

            point = self.estimate_selected_point(
                depth_image,
                depth_msg,
                u,
                v
            )

            if point is not None:
                x_camera, y_camera, z_camera = point

                position_text = (
                    "Camera: "
                    f"[{x_camera:.3f}, "
                    f"{y_camera:.3f}, "
                    f"{z_camera:.3f}] m"
                )

                cv2.putText(
                    display_image,
                    position_text,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

            else:
                cv2.putText(
                    display_image,
                    "No valid depth at selected pixel",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

        cv2.imshow(self.window_name, display_image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("c"):
            self.selected_pixel = None
            rospy.loginfo("Pixel selection cleared.")

        elif key == ord("q"):
            rospy.signal_shutdown("User requested shutdown.")

    def shutdown_callback(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("manual_rgbd_localizer")

    try:
        localizer = ManualRgbdLocalizer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass