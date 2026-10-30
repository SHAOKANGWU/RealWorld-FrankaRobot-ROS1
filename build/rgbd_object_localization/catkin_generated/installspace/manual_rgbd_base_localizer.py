#!/usr/bin/env python3

"""Select an RGB pixel and publish its 3D position in camera and base frames."""

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image


class ManualRgbdBaseLocalizer:

    def __init__(self):
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_model_ready = False
        self.camera_frame = None
        self.selected_pixel = None
        self.latest_rgbd = None

        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.depth_window_radius = int(rospy.get_param(
            "~depth_window_radius", 3))
        self.transform_timeout = float(rospy.get_param(
            "~transform_timeout", 0.2))
        sync_queue_size = int(rospy.get_param("~sync_queue_size", 10))
        sync_slop = float(rospy.get_param("~sync_slop", 0.05))

        if not np.isfinite(self.depth_scale) or self.depth_scale <= 0.0:
            raise ValueError("~depth_scale must be a positive finite number")
        if self.depth_window_radius < 0:
            raise ValueError("~depth_window_radius must be non-negative")
        if self.transform_timeout < 0.0:
            raise ValueError("~transform_timeout must be non-negative")
        if sync_queue_size <= 0:
            raise ValueError("~sync_queue_size must be positive")
        if sync_slop < 0.0:
            raise ValueError("~sync_slop must be non-negative")

        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info")
        color_topic = rospy.get_param(
            "~color_topic", "/camera/color/image_raw")
        depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        camera_point_topic = rospy.get_param(
            "~camera_point_topic", "/object_position/camera")
        base_point_topic = rospy.get_param(
            "~base_point_topic", "/object_position/base")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_point_publisher = rospy.Publisher(
            camera_point_topic, PointStamped, queue_size=10)
        self.base_point_publisher = rospy.Publisher(
            base_point_topic, PointStamped, queue_size=10)

        self.camera_info_subscriber = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )
        self.color_subscriber = message_filters.Subscriber(
            color_topic, Image, queue_size=1, buff_size=2**24)
        self.depth_subscriber = message_filters.Subscriber(
            depth_topic, Image, queue_size=1, buff_size=2**24)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=sync_queue_size,
            slop=sync_slop
        )
        self.synchronizer.registerCallback(self.rgbd_callback)

        self.window_name = "RGB-D Object Selection in Robot Base"
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        except cv2.error as error:
            raise RuntimeError(
                "Cannot create the OpenCV window. Run the node in a desktop "
                "session, or enable X11 forwarding and set DISPLAY. OpenCV "
                "reported: %s" % error
            )
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        rospy.on_shutdown(self.shutdown_callback)

        rospy.loginfo(
            "Manual RGB-D base localizer initialized: base_frame=%s, "
            "camera_topic=%s, depth_topic=%s",
            self.base_frame,
            color_topic,
            depth_topic
        )
        rospy.loginfo("Left-click an object to publish its 3D position.")
        rospy.loginfo("Press 'c' to clear the selection or 'q' to stop.")

    def camera_info_callback(self, camera_info_msg):
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.camera_frame = camera_info_msg.header.frame_id
        self.camera_model_ready = True
        rospy.loginfo_once(
            "Camera model initialized. Optical frame: %s",
            self.camera_frame
        )

    def mouse_callback(self, event, x, y, flags, parameter):
        del flags, parameter
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_pixel = (x, y)
            rospy.loginfo("Selected pixel: u=%d, v=%d", x, y)

    def depth_at_pixel(self, depth_image, depth_encoding, u, v):
        height, width = depth_image.shape[:2]
        if not (0 <= u < width and 0 <= v < height):
            rospy.logwarn_throttle(
                1.0, "Selected pixel is outside the depth image.")
            return None

        radius = self.depth_window_radius
        region = depth_image[
            max(0, v - radius):min(height, v + radius + 1),
            max(0, u - radius):min(width, u + radius + 1)
        ]
        valid_depths = region[np.isfinite(region) & (region > 0)]
        if valid_depths.size == 0:
            rospy.logwarn_throttle(
                1.0, "No valid depth around pixel (%d, %d).", u, v)
            return None

        depth_raw = float(np.median(valid_depths))
        if depth_encoding in ("16UC1", "mono16"):
            depth_metres = depth_raw * self.depth_scale
        elif depth_encoding == "32FC1":
            depth_metres = depth_raw
        else:
            rospy.logerr_throttle(
                1.0, "Unsupported depth encoding: %s", depth_encoding)
            return None

        if not np.isfinite(depth_metres) or depth_metres <= 0.0:
            rospy.logwarn_throttle(
                1.0, "Invalid depth at pixel (%d, %d).", u, v)
            return None

        return depth_raw, depth_metres

    def make_camera_point(self, depth_msg, u, v, depth_metres):
        rectified_pixel = self.camera_model.rectifyPoint(
            (float(u), float(v)))
        ray = self.camera_model.projectPixelTo3dRay(rectified_pixel)
        if abs(ray[2]) < 1e-9:
            rospy.logerr_throttle(
                1.0, "Projection ray has a zero Z component.")
            return None

        scale = depth_metres / ray[2]
        point = PointStamped()
        point.header.stamp = depth_msg.header.stamp
        point.header.frame_id = self.camera_frame
        point.point.x = ray[0] * scale
        point.point.y = ray[1] * scale
        point.point.z = ray[2] * scale
        return point

    def transform_to_base(self, point_camera):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                point_camera.header.frame_id,
                point_camera.header.stamp,
                rospy.Duration(self.transform_timeout)
            )
            return tf2_geometry_msgs.do_transform_point(
                point_camera, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as error:
            rospy.logwarn_throttle(
                1.0,
                "Cannot transform selected point from %s to %s: %s",
                point_camera.header.frame_id,
                self.base_frame,
                error
            )
            return None

    def estimate_selected_point(self, depth_image, depth_msg, u, v):
        depth = self.depth_at_pixel(
            depth_image, depth_msg.encoding, u, v)
        if depth is None:
            return None, None, None

        depth_raw, depth_metres = depth
        point_camera = self.make_camera_point(
            depth_msg, u, v, depth_metres)
        if point_camera is None:
            return None, None, depth_raw

        self.camera_point_publisher.publish(point_camera)
        point_base = self.transform_to_base(point_camera)
        if point_base is not None:
            self.base_point_publisher.publish(point_base)
            rospy.loginfo_throttle(
                1.0,
                "Pixel=(%d, %d), depth=%.4f m, %s=[%.4f, %.4f, %.4f] m",
                u,
                v,
                depth_metres,
                self.base_frame,
                point_base.point.x,
                point_base.point.y,
                point_base.point.z
            )

        return point_camera, point_base, depth_raw

    @staticmethod
    def draw_point_text(image, label, point, row, color):
        text = "%s: [%.3f, %.3f, %.3f] m" % (
            label,
            point.point.x,
            point.point.y,
            point.point.z
        )
        cv2.putText(
            image,
            text,
            (10, row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2
        )

    def rgbd_callback(self, color_msg, depth_msg):
        """Convert and cache a synchronized pair; the main thread draws it."""
        try:
            color_image = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
        except CvBridgeError as error:
            rospy.logerr("CV Bridge error: %s", error)
            return

        color_height, color_width = color_image.shape[:2]
        depth_height, depth_width = depth_image.shape[:2]
        if color_width != depth_width or color_height != depth_height:
            rospy.logerr_throttle(
                1.0,
                "Image size mismatch: RGB=%dx%d, depth=%dx%d",
                color_width,
                color_height,
                depth_width,
                depth_height
            )
            return

        # Replacing one tuple is atomic in CPython, so the GUI always obtains a
        # mutually consistent color image, depth image, and depth message.
        self.latest_rgbd = (color_image, depth_image, depth_msg)

    def draw_waiting_screen(self):
        display_image = np.zeros((360, 800, 3), dtype=np.uint8)
        lines = [
            "Waiting for synchronized RGB and aligned-depth images...",
            "RGB: %s" % self.color_topic,
            "Depth: %s" % self.depth_topic,
            "The RealSense driver must be started with align_depth:=true",
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                display_image,
                line,
                (20, 55 + index * 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255) if index == 0 else (220, 220, 220),
                2
            )
        return display_image

    def render_gui(self):
        """Draw the current image and process mouse/keyboard events."""
        rgbd = self.latest_rgbd
        if rgbd is None:
            display_image = self.draw_waiting_screen()
            cv2.imshow(self.window_name, display_image)
            self.process_key(cv2.waitKey(1) & 0xFF)
            return

        color_image, depth_image, depth_msg = rgbd
        display_image = color_image.copy()
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
            cv2.drawMarker(
                display_image,
                (u, v),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2
            )
            if not self.camera_model_ready:
                cv2.putText(
                    display_image,
                    "Waiting for color camera_info",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 165, 255),
                    2
                )
            else:
                point_camera, point_base, _ = self.estimate_selected_point(
                    depth_image, depth_msg, u, v)
                if point_camera is None:
                    cv2.putText(
                        display_image,
                        "No valid depth at selected pixel",
                        (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2
                    )
                else:
                    self.draw_point_text(
                        display_image,
                        "Camera", point_camera, 55, (0, 255, 0)
                    )
                    if point_base is not None:
                        self.draw_point_text(
                            display_image,
                            self.base_frame,
                            point_base,
                            82,
                            (255, 255, 0)
                        )
                    else:
                        cv2.putText(
                            display_image,
                            "Waiting for camera-to-base TF",
                            (10, 82),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 165, 255),
                            2
                        )

        cv2.imshow(self.window_name, display_image)
        self.process_key(cv2.waitKey(1) & 0xFF)

    def process_key(self, key):
        if key == ord("c"):
            self.selected_pixel = None
            rospy.loginfo("Pixel selection cleared.")
        elif key == ord("q"):
            rospy.signal_shutdown("User requested shutdown.")

    def run(self):
        """Keep OpenCV GUI work on the main thread, even without images."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.render_gui()
            rate.sleep()

    @staticmethod
    def shutdown_callback():
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("manual_rgbd_base_localizer")
    try:
        node = ManualRgbdBaseLocalizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
