#!/usr/bin/env python3

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ArucoDictionaryIdentifier:

    def __init__(self):
        self.bridge = CvBridge()
        self.window_name = "ArUco Dictionary Identification"

        self.detector_parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        dictionary_names = [
            "DICT_4X4_50",
            "DICT_4X4_100",
            "DICT_4X4_250",
            "DICT_4X4_1000",
            "DICT_ARUCO_ORIGINAL"
        ]

        self.dictionaries = {}

        for dictionary_name in dictionary_names:
            if hasattr(cv2.aruco, dictionary_name):
                dictionary_number = getattr(
                    cv2.aruco,
                    dictionary_name
                )

                self.dictionaries[dictionary_name] = (
                    cv2.aruco.Dictionary_get(
                        dictionary_number
                    )
                )

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL
        )

        self.image_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.on_shutdown(self.shutdown_callback)

        rospy.loginfo(
            "Testing the following dictionaries: %s",
            ", ".join(self.dictionaries.keys())
        )

    def image_callback(self, image_msg):
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

        display_image = color_image.copy()
        detection_results = []

        first_corners = None
        first_ids = None

        for dictionary_name, dictionary in self.dictionaries.items():
            corners, marker_ids, rejected = (
                cv2.aruco.detectMarkers(
                    gray_image,
                    dictionary,
                    parameters=self.detector_parameters
                )
            )

            if marker_ids is not None:
                detected_ids = marker_ids.flatten().tolist()

                detection_results.append(
                    f"{dictionary_name}: {detected_ids}"
                )

                if first_ids is None:
                    first_corners = corners
                    first_ids = marker_ids

        if first_ids is not None:
            cv2.aruco.drawDetectedMarkers(
                display_image,
                first_corners,
                first_ids
            )

            result_text = " | ".join(detection_results)

            rospy.loginfo_throttle(
                1.0,
                "Detected marker: %s",
                result_text
            )

            cv2.putText(
                display_image,
                result_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        else:
            rospy.logwarn_throttle(
                1.0,
                "No compatible ArUco marker detected."
            )

            cv2.putText(
                display_image,
                "No marker detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
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

    def shutdown_callback(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("identify_aruco_dictionary")

    try:
        identifier = ArucoDictionaryIdentifier()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass