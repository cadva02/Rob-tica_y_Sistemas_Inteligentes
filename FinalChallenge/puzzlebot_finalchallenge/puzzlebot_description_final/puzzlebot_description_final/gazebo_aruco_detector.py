import math
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


class GazeboArucoDetector(Node):
    """Detect ArUco markers from a Gazebo camera and publish as MarkerArray for EKF."""

    def __init__(self) -> None:
        super().__init__('gazebo_aruco_detector')

        self.declare_parameter('image_topic', '/camera')
        self.declare_parameter('image_topic_alt', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_info_topic_alt', '/camera/camera_info')
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('annotated_image_topic', '/aruco/image_annotated')
        self.declare_parameter('marker_size_m', 0.14)
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        image_topic = str(self.get_parameter('image_topic').value)
        image_topic_alt = str(self.get_parameter('image_topic_alt').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        camera_info_topic_alt = str(self.get_parameter('camera_info_topic_alt').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.annotated_image_topic = str(self.get_parameter('annotated_image_topic').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.last_dictionary_name = None

        self.lock = threading.Lock()

        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.info_cb, 10)
        self.info_sub_alt = self.create_subscription(CameraInfo, camera_info_topic_alt, self.info_cb, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_cb, 5)
        self.image_sub_alt = self.create_subscription(Image, image_topic_alt, self.image_cb, 5)
        self.pub = self.create_publisher(MarkerArray, self.aruco_topic, 10)
        self.image_pub = self.create_publisher(Image, self.annotated_image_topic, 10)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.dictionary_names = [
            str(self.get_parameter('dictionary').value),
            'DICT_4X4_50',
            'DICT_5X5_50',
            'DICT_6X6_50',
            'DICT_4X4_100',
        ]
        self.aruco_dicts = []
        for dictionary_name in self.dictionary_names:
            dictionary_id = getattr(cv2.aruco, dictionary_name, None)
            if dictionary_id is None:
                continue
            if hasattr(cv2.aruco, 'getPredefinedDictionary'):
                self.aruco_dicts.append((dictionary_name, cv2.aruco.getPredefinedDictionary(dictionary_id)))
            else:
                self.aruco_dicts.append((dictionary_name, cv2.aruco.Dictionary_get(dictionary_id)))

        self.get_logger().info(f'Gazebo ArUco detector started | image={image_topic}, info={camera_info_topic}, publish={self.aruco_topic}')

    def info_cb(self, msg: CameraInfo) -> None:
        with self.lock:
            if self.camera_matrix is None:
                self.camera_matrix = np.array(msg.k, dtype=float).reshape((3, 3))
                self.dist_coeffs = np.array(msg.d, dtype=float) if msg.d else np.zeros((5,))
                self.get_logger().info(
                    'Camera info received from %s, ready to detect markers'
                    % (msg.header.frame_id if msg.header.frame_id else 'camera_info')
                )

    def preview_timer_cb(self) -> None:
        return

    def image_cb(self, msg: Image) -> None:
        with self.lock:
            if self.camera_matrix is None:
                return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        annotated = cv_image.copy()
        marker_array = MarkerArray()
        corners = None
        ids = None
        detected_dictionary = None

        for dictionary_name, aruco_dict in self.aruco_dicts:
            test_corners, test_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=self.aruco_params)
            if test_ids is not None and len(test_ids) > 0:
                corners = test_corners
                ids = test_ids
                detected_dictionary = dictionary_name
                break

        if ids is None or len(ids) == 0:
            self.pub.publish(marker_array)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception:
                pass
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs,
        )

        if detected_dictionary and detected_dictionary != self.last_dictionary_name:
            self.last_dictionary_name = detected_dictionary
            self.get_logger().info(f'ArUco detected using {detected_dictionary}')

        ts = self.get_clock().now().to_msg()

        for idx, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[idx][0]
            tvec = tvecs[idx][0]

            R, _ = cv2.Rodrigues(rvec)
            qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
            qx = (R[2, 1] - R[1, 2]) / (4.0 * qw + 1e-12)
            qy = (R[0, 2] - R[2, 0]) / (4.0 * qw + 1e-12)
            qz = (R[1, 0] - R[0, 1]) / (4.0 * qw + 1e-12)

            m = Marker()
            m.header.stamp = ts
            m.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
            m.ns = 'aruco_gazebo'
            m.id = int(marker_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # OpenCV camera frame: x right, y down, z forward.
            # ROS camera frame: x forward, y left, z up.
            m.pose.position.x = float(tvec[2])
            m.pose.position.y = float(-tvec[0])
            m.pose.position.z = float(-tvec[1])
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            # Encode the estimated marker distance (meters) in the Marker.scale fields
            # so downstream nodes (EKF) can adjust measurement confidence dynamically.
            distance = float(np.linalg.norm(tvec))
            m.scale.x = distance
            m.scale.y = distance
            m.scale.z = distance
            m.color.a = 1.0
            m.color.r = 0.8
            m.color.g = 0.2
            m.color.b = 0.2

            try:
                self.get_logger().info(
                    f'ArUco raw tvec=({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}) -> '
                    f'camera pose=({m.pose.position.x:.2f}, {m.pose.position.y:.2f}, {m.pose.position.z:.2f})'
                )
            except Exception:
                pass

            marker_array.markers.append(m)

            # Strong visual overlay for the image viewer.
            pts = corners[idx].astype(int).reshape((-1, 2))
            cv2.polylines(annotated, [pts], True, (0, 255, 255), 4, cv2.LINE_AA)
            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))
            cv2.circle(annotated, (center_x, center_y), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                annotated,
                f'ID {int(marker_id)}',
                (center_x + 8, center_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                f'd={distance:.2f}m',
                (center_x + 8, center_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        try:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            for idx in range(len(ids)):
                cv2.drawFrameAxes(
                    annotated,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[idx],
                    tvecs[idx],
                    self.marker_size_m * 0.5,
                )
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        except Exception:
            pass

        self.pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
