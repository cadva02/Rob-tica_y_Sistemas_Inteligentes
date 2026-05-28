import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray as VizMarkerArray
from visualization_msgs.msg import Marker as VizMarker


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(first_pose: np.ndarray, second_pose: np.ndarray) -> np.ndarray:
    first_theta = float(first_pose[2])
    cos_first = math.cos(first_theta)
    sin_first = math.sin(first_theta)

    x = float(first_pose[0]) + cos_first * float(second_pose[0]) - sin_first * float(second_pose[1])
    y = float(first_pose[1]) + sin_first * float(second_pose[0]) + cos_first * float(second_pose[1])
    theta = normalize_angle(first_theta + float(second_pose[2]))
    return np.array([x, y, theta], dtype=float)


def inverse_pose(pose: np.ndarray) -> np.ndarray:
    theta = float(pose[2])
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    x = -(cos_theta * float(pose[0]) + sin_theta * float(pose[1]))
    y = sin_theta * float(pose[0]) - cos_theta * float(pose[1])
    return np.array([x, y, normalize_angle(-theta)], dtype=float)


class SimulatedLandmarks(Node):
    """Publish fixed landmarks as simulated ArUco observations and RViz markers."""

    def __init__(self) -> None:
        super().__init__('simulated_landmarks')

        self.declare_parameter('pose_topic', 'pose_sim')
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('visualization_topic', '/sim_landmarks')
        self.declare_parameter('camera_base_x', 0.1241)
        self.declare_parameter('camera_base_y', 0.0)
        self.declare_parameter('camera_base_theta', 0.0)
        self.declare_parameter('max_detection_range', 0.5)
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter(
            'marker_map_json',
            json.dumps(
                [
                    {'id': 1, 'x': 4.2, 'y': 2.0, 'theta': 0.0},
                    {'id': 3, 'x': 0.00, 'y': 2.20, 'theta': math.pi / 2.0},
                ]
            ),
        )

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.visualization_topic = str(self.get_parameter('visualization_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.camera_base_pose = np.array(
            [
                float(self.get_parameter('camera_base_x').value),
                float(self.get_parameter('camera_base_y').value),
                float(self.get_parameter('camera_base_theta').value),
            ],
            dtype=float,
        )
        self.max_detection_range = float(self.get_parameter('max_detection_range').value)
        self.camera_fov_rad = math.radians(float(self.get_parameter('camera_fov_deg').value))

        marker_map_json = str(self.get_parameter('marker_map_json').value)
        self.marker_map = self._parse_marker_map(marker_map_json)
        self.latest_robot_pose: np.ndarray | None = None

        aruco_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        viz_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.aruco_pub = self.create_publisher(VizMarkerArray, self.aruco_topic, aruco_qos)
        self.viz_pub = self.create_publisher(VizMarkerArray, self.visualization_topic, viz_qos)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info(
            'Simulated landmarks started | pose_topic=%s, aruco_topic=%s, visualization_topic=%s, markers=%d, range=%.2fm, fov=%.1fdeg'
            % (
                self.pose_topic,
                self.aruco_topic,
                self.visualization_topic,
                len(self.marker_map),
                self.max_detection_range,
                math.degrees(self.camera_fov_rad),
            )
        )

    def _parse_marker_map(self, marker_map_json: str) -> dict[int, np.ndarray]:
        marker_map: dict[int, np.ndarray] = {}
        try:
            entries = json.loads(marker_map_json)
        except json.JSONDecodeError:
            self.get_logger().warning('marker_map_json is invalid; using empty landmark map')
            return marker_map

        for entry in entries:
            try:
                marker_id = int(entry['id'])
                marker_map[marker_id] = np.array(
                    [float(entry['x']), float(entry['y']), float(entry['theta'])],
                    dtype=float,
                )
            except (KeyError, TypeError, ValueError):
                continue

        return marker_map

    def pose_cb(self, msg: PoseStamped) -> None:
        pose = msg.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        self.latest_robot_pose = np.array([float(pose.position.x), float(pose.position.y), yaw], dtype=float)

    def timer_cb(self) -> None:
        self._publish_visualization_markers()

        if self.latest_robot_pose is None:
            return

        stamp = self.get_clock().now().to_msg()
        camera_pose_in_world = compose_pose(self.latest_robot_pose, self.camera_base_pose)
        detections = VizMarkerArray()

        for marker_id, marker_pose_in_world in self.marker_map.items():
            marker_pose_in_camera = compose_pose(inverse_pose(camera_pose_in_world), marker_pose_in_world)
            distance = math.hypot(float(marker_pose_in_camera[0]), float(marker_pose_in_camera[1]))
            bearing = math.atan2(float(marker_pose_in_camera[1]), float(marker_pose_in_camera[0]))
            visible = distance <= self.max_detection_range and abs(bearing) <= 0.5 * self.camera_fov_rad

            marker = VizMarker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'camera_link'
            marker.id = int(marker_id)
            marker.type = VizMarker.SPHERE
            marker.action = VizMarker.ADD
            marker.pose.position.x = float(marker_pose_in_camera[0])
            marker.pose.position.y = float(marker_pose_in_camera[1])
            marker.pose.position.z = 0.0
            half_yaw = 0.5 * float(marker_pose_in_camera[2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(half_yaw)
            marker.pose.orientation.w = math.cos(half_yaw)
            marker.scale.x = 0.18 if visible else 0.12
            marker.scale.y = 0.18 if visible else 0.12
            marker.scale.z = 0.18 if visible else 0.12
            marker.color.r = 0.95 if visible else 0.4
            marker.color.g = 0.8 if visible else 0.4
            marker.color.b = 0.2 if visible else 0.4
            marker.color.a = 1.0 if visible else 0.15
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            if visible:
                detections.markers.append(marker)

        self.aruco_pub.publish(detections)

    def _publish_visualization_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        marker_array = VizMarkerArray()

        for marker_id, marker_pose_in_world in self.marker_map.items():
            marker = VizMarker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'odom'
            marker.ns = 'sim_landmarks'
            marker.id = int(marker_id)
            marker.type = VizMarker.SPHERE
            marker.action = VizMarker.ADD
            marker.pose.position.x = float(marker_pose_in_world[0])
            marker.pose.position.y = float(marker_pose_in_world[1])
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.9
            marker.color.g = 0.55
            marker.color.b = 0.1
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array.markers.append(marker)

            label = VizMarker()
            label.header.stamp = stamp
            label.header.frame_id = 'odom'
            label.ns = 'sim_landmarks_label'
            label.id = 1000 + int(marker_id)
            label.type = VizMarker.TEXT_VIEW_FACING
            label.action = VizMarker.ADD
            label.pose.position.x = float(marker_pose_in_world[0])
            label.pose.position.y = float(marker_pose_in_world[1])
            label.pose.position.z = 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(marker_id)
            label.lifetime.sec = 0
            label.lifetime.nanosec = 0
            marker_array.markers.append(label)

        self.viz_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulatedLandmarks()

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
