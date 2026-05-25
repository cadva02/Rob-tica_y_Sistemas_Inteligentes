import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

try:
    from aruco_msgs.msg import MarkerArray  # type: ignore[reportMissingImports]
except ImportError:  # pragma: no cover - optional at runtime
    MarkerArray = None


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


def quaternion_to_yaw(orientation) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EkfLocalization(Node):
    """EKF localization using wheel encoders and ArUco pose observations."""

    def __init__(self) -> None:
        super().__init__('localisation_ekf')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('sample_time', 0.02)
        self.declare_parameter('sigma_v', 0.01)
        self.declare_parameter('sigma_w', 0.1)
        self.declare_parameter('sigma_obs_x', 0.08)
        self.declare_parameter('sigma_obs_y', 0.08)
        self.declare_parameter('sigma_obs_theta', 0.15)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_base_x', 0.1241)
        self.declare_parameter('camera_base_y', 0.0)
        self.declare_parameter('camera_base_theta', 0.0)
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('publish_dead_reckoning_aux', True)
        self.declare_parameter(
            'marker_map_json',
            json.dumps(
                [
                    {'id': 0, 'x': 2.5, 'y': -0.5, 'theta': math.pi},
                    {'id': 1, 'x': 2.5, 'y': 2.5, 'theta': 3.0 * math.pi / 2.0},
                    {'id': 2, 'x': -0.5, 'y': 2.5, 'theta': 0.0},
                    {'id': 3, 'x': -0.5, 'y': -0.5, 'theta': math.pi / 2.0},
                ]
            ),
        )

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.sample_time = float(self.get_parameter('sample_time').value)
        self.sigma_v = float(self.get_parameter('sigma_v').value)
        self.sigma_w = float(self.get_parameter('sigma_w').value)
        self.sigma_obs_x = float(self.get_parameter('sigma_obs_x').value)
        self.sigma_obs_y = float(self.get_parameter('sigma_obs_y').value)
        self.sigma_obs_theta = float(self.get_parameter('sigma_obs_theta').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.camera_base_pose = np.array(
            [
                float(self.get_parameter('camera_base_x').value),
                float(self.get_parameter('camera_base_y').value),
                float(self.get_parameter('camera_base_theta').value),
            ],
            dtype=float,
        )

        marker_map_json = str(self.get_parameter('marker_map_json').value)
        self.marker_map = self._parse_marker_map(marker_map_json)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.publish_dead_reckoning_aux = bool(self.get_parameter('publish_dead_reckoning_aux').value)

        self.state = np.array(
            [
                float(self.get_parameter('x0').value),
                float(self.get_parameter('y0').value),
                float(self.get_parameter('theta0').value),
            ],
            dtype=float,
        )

        self.covariance = np.diag([0.05, 0.05, 0.10]).astype(float)

        self.wr = 0.0
        self.wl = 0.0
        self.last_marker_observations: list[np.ndarray] = []

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_dr_pub = self.create_publisher(Odometry, 'odom_dr', 10)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, 10)

        if MarkerArray is not None:
            self.marker_sub = self.create_subscription(
                MarkerArray,
                self.aruco_topic,
                self.marker_cb,
                10,
            )
        else:
            self.marker_sub = None
            self.get_logger().warning(
                'aruco_msgs is not available; EKF will run in prediction-only mode until ArUco '
                f'observations are provided on {self.aruco_topic}'
            )

        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        self.get_logger().info(
            'EKF localization started | odom_frame=%s, base_frame=%s, markers=%d, aruco_topic=%s'
            % (self.odom_frame, self.base_frame, len(self.marker_map), self.aruco_topic)
        )

    def _parse_marker_map(self, marker_map_json: str) -> dict[int, np.ndarray]:
        marker_map: dict[int, np.ndarray] = {}
        try:
            entries = json.loads(marker_map_json)
        except json.JSONDecodeError:
            self.get_logger().warning('marker_map_json is invalid; using empty marker map')
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

    def wr_cb(self, msg: Float32) -> None:
        self.wr = float(msg.data)

    def wl_cb(self, msg: Float32) -> None:
        self.wl = float(msg.data)

    def marker_cb(self, msg) -> None:
        observations: list[np.ndarray] = []
        for marker in getattr(msg, 'markers', []):
            marker_id = self._get_marker_id(marker)
            if marker_id is None or marker_id not in self.marker_map:
                continue

            marker_pose = self._extract_marker_pose(marker)
            if marker_pose is None:
                continue

            robot_pose = self._robot_pose_from_marker(marker_id, marker_pose)
            if robot_pose is not None:
                observations.append(robot_pose)

        self.last_marker_observations = observations

    def _get_marker_id(self, marker) -> int | None:
        marker_id = getattr(marker, 'id', None)
        if marker_id is None:
            marker_id = getattr(marker, 'marker_id', None)
        if marker_id is None:
            return None
        try:
            return int(marker_id)
        except (TypeError, ValueError):
            return None

    def _extract_marker_pose(self, marker) -> np.ndarray | None:
        pose_field = getattr(marker, 'pose', None)
        if pose_field is None:
            return None

        if hasattr(pose_field, 'pose'):
            pose_field = pose_field.pose

        if hasattr(pose_field, 'position') and hasattr(pose_field, 'orientation'):
            position = pose_field.position
            orientation = pose_field.orientation
            yaw = quaternion_to_yaw(orientation)
            return np.array([float(position.x), float(position.y), yaw], dtype=float)

        return None

    def _robot_pose_from_marker(self, marker_id: int, marker_pose_in_camera: np.ndarray) -> np.ndarray | None:
        marker_pose_in_world = self.marker_map.get(marker_id)
        if marker_pose_in_world is None:
            return None

        camera_pose_in_base = self.camera_base_pose
        base_pose_in_camera = inverse_pose(camera_pose_in_base)
        marker_pose_in_camera_inv = inverse_pose(marker_pose_in_camera)
        robot_pose = compose_pose(marker_pose_in_world, compose_pose(marker_pose_in_camera_inv, base_pose_in_camera))
        return robot_pose

    def timer_cb(self) -> None:
        dead_reckoning_state, dead_reckoning_covariance = self._predict()

        if self.publish_dead_reckoning_aux:
            self._publish_dead_reckoning_state(dead_reckoning_state, dead_reckoning_covariance)

        if self.last_marker_observations:
            for observation in self.last_marker_observations:
                self._update(observation)
            self.last_marker_observations = []

        self._publish_state()

    def _predict(self) -> tuple[np.ndarray, np.ndarray]:
        dt = self.sample_time
        v = 0.5 * self.wheel_radius * (self.wr + self.wl)
        w = self.wheel_radius * (self.wr - self.wl) / self.wheel_base

        theta = float(self.state[2])
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        self.state[0] += v * cos_theta * dt
        self.state[1] += v * sin_theta * dt
        self.state[2] = normalize_angle(theta + w * dt)

        f_jacobian = np.array(
            [
                [1.0, 0.0, -v * sin_theta * dt],
                [0.0, 1.0, v * cos_theta * dt],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        l_jacobian = np.array(
            [
                [cos_theta * dt, 0.0],
                [sin_theta * dt, 0.0],
                [0.0, dt],
            ],
            dtype=float,
        )

        process_covariance = np.diag([self.sigma_v**2, self.sigma_w**2]).astype(float)
        self.covariance = f_jacobian @ self.covariance @ f_jacobian.T + l_jacobian @ process_covariance @ l_jacobian.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return self.state.copy(), self.covariance.copy()

    def _update(self, observation: np.ndarray) -> None:
        measurement = np.array(
            [float(observation[0]), float(observation[1]), float(observation[2])],
            dtype=float,
        )

        innovation = measurement - self.state
        innovation[2] = normalize_angle(float(innovation[2]))

        measurement_covariance = np.diag(
            [self.sigma_obs_x**2, self.sigma_obs_y**2, self.sigma_obs_theta**2]
        ).astype(float)

        s_matrix = self.covariance + measurement_covariance
        gain = self.covariance @ np.linalg.inv(s_matrix)

        self.state = self.state + gain @ innovation
        self.state[2] = normalize_angle(float(self.state[2]))

        identity = np.eye(3, dtype=float)
        joseph = (identity - gain) @ self.covariance @ (identity - gain).T + gain @ measurement_covariance @ gain.T
        self.covariance = 0.5 * (joseph + joseph.T)

    def _publish_state(self) -> None:
        self._publish_odom_message(self.odom_pub, self.state, self.covariance, publish_tf=True)

    def _publish_dead_reckoning_state(self, state: np.ndarray, covariance: np.ndarray) -> None:
        self._publish_odom_message(self.odom_dr_pub, state, covariance, publish_tf=False)

    def _publish_odom_message(
        self,
        publisher,
        state: np.ndarray,
        covariance_matrix: np.ndarray,
        publish_tf: bool,
    ) -> None:
        x = float(state[0])
        y = float(state[1])
        theta = float(state[2])
        half_yaw = 0.5 * theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        if publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = 0.5 * self.wheel_radius * (self.wr + self.wl)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.wheel_radius * (self.wr - self.wl) / self.wheel_base

        covariance = [0.0] * 36
        covariance[0] = float(covariance_matrix[0, 0])
        covariance[1] = float(covariance_matrix[0, 1])
        covariance[5] = float(covariance_matrix[0, 2])
        covariance[6] = float(covariance_matrix[1, 0])
        covariance[7] = float(covariance_matrix[1, 1])
        covariance[11] = float(covariance_matrix[1, 2])
        covariance[30] = float(covariance_matrix[2, 0])
        covariance[31] = float(covariance_matrix[2, 1])
        covariance[35] = float(covariance_matrix[2, 2])
        odom.pose.covariance = covariance

        publisher.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfLocalization()

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
