import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class SetPointGenerator(Node):
    """Generate a sequence of set points for square and triangle trajectories."""

    def __init__(self) -> None:
        super().__init__('setpoint_generator')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('trajectory_type', 'both')  # 'square', 'triangle', 'both'
        self.declare_parameter('side_length', 0.8)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        self.publish_rate = float(self.get_parameter('publish_rate').value)  # type: ignore
        self.trajectory_type = str(self.get_parameter('trajectory_type').value)  # type: ignore
        self.side_length = float(self.get_parameter('side_length').value)  # type: ignore
        self.start_x = float(self.get_parameter('start_x').value)  # type: ignore
        self.start_y = float(self.get_parameter('start_y').value)  # type: ignore

        self.points = self._build_points(self.trajectory_type)
        self.current_index = 0
        self.last_reached = False

        self.set_point_pub = self.create_publisher(Vector3, 'next_point', 10)
        marker_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', marker_qos)
        self.reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)
        
        self.get_logger().info(
            f'SetPointGenerator started with trajectory_type={self.trajectory_type}, '
            f'{len(self.points)} waypoints'
        )
        # Publish persistent markers for the active trajectory so they are visible in RViz
        try:
            self.publish_waypoint_markers()
        except Exception:
            self.get_logger().warning('Failed to publish waypoint markers')

    def _build_points(self, trajectory_type: str) -> list:
        """Build waypoints based on trajectory type."""
        x0 = self.start_x
        y0 = self.start_y
        l = self.side_length

        # Square trajectory (4 points)
        square_points = [
            (x0 + l, y0, 0.0),
            (x0 + l, y0 + l, math.pi / 2.0),
            (x0, y0 + l, math.pi),
            (x0, y0, -math.pi / 2.0),
        ]

        # Triangle trajectory (3 points - equilateral)
        triangle_points = [
            (x0 + l, y0, 0.0),
            (x0 + l * 0.5, y0 + l * math.sqrt(3) / 2, 2 * math.pi / 3),
            (x0, y0, 4 * math.pi / 3),
        ]

        # store groups for marker publication
        self._square_points = square_points
        self._triangle_points = triangle_points

        if trajectory_type == 'square':
            return square_points
        elif trajectory_type == 'triangle':
            return triangle_points
        elif trajectory_type == 'both':
            # Combine both: square then triangle in loop
            return square_points + triangle_points
        else:
            self.get_logger().warning(f'Unknown trajectory type: {trajectory_type}, using square')
            return square_points

    def goal_reached_cb(self, msg: Bool) -> None:
        """Called when goal is reached. Move to next waypoint."""
        if msg.data and not self.last_reached:
            self.current_index = (self.current_index + 1) % len(self.points)
            x, y, theta = self.points[self.current_index]
            self.get_logger().info(
                f'Goal reached! Moving to waypoint {self.current_index}/{len(self.points)} -> ({x:.2f}, {y:.2f}, {theta:.2f})'
            )
        self.last_reached = msg.data

    def timer_cb(self) -> None:
        """Publish current setpoint waypoint."""
        x, y, theta = self.points[self.current_index]
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(theta)
        self.set_point_pub.publish(msg)

    def publish_waypoint_markers(self) -> None:
        """Publish visualization markers for the active trajectory waypoints."""
        namespace = (self.get_namespace() or '').lstrip('/')
        frame_id = f"{namespace}/odom" if namespace else 'odom'

        marker_id = 0
        markers = []

        def make_marker(x, y, z, m_id, r, g, b, marker_namespace):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = frame_id
            m.ns = marker_namespace
            m.id = int(m_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.05
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            return m

        def make_line_marker(points, m_id, r, g, b, marker_namespace):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = frame_id
            m.ns = marker_namespace
            m.id = int(m_id)
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.03
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            for (x, y, _) in points:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.05
                m.points.append(p)
            if points:
                first_x, first_y, _ = points[0]
                p = Point()
                p.x = float(first_x)
                p.y = float(first_y)
                p.z = 0.05
                m.points.append(p)
            return m

        if self.trajectory_type == 'triangle':
            active_points = getattr(self, '_triangle_points', [])
            marker_namespace = f'{namespace}/triangle' if namespace else 'triangle'
            color = (0.0, 1.0, 0.0)
        elif self.trajectory_type == 'square':
            active_points = getattr(self, '_square_points', [])
            marker_namespace = f'{namespace}/square' if namespace else 'square'
            color = (1.0, 0.0, 0.0)
        else:
            active_points = getattr(self, '_square_points', []) + getattr(self, '_triangle_points', [])
            marker_namespace = f'{namespace}/trajectory' if namespace else 'trajectory'
            color = None

        if color is not None:
            line_marker = make_line_marker(active_points, marker_id, color[0], color[1], color[2], f'{marker_namespace}/line')
            markers.append(line_marker)
            marker_id += 1
            for (x, y, _) in active_points:
                m = make_marker(x, y, 0.0, marker_id, color[0], color[1], color[2], marker_namespace)
                markers.append(m)
                marker_id += 1
        else:
            line_marker_square = make_line_marker(
                getattr(self, '_square_points', []),
                marker_id,
                1.0,
                0.0,
                0.0,
                f'{namespace}/square/line' if namespace else 'square/line',
            )
            markers.append(line_marker_square)
            marker_id += 1

            for (x, y, _) in getattr(self, '_square_points', []):
                m = make_marker(x, y, 0.0, marker_id, 1.0, 0.0, 0.0, f'{namespace}/square' if namespace else 'square')
                markers.append(m)
                marker_id += 1

            line_marker_triangle = make_line_marker(
                getattr(self, '_triangle_points', []),
                marker_id,
                0.0,
                1.0,
                0.0,
                f'{namespace}/triangle/line' if namespace else 'triangle/line',
            )
            markers.append(line_marker_triangle)
            marker_id += 1

            for (x, y, _) in getattr(self, '_triangle_points', []):
                m = make_marker(x, y, 0.0, marker_id, 0.0, 1.0, 0.0, f'{namespace}/triangle' if namespace else 'triangle')
                markers.append(m)
                marker_id += 1

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SetPointGenerator()

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
