import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool


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
        self.reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)
        
        self.get_logger().info(
            f'SetPointGenerator started with trajectory_type={self.trajectory_type}, '
            f'{len(self.points)} waypoints'
        )

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
