import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool


class SetPointGenerator(Node):
    """Generate a sequence of set points for controller testing."""

    def __init__(self) -> None:
        super().__init__('setpoint_generator')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('trajectory_type', 'square')
        self.declare_parameter('side_length', 0.8)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.trajectory_type = str(self.get_parameter('trajectory_type').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)

        self.points = self.build_points(self.trajectory_type)
        self.current_index = 0
        self.last_reached = False

        self.set_point_pub = self.create_publisher(Pose2D, 'set_point', 10)
        self.reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

    def build_points(self, trajectory_type: str):
        if trajectory_type == 'pentagon':
            return self.regular_polygon_points(5, self.side_length, self.start_x, self.start_y)
        if trajectory_type == 'triangle':
            return self.regular_polygon_points(3, self.side_length, self.start_x, self.start_y)

        # Default: square aligned to axes.
        x = self.start_x
        y = self.start_y
        l = self.side_length
        return [
            (x + l, y, 0.0),
            (x + l, y + l, math.pi / 2.0),
            (x, y + l, math.pi),
            (x, y, -math.pi / 2.0),
        ]

    @staticmethod
    def regular_polygon_points(sides: int, side_length: float, x0: float, y0: float):
        radius = side_length / (2.0 * math.sin(math.pi / sides))
        points = []
        for i in range(sides):
            ang = 2.0 * math.pi * i / sides
            x = x0 + radius * math.cos(ang)
            y = y0 + radius * math.sin(ang)
            points.append((x, y, ang))
        return points

    def goal_reached_cb(self, msg: Bool) -> None:
        if msg.data and not self.last_reached:
            self.current_index = (self.current_index + 1) % len(self.points)
        self.last_reached = msg.data

    def timer_cb(self) -> None:
        x, y, theta = self.points[self.current_index]
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
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
