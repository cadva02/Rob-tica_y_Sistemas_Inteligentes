import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TriangleTrajectory(Node):
    """Publish cmd_vel commands to execute a triangular trajectory."""

    def __init__(self) -> None:
        super().__init__('triangle_trajectory')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('side_time', 4.0)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('turn_time', (2.0 * math.pi / 3.0) / 0.8)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.side_time = float(self.get_parameter('side_time').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_time = float(self.get_parameter('turn_time').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()

        self.segment_time = self.side_time + self.turn_time
        self.cycle_time = 3.0 * self.segment_time

        self.timer = self.create_timer(1.0 / self.control_rate, self.timer_cb)

    def timer_cb(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        t_cycle = elapsed % self.cycle_time
        step_t = t_cycle % self.segment_time

        cmd = Twist()
        if step_t < self.side_time:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TriangleTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
