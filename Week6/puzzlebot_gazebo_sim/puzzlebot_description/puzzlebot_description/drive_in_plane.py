import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveInPlane(Node):
    def __init__(self) -> None:
        super().__init__('drive_in_plane')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('forward_time', 5.0)
        self.declare_parameter('turn_time', 2.0)
        self.declare_parameter('publish_rate', 20.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)  # type: ignore
        self.angular_speed = float(self.get_parameter('angular_speed').value)  # type: ignore
        self.forward_time = float(self.get_parameter('forward_time').value)  # type: ignore
        self.turn_time = float(self.get_parameter('turn_time').value)  # type: ignore
        self.publish_rate = float(self.get_parameter('publish_rate').value)  # type: ignore

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

    def timer_cb(self) -> None:
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        cycle = self.forward_time + self.turn_time
        phase = elapsed % cycle

        cmd = Twist()
        if phase < self.forward_time:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveInPlane()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
