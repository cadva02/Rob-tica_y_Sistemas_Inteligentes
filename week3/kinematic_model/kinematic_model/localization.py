import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class DeadReckoningLocalization(Node):
    """Estimate robot pose from wr/wl wheel speeds (dead reckoning)."""

    def __init__(self) -> None:
        super().__init__('localisation')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('sample_time', 0.02)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)# type: ignore
        self.wheel_base = float(self.get_parameter('wheel_base').value)# type: ignore
        self.sample_time = float(self.get_parameter('sample_time').value)# type: ignore

        self.x = float(self.get_parameter('x0').value)# type: ignore
        self.y = float(self.get_parameter('y0').value)# type: ignore
        self.theta = float(self.get_parameter('theta0').value)# type: ignore

        self.wr = 0.0
        self.wl = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, 10)

        self.timer = self.create_timer(self.sample_time, self.timer_cb)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def wr_cb(self, msg: Float32) -> None:
        self.wr = float(msg.data)

    def wl_cb(self, msg: Float32) -> None:
        self.wl = float(msg.data)

    def timer_cb(self) -> None:
        v = 0.5 * self.wheel_radius * (self.wr + self.wl)
        w = self.wheel_radius * (self.wr - self.wl) / self.wheel_base

        dt = self.sample_time
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = self.normalize_angle(self.theta + w * dt)

        half_yaw = 0.5 * self.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningLocalization()

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
