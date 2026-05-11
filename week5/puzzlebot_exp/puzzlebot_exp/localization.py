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
        self.declare_parameter('sigma_v', 0.01)
        self.declare_parameter('sigma_w', 0.1)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)  # type: ignore
        self.wheel_base = float(self.get_parameter('wheel_base').value)  # type: ignore
        self.sample_time = float(self.get_parameter('sample_time').value)  # type: ignore

        # process noise (std dev for linear and angular velocity)
        self.sigma_v = float(self.get_parameter('sigma_v').value)  # type: ignore
        self.sigma_w = float(self.get_parameter('sigma_w').value)  # type: ignore

        self.x = float(self.get_parameter('x0').value)  # type: ignore
        self.y = float(self.get_parameter('y0').value)  # type: ignore
        self.theta = float(self.get_parameter('theta0').value)  # type: ignore

        self.odom_frame = str(self.get_parameter('odom_frame').value)  # type: ignore
        self.base_frame = str(self.get_parameter('base_frame').value)  # type: ignore

        self.wr = 0.0
        self.wl = 0.0

        # 3x3 covariance for [x, y, theta]
        # stored as nested lists: P[row][col]
        self.P = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_cb, 10)

        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        self.get_logger().info(
            f'Dead reckoning localisation started | odom_frame={self.odom_frame}, '
            f'base_frame={self.base_frame}'
        )

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
        # propagate mean
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = self.normalize_angle(self.theta + w * dt)

        # propagate covariance P = F P F^T + L Q L^T
        # where state x = [x, y, theta], control u = [v, w]
        # F = df/dx, L = df/du
        sin_th = math.sin(self.theta)
        cos_th = math.cos(self.theta)

        # Jacobian F (3x3)
        F00 = 1.0
        F01 = 0.0
        F02 = -v * sin_th * dt
        F10 = 0.0
        F11 = 1.0
        F12 = v * cos_th * dt
        F20 = 0.0
        F21 = 0.0
        F22 = 1.0

        # Jacobian L (3x2)
        L00 = cos_th * dt
        L01 = 0.0
        L10 = sin_th * dt
        L11 = 0.0
        L20 = 0.0
        L21 = dt

        # control noise covariance Q (2x2)
        q11 = (self.sigma_v ** 2)
        q22 = (self.sigma_w ** 2)

        # compute F * P
        FP00 = F00 * self.P[0][0] + F01 * self.P[1][0] + F02 * self.P[2][0]
        FP01 = F00 * self.P[0][1] + F01 * self.P[1][1] + F02 * self.P[2][1]
        FP02 = F00 * self.P[0][2] + F01 * self.P[1][2] + F02 * self.P[2][2]

        FP10 = F10 * self.P[0][0] + F11 * self.P[1][0] + F12 * self.P[2][0]
        FP11 = F10 * self.P[0][1] + F11 * self.P[1][1] + F12 * self.P[2][1]
        FP12 = F10 * self.P[0][2] + F11 * self.P[1][2] + F12 * self.P[2][2]

        FP20 = F20 * self.P[0][0] + F21 * self.P[1][0] + F22 * self.P[2][0]
        FP21 = F20 * self.P[0][1] + F21 * self.P[1][1] + F22 * self.P[2][1]
        FP22 = F20 * self.P[0][2] + F21 * self.P[1][2] + F22 * self.P[2][2]

        # compute FPF^T (3x3)
        FPFT00 = FP00 * F00 + FP01 * F01 + FP02 * F02
        FPFT01 = FP00 * F10 + FP01 * F11 + FP02 * F12
        FPFT02 = FP00 * F20 + FP01 * F21 + FP02 * F22

        FPFT10 = FP10 * F00 + FP11 * F01 + FP12 * F02
        FPFT11 = FP10 * F10 + FP11 * F11 + FP12 * F12
        FPFT12 = FP10 * F20 + FP11 * F21 + FP12 * F22

        FPFT20 = FP20 * F00 + FP21 * F01 + FP22 * F02
        FPFT21 = FP20 * F10 + FP21 * F11 + FP22 * F12
        FPFT22 = FP20 * F20 + FP21 * F21 + FP22 * F22

        # compute L Q L^T (3x3)
        # L * Q = 3x2 * 2x2 -> 3x2
        LQ00 = L00 * q11 + L01 * 0.0
        LQ01 = L00 * 0.0 + L01 * q22

        LQ10 = L10 * q11 + L11 * 0.0
        LQ11 = L10 * 0.0 + L11 * q22

        LQ20 = L20 * q11 + L21 * 0.0
        LQ21 = L20 * 0.0 + L21 * q22

        # (LQ) * L^T => 3x2 * 2x3 -> 3x3
        LQLT00 = LQ00 * L00 + LQ01 * L01
        LQLT01 = LQ00 * L10 + LQ01 * L11
        LQLT02 = LQ00 * L20 + LQ01 * L21

        LQLT10 = LQ10 * L00 + LQ11 * L01
        LQLT11 = LQ10 * L10 + LQ11 * L11
        LQLT12 = LQ10 * L20 + LQ11 * L21

        LQLT20 = LQ20 * L00 + LQ21 * L01
        LQLT21 = LQ20 * L10 + LQ21 * L11
        LQLT22 = LQ20 * L20 + LQ21 * L21

        # new covariance
        self.P[0][0] = FPFT00 + LQLT00
        self.P[0][1] = FPFT01 + LQLT01
        self.P[0][2] = FPFT02 + LQLT02

        self.P[1][0] = FPFT10 + LQLT10
        self.P[1][1] = FPFT11 + LQLT11
        self.P[1][2] = FPFT12 + LQLT12

        self.P[2][0] = FPFT20 + LQLT20
        self.P[2][1] = FPFT21 + LQLT21
        self.P[2][2] = FPFT22 + LQLT22

        half_yaw = 0.5 * self.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
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
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w

        # fill pose covariance 6x6 (row-major) from 3x3 P ([x,y,theta] -> indices 0,1,5)
        cov = [0.0] * 36
        # x,y -> rows 0,1 ; theta -> row/col 5 (rotation around z)
        cov[0 * 6 + 0] = self.P[0][0]
        cov[0 * 6 + 1] = self.P[0][1]
        cov[0 * 6 + 5] = self.P[0][2]

        cov[1 * 6 + 0] = self.P[1][0]
        cov[1 * 6 + 1] = self.P[1][1]
        cov[1 * 6 + 5] = self.P[1][2]

        cov[5 * 6 + 0] = self.P[2][0]
        cov[5 * 6 + 1] = self.P[2][1]
        cov[5 * 6 + 5] = self.P[2][2]

        odom.pose.covariance = cov

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