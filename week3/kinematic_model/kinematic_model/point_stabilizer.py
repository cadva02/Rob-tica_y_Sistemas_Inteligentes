import math

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PointStabilizer(Node):
    """Point stabilization controller for differential-drive robot."""

    def __init__(self) -> None:
        super().__init__('point_stabilizer')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('goal_theta', 0.0)

        self.declare_parameter('k_rho', 0.9)
        self.declare_parameter('k_alpha', 2.0)
        self.declare_parameter('k_beta', -0.6)

        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('angle_tolerance', 0.03)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_theta = float(self.get_parameter('goal_theta').value)

        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_alpha = float(self.get_parameter('k_alpha').value)
        self.k_beta = float(self.get_parameter('k_beta').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_ready = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(Vector3, 'pose_error', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        period = 1.0 / float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(period, self.control_loop)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.pose_ready = True

    def control_loop(self) -> None:
        if not self.pose_ready:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        rho = math.hypot(dx, dy)
        alpha = self.normalize_angle(math.atan2(dy, dx) - self.theta)
        beta = self.normalize_angle(self.goal_theta - self.theta - alpha)

        heading_error = self.normalize_angle(self.goal_theta - self.theta)

        cmd = Twist()
        if rho < self.position_tolerance and abs(heading_error) < self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            v = self.k_rho * rho
            w = self.k_alpha * alpha + self.k_beta * beta

            cmd.linear.x = max(min(v, self.v_max), -self.v_max)
            cmd.angular.z = max(min(w, self.w_max), -self.w_max)

        err = Vector3()
        err.x = rho
        err.y = alpha
        err.z = beta

        self.error_pub.publish(err)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointStabilizer()

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
