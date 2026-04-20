import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class KinematicSimulator(Node):
    """Puzzlebot kinematic model node for Mini Challenge 2."""

    def __init__(self) -> None:
        # Required node name from challenge statement.
        super().__init__('puzzlebot_sim')

        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('publish_tf', True)

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = float(self.get_parameter('theta0').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_cb)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.linear_cmd = msg.linear.x
        self.angular_cmd = msg.angular.z

    def timer_cb(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # Nonholonomic model: x_dot=v*cos(theta), y_dot=v*sin(theta), theta_dot=w.
        self.x += self.linear_cmd * math.cos(self.theta) * dt
        self.y += self.linear_cmd * math.sin(self.theta) * dt
        self.theta = self.normalize_angle(self.theta + self.angular_cmd * dt)

        omega_right = (
            (2.0 * self.linear_cmd + self.angular_cmd * self.wheel_base)
            / (2.0 * self.wheel_radius)
        )
        omega_left = (
            (2.0 * self.linear_cmd - self.angular_cmd * self.wheel_base)
            / (2.0 * self.wheel_radius)
        )

        self.right_wheel_angle += omega_right * dt
        self.left_wheel_angle += omega_left * dt

        # Quaternion from yaw using basic trigonometry (no external quaternion library).
        half_yaw = 0.5 * self.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        stamp = now.to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        if self.publish_tf:
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

        wr_msg = Float32()
        wr_msg.data = float(omega_right)
        self.wr_pub.publish(wr_msg)

        wl_msg = Float32()
        wl_msg.data = float(omega_left)
        self.wl_pub.publish(wl_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = ['wheel_left_joint', 'wheel_right_joint']
        joint_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_msg.velocity = [omega_left, omega_right]
        self.joint_pub.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicSimulator()

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
