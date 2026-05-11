import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class KinematicSimulator(Node):

    def __init__(self) -> None:
        super().__init__('puzzlebot_sim')

        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_frame', 'odom')

        self.update_rate = float(self.get_parameter('update_rate').value)  # type: ignore
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)  # type: ignore
        self.wheel_base = float(self.get_parameter('wheel_base').value)  # type: ignore

        self.x = float(self.get_parameter('x0').value)  # type: ignore
        self.y = float(self.get_parameter('y0').value)  # type: ignore
        self.theta = float(self.get_parameter('theta0').value)  # type: ignore

        self.publish_tf = bool(self.get_parameter('publish_tf').value)  # type: ignore
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)  # type: ignore

        self.odom_frame = str(self.get_parameter('odom_frame').value)  # type: ignore
        self.base_frame = str(self.get_parameter('base_frame').value)  # type: ignore
        self.pose_frame = str(self.get_parameter('pose_frame').value)  # type: ignore

        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)

        self.joint_pub = None
        if self.publish_joint_states:
            self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info(
            f'Kinematic simulator started | odom_frame={self.odom_frame}, '
            f'base_frame={self.base_frame}, pose_frame={self.pose_frame}, '
            f'publish_tf={self.publish_tf}, publish_joint_states={self.publish_joint_states}'
        )

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

        half_yaw = 0.5 * self.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        stamp = now.to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.pose_frame
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

        wr_msg = Float32()
        wr_msg.data = float(omega_right)
        self.wr_pub.publish(wr_msg)

        wl_msg = Float32()
        wl_msg.data = float(omega_left)
        self.wl_pub.publish(wl_msg)

        if self.publish_joint_states and self.joint_pub is not None:
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