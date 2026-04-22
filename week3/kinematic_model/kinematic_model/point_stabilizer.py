import math

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class PointStabilizer(Node):
    """PID-based point stabilization controller for differential-drive robot."""

    def __init__(self) -> None:
        super().__init__('point_stabilizer')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('goal_theta', 0.0)

        # PID gains for linear velocity
        self.declare_parameter('kp_x', 0.5)
        self.declare_parameter('ki_x', 0.1)
        self.declare_parameter('kd_x', 0.2)

        # PID gains for angular velocity
        self.declare_parameter('kp_theta', 0.8)
        self.declare_parameter('ki_theta', 0.05)
        self.declare_parameter('kd_theta', 0.3)

        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('angle_tolerance', 0.03)

        self.goal_x = float(self.get_parameter('goal_x').value)  # type: ignore
        self.goal_y = float(self.get_parameter('goal_y').value)  # type: ignore
        self.goal_theta = float(self.get_parameter('goal_theta').value)  # type: ignore

        # PID gains for velocity
        self.kp_x = float(self.get_parameter('kp_x').value)  # type: ignore
        self.ki_x = float(self.get_parameter('ki_x').value)  # type: ignore
        self.kd_x = float(self.get_parameter('kd_x').value)  # type: ignore

        # PID gains for angle
        self.kp_theta = float(self.get_parameter('kp_theta').value)  # type: ignore
        self.ki_theta = float(self.get_parameter('ki_theta').value)  # type: ignore
        self.kd_theta = float(self.get_parameter('kd_theta').value)  # type: ignore

        self.v_max = float(self.get_parameter('v_max').value)  # type: ignore
        self.w_max = float(self.get_parameter('w_max').value)  # type: ignore
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)  # type: ignore
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)  # type: ignore

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_ready = False

        # PID error tracking
        self.integral_error_x = 0.0
        self.integral_error_theta = 0.0
        self.prev_error_x = 0.0
        self.prev_error_theta = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(Vector3, 'pose_error', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.setpoint_sub = self.create_subscription(Vector3, 'next_point', self.setpoint_cb, 10)

        period = 1.0 / float(self.get_parameter('control_rate').value)# type: ignore
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

    def setpoint_cb(self, msg: Vector3) -> None:
        """Receive new setpoint from trajectory generator."""
        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.goal_theta = float(msg.z)
        self.get_logger().debug(
            f'New setpoint received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, theta={self.goal_theta:.2f}'
        )

    def control_loop(self) -> None:
        if not self.pose_ready:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        # Distance error (setpoint for velocity)
        distance_error = math.hypot(dx, dy)
        
        # Angle error (setpoint for angular velocity)
        angle_error = self.normalize_angle(self.goal_theta - self.theta)
        
        # Direction to goal
        direction_to_goal = math.atan2(dy, dx)

        cmd = Twist()
        goal_reached = False
        
        if distance_error < self.position_tolerance and abs(angle_error) < self.angle_tolerance:
            # Goal reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.integral_error_x = 0.0
            self.integral_error_theta = 0.0
            goal_reached = True
            self.get_logger().info('Goal reached!')
        else:
            # PID for linear velocity (distance error)
            dt = 1.0 / float(self.get_parameter('control_rate').value)  # type: ignore
            
            # Proportional term
            p_x = self.kp_x * distance_error
            
            # Integral term (anti-windup)
            self.integral_error_x += distance_error * dt
            self.integral_error_x = max(min(self.integral_error_x, 1.0), -1.0)  # Clamp
            i_x = self.ki_x * self.integral_error_x
            
            # Derivative term
            d_error_x = (distance_error - self.prev_error_x) / dt if dt > 0 else 0.0
            d_x = self.kd_x * d_error_x
            self.prev_error_x = distance_error
            
            v = p_x + i_x + d_x
            
            # PID for angular velocity (angle error)
            p_theta = self.kp_theta * angle_error
            
            # Integral term (anti-windup)
            self.integral_error_theta += angle_error * dt
            self.integral_error_theta = max(min(self.integral_error_theta, 1.0), -1.0)  # Clamp
            i_theta = self.ki_theta * self.integral_error_theta
            
            # Derivative term
            d_error_theta = (angle_error - self.prev_error_theta) / dt if dt > 0 else 0.0
            d_theta = self.kd_theta * d_error_theta
            self.prev_error_theta = angle_error
            
            w = p_theta + i_theta + d_theta
            
            # Saturate velocities
            cmd.linear.x = max(min(v, self.v_max), -self.v_max)
            cmd.angular.z = max(min(w, self.w_max), -self.w_max)

        err = Vector3()
        err.x = self.goal_x - self.x
        err.y = self.goal_y - self.y
        err.z = self.normalize_angle(self.goal_theta - self.theta)

        # Publish goal_reached flag
        goal_msg = Bool()
        goal_msg.data = goal_reached
        self.goal_reached_pub.publish(goal_msg)

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
