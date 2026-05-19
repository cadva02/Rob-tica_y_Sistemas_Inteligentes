import math
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_avoidance')

        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('yaw_tolerance', 0.25)
        self.declare_parameter('obstacle_distance', 0.6)
        self.declare_parameter('front_angle', 40.0)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('goal_topic', 'next_point')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 20.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)  # type: ignore
        self.angular_speed = float(self.get_parameter('angular_speed').value)  # type: ignore
        self.goal_x = float(self.get_parameter('goal_x').value)  # type: ignore
        self.goal_y = float(self.get_parameter('goal_y').value)  # type: ignore
        self.goal_theta = float(self.get_parameter('goal_theta').value)  # type: ignore
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)  # type: ignore
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)  # type: ignore
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)  # type: ignore
        self.front_angle = float(self.get_parameter('front_angle').value)  # type: ignore
        self.scan_topic = str(self.get_parameter('scan_topic').value)  # type: ignore
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)  # type: ignore
        self.goal_topic = str(self.get_parameter('goal_topic').value)  # type: ignore
        self.odom_topic = str(self.get_parameter('odom_topic').value)  # type: ignore
        self.publish_rate = float(self.get_parameter('publish_rate').value)  # type: ignore

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.have_goal = False
        self.latest_scan = None
        self.goal_reached = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Vector3, self.goal_topic, self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Node Started.')

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_callback(self, msg: Vector3) -> None:
        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.goal_theta = float(msg.z)
        self.have_goal = True
        self.goal_reached = False
        self.get_logger().info(f'New goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, theta={self.goal_theta:.2f}')

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_odom = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def control_loop(self) -> None:
        cmd = Twist()

        if not self.have_odom:
            self.get_logger().debug('Waiting for odometry...')
            self.cmd_pub.publish(cmd)
            return

        distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        heading_error = self.normalize_angle(heading - self.theta)
        yaw_error = self.normalize_angle(self.goal_theta - self.theta)

        if distance < self.goal_tolerance and abs(yaw_error) < self.yaw_tolerance:
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
            self.goal_reached = True
            self.cmd_pub.publish(cmd)
            return

        obstacle = self.is_obstacle_ahead()
        if obstacle:
            cmd = self.avoid_obstacle()
        else:
            cmd = self.navigate_to_goal(distance, heading_error)

        self.cmd_pub.publish(cmd)

    def is_obstacle_ahead(self) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf)

        center_index = int(round((0.0 - self.latest_scan.angle_min) / self.latest_scan.angle_increment))
        half_angle = math.radians(self.front_angle / 2.0)
        half_offset = max(1, int(round(half_angle / self.latest_scan.angle_increment)))
        indices = np.arange(center_index - half_offset, center_index + half_offset + 1)
        indices = np.clip(indices, 0, len(ranges) - 1)
        front_ranges = ranges[indices]

        return bool(np.any(front_ranges < self.obstacle_distance))

    def avoid_obstacle(self) -> Twist:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed

        if self.latest_scan is None:
            return twist

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf)

        left = self._sector_min(ranges, 20.0, 90.0)
        right = self._sector_min(ranges, -90.0, -20.0)

        if left > right:
            twist.angular.z = self.angular_speed
        else:
            twist.angular.z = -self.angular_speed

        return twist

    def navigate_to_goal(self, distance: float, heading_error: float) -> Twist:
        twist = Twist()
        angle_speed = 1.5 * heading_error
        twist.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_speed))

        if abs(heading_error) < 0.4:
            speed_scale = max(0.0, 1.0 - abs(heading_error) / 0.8)
            twist.linear.x = self.linear_speed * speed_scale
        else:
            twist.linear.x = 0.0

        return twist

    def _sector_min(self, ranges: np.ndarray, start_deg: float, end_deg: float) -> float:
        assert self.latest_scan is not None
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        start_idx = int(round((start_rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment))
        end_idx = int(round((end_rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment))
        start_idx = max(0, min(start_idx, len(ranges) - 1))
        end_idx = max(0, min(end_idx, len(ranges) - 1))
        if start_idx <= end_idx:
            sector = ranges[start_idx:end_idx + 1]
        else:
            sector = np.concatenate((ranges[start_idx:], ranges[:end_idx + 1]))
        return float(np.min(sector))

    def stop_handler(self, signum, frame):
        self.get_logger().info('Interrupt received! Stopping node...')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
