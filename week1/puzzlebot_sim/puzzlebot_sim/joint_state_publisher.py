import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')
        
        # Create Transform Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create Joint State Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create a Timer
        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        # Robot parameters
        self.start_time = self.get_clock().now()
        self.wheel_radius = 0.035  # meters
        self.wheel_separation = 0.16  # meters
        
        # Wheel angular velocities (rad/s)
        self.omega_left = 2.0
        self.omega_right = 2.0

    #Timer Callback
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Publish map -> odom transform (static - represents map origin)
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        
        # Map is offset from odom to show they are different frames
        # In real robots, this would be updated by SLAM/localization
        t_map.transform.translation.x = 0.5  # Negative offset (inverted relationship)
        t_map.transform.translation.y = 0.5  # Negative offset (inverted relationship)
        t_map.transform.translation.z = 0.0
        
        q_map = transforms3d.euler.euler2quat(0, 0, 0)
        t_map.transform.rotation.x = q_map[1]
        t_map.transform.rotation.y = q_map[2]
        t_map.transform.rotation.z = q_map[3]
        t_map.transform.rotation.w = q_map[0]
        
        self.tf_broadcaster.sendTransform(t_map)
        
        # Publish odom -> base_footprint transform
        t_odom = TransformStamped()
        t_odom.header.stamp = self.get_clock().now().to_msg()
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_footprint'
        
        # Robot orbits around the map origin in odom frame
        # The map is at (-0.5, -0.5) in odom frame (inverse of the map->odom transform)
        
        radius = 0.5
        angular_velocity = 0.5  # rad/s
        angle = angular_velocity * elapsed_time
        
        # Center of circle in odom frame (at map origin)
        center_x = -0.5
        center_y = -0.5
        
        # Position on the circle
        t_odom.transform.translation.x = center_x + radius * np.cos(angle)
        t_odom.transform.translation.y = center_y + radius * np.sin(angle)
        t_odom.transform.translation.z = 0.0
        
        # Rotation to face the direction of movement (tangent to circle)
        heading_angle = angle + np.pi / 2.0  # Perpendicular to radius
        q = transforms3d.euler.euler2quat(0, 0, heading_angle)
        t_odom.transform.rotation.x = q[1]
        t_odom.transform.rotation.y = q[2]
        t_odom.transform.rotation.z = q[3]
        t_odom.transform.rotation.w = q[0]
        
        self.tf_broadcaster.sendTransform(t_odom)
        
        # Publish base_footprint -> base_link transform
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'base_footprint'
        t_base.child_frame_id = 'base_link'
        
        # Fixed height offset only (robot rotates around itself)
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.05
        
        # No rotation
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_base.transform.rotation.x = q[1]
        t_base.transform.rotation.y = q[2]
        t_base.transform.rotation.z = q[3]
        t_base.transform.rotation.w = q[0]
        
        self.tf_broadcaster.sendTransform(t_base)
        
        # Publish left_wheel transform (base_link -> left_wheel)
        t_left = TransformStamped()
        t_left.header.stamp = self.get_clock().now().to_msg()
        t_left.header.frame_id = 'base_link'
        t_left.child_frame_id = 'left_wheel'
        
        t_left.transform.translation.x = 0.0
        t_left.transform.translation.y = self.wheel_separation / 2.0
        t_left.transform.translation.z = -0.04
        
        left_angle = self.omega_left * elapsed_time
        q_left = transforms3d.euler.euler2quat(0, left_angle, 0)
        t_left.transform.rotation.x = q_left[1]
        t_left.transform.rotation.y = q_left[2]
        t_left.transform.rotation.z = q_left[3]
        t_left.transform.rotation.w = q_left[0]
        
        self.tf_broadcaster.sendTransform(t_left)
        
        # Publish right_wheel transform (base_link -> right_wheel)
        t_right = TransformStamped()
        t_right.header.stamp = self.get_clock().now().to_msg()
        t_right.header.frame_id = 'base_link'
        t_right.child_frame_id = 'right_wheel'
        
        t_right.transform.translation.x = 0.0
        t_right.transform.translation.y = -self.wheel_separation / 2.0
        t_right.transform.translation.z = -0.04
        
        right_angle = self.omega_right * elapsed_time
        q_right = transforms3d.euler.euler2quat(0, right_angle, 0)
        t_right.transform.rotation.x = q_right[1]
        t_right.transform.rotation.y = q_right[2]
        t_right.transform.rotation.z = q_right[3]
        t_right.transform.rotation.w = q_right[0]
        
        self.tf_broadcaster.sendTransform(t_right)
        
        # Publish caster_ball transform (base_link -> caster_ball)
        t_caster = TransformStamped()
        t_caster.header.stamp = self.get_clock().now().to_msg()
        t_caster.header.frame_id = 'base_link'
        t_caster.child_frame_id = 'caster_ball'
        
        t_caster.transform.translation.x = -0.12
        t_caster.transform.translation.y = 0.0
        t_caster.transform.translation.z = -0.08
        
        q_caster = transforms3d.euler.euler2quat(0, 0, 0)
        t_caster.transform.rotation.x = q_caster[1]
        t_caster.transform.rotation.y = q_caster[2]
        t_caster.transform.rotation.z = q_caster[3]
        t_caster.transform.rotation.w = q_caster[0]
        
        self.tf_broadcaster.sendTransform(t_caster)
        
        # Publish base_link -> base_laser transform
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'base_laser'
        
        # Position above the base_link (on top of the robot)
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.08
        
        # No rotation
        q_laser = transforms3d.euler.euler2quat(0, 0, 0)
        t_laser.transform.rotation.x = q_laser[1]
        t_laser.transform.rotation.y = q_laser[2]
        t_laser.transform.rotation.z = q_laser[3]
        t_laser.transform.rotation.w = q_laser[0]
        
        self.tf_broadcaster.sendTransform(t_laser)
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_angle, right_angle]
        joint_state.velocity = [self.omega_left, self.omega_right]
        
        self.joint_pub.publish(joint_state)

    def define_TF(self):
        #Create Trasnform Messages
        pass




def main(args=None):
    rclpy.init(args=args)

    node = DronePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()