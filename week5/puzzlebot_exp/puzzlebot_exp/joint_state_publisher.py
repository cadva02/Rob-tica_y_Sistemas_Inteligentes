import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Solo necesitamos publicar JointState
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Suscribirse a la odometría para calcular la rotación de las ruedas
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        
        # Timer a 20Hz (0.05s)
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        # Parámetros (Asegúrate que coincidan con tu URDF)
        self.wheel_radius = 0.05
        self.wheel_separation = 0.19 
        
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.omega_left = 0.0
        self.omega_right = 0.0

    def odom_cb(self, msg: Odometry):
        # Obtenemos las velocidades del robot para calcular qué tanto giran las ruedas
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        # Cinemática inversa para ruedas
        if self.wheel_radius > 0:
            self.omega_right = (2.0 * vx + wz * self.wheel_separation) / (2.0 * self.wheel_radius)
            self.omega_left = (2.0 * vx - wz * self.wheel_separation) / (2.0 * self.wheel_radius)

    def timer_cb(self):
        # Actualizar los ángulos de las ruedas (integración simple)
        self.left_wheel_angle += self.omega_left * self.timer_period
        self.right_wheel_angle += self.omega_right * self.timer_period
        
        # Publicar JointState
        # IMPORTANTE: Los nombres deben ser EXACTAMENTE iguales a los del URDF
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['wheel_left_joint', 'wheel_right_joint'] 
        joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = [self.omega_left, self.omega_right]
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DronePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()