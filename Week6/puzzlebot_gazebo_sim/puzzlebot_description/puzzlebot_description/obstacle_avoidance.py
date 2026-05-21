import math
import signal
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleAvoidanceBug0(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_avoidance_bug0')

        # --- PARÁMETROS CONFIGURABLES ---
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('goal_x', 1.45)
        self.declare_parameter('goal_y', 1.20)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.25)
        self.declare_parameter('obstacle_distance', 0.40) # Umbral frontal (40 cm)
        self.declare_parameter('wall_dist_target', 0.35)   # Distancia deseada a la pared derecha (35 cm)
        self.declare_parameter('front_angle', 40.0)
        # Control parameters for smoother wall-following
        self.declare_parameter('kp_wall', 1.0)
        self.declare_parameter('wall_ang_limit', 0.8)
        # Parameters for obstacle detection smoothing and suppression
        self.declare_parameter('obstacle_detection_count', 3)
        self.declare_parameter('wall_exit_suppression', 1.0)
        
        # Nombres de tópicos según tu arquitectura
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('goal_topic', 'next_point')
        self.declare_parameter('odom_topic', 'odom')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 20.0)

        # Asignación de variables desde los parámetros
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_theta = float(self.get_parameter('goal_theta').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.wall_dist_target = float(self.get_parameter('wall_dist_target').value)
        self.front_angle = float(self.get_parameter('front_angle').value)
        
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.obstacle_detection_count = int(self.get_parameter('obstacle_detection_count').value)
        self.wall_exit_suppression = float(self.get_parameter('wall_exit_suppression').value)

        # --- ESTADO INTERNO DEL ROBOT ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.have_goal = True  # Inicia en True porque toma los del parámetro inicial
        self.latest_scan = None
        self.goal_reached = False
        
        # Estados Bug 0: 'GIRAR_HACIA_META', 'AVANZAR_A_META', 'WALL_FOLLOWING'
        self.state = 'GIRAR_HACIA_META'
        self.hit_distance = float('inf')
        # smoothing state for wall following
        self.prev_angular = 0.0
        self.wall_follow_side = 'RIGHT'
        self.kp_wall = float(self.get_parameter('kp_wall').value)
        self.wall_ang_limit = float(self.get_parameter('wall_ang_limit').value)
        # smoothing for obstacle detection
        self.obstacle_ahead_count = 0
        self.last_wall_exit_time = -1e9

        # --- PUBLICADORES Y SUSCRIPTORES ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # use relative topic name so other nodes receive it in same namespace
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Vector3, self.goal_topic, self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Timer de control asignado a la frecuencia configurada
        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        self.get_logger().info('Obstacle Avoidance con Bug 0 Inicializado.')

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_callback(self, msg: Vector3) -> None:
        # Only accept a new goal if it's meaningfully different from the current one
        new_x = float(msg.x)
        new_y = float(msg.y)
        new_theta = float(msg.z)

        dist = math.hypot(new_x - self.goal_x, new_y - self.goal_y)
        ang_diff = abs(self.normalize_angle(new_theta - self.goal_theta))

        # thresholds: 1 cm position, ~0.01 rad orientation
        if dist < 0.01 and ang_diff < 0.01:
            # duplicate/unchanged setpoint -- ignore to avoid resetting state repeatedly
            return

        self.goal_x = new_x
        self.goal_y = new_y
        self.goal_theta = new_theta
        self.have_goal = True
        self.goal_reached = False
        self.state = 'GIRAR_HACIA_META'
        self.hit_distance = float('inf')
        self.get_logger().info(f'Nueva meta recibida: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

    def odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Conversión de Cuaternión a Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def control_loop(self) -> None:
        cmd = Twist()

        # Validar sincronización de sensores
        if not self.have_odom or not self.have_goal or self.latest_scan is None:
            self.cmd_pub.publish(cmd)
            return

        # Si ya se alcanzó el objetivo global, detener motores
        if self.goal_reached:
            self.cmd_pub.publish(cmd)
            return

        # Cálculos geométricos del error hacia la meta
        distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        alpha = self.normalize_angle(heading - self.theta)

        # CONDICIÓN GLOBAL DE PARADA
        if distance < self.goal_tolerance:
            self.get_logger().info('¡Meta alcanzada con éxito!')
            self.goal_reached = True
            
            # Publicar confirmación en /goal_reached
            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            
            self.cmd_pub.publish(cmd)
            return

        # Evaluación de obstáculos usando el LiDAR
        # Smooth obstacle detection: require consecutive detections to flip state
        obstacle_ahead_raw = self.is_obstacle_ahead()
        if obstacle_ahead_raw:
            self.obstacle_ahead_count = min(self.obstacle_detection_count, self.obstacle_ahead_count + 1)
        else:
            # decay counter slowly to avoid flip-flopping
            self.obstacle_ahead_count = max(0, self.obstacle_ahead_count - 1)

        obstacle_ahead = self.obstacle_ahead_count >= self.obstacle_detection_count

        # Suppress re-entering wall-following for a short time after exit
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_wall_exit_time) < self.wall_exit_suppression and self.state != 'WALL_FOLLOWING':
            obstacle_ahead = False

        # --- MÁQUINA DE ESTADOS BUG 0 ---
        if self.state == 'GIRAR_HACIA_META':
            if obstacle_ahead:
                self.hit_distance = distance
                self.wall_follow_side = self.select_wall_side()
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info('[BUG 0] Obstáculo detectado al girar. Pasando a WALL_FOLLOWING.')
                cmd = self.follow_wall(self.wall_follow_side)
            else:
                if abs(alpha) > 0.15:
                    cmd.linear.x = 0.0
                    cmd.angular.z = np.sign(alpha) * self.angular_speed
                else:
                    self.state = 'AVANZAR_A_META'

        elif self.state == 'AVANZAR_A_META':
            if obstacle_ahead:
                self.hit_distance = distance
                self.wall_follow_side = self.select_wall_side()
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 0] Obstáculo detectado en camino libre. Siguiendo pared (hit_dist={distance:.2f}m).')
                cmd = self.follow_wall(self.wall_follow_side)
            else:
                if abs(alpha) > 0.30:
                    self.state = 'GIRAR_HACIA_META'
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0

        elif self.state == 'WALL_FOLLOWING':
            self.wall_follow_side = self.select_wall_side()
            # Evaluar si el camino directo hacia la meta está completamente limpio
            path_clear = self.is_path_to_goal_clear(heading)
            
            # Condición estricta de escape de Bug 0: Frente despejado Y estar más cerca que cuando impactamos
            if not obstacle_ahead and path_clear and distance < (self.hit_distance - 0.10):
                self.get_logger().info('[BUG 0] Escape válido. Volviendo a orientarse a la meta.')
                self.state = 'GIRAR_HACIA_META'
                self.hit_distance = float('inf')
                # record wall exit time to avoid immediate re-entry
                self.last_wall_exit_time = now
                
                # Giro inmediato para romper inercia de la pared
                cmd.linear.x = 0.0
                cmd.angular.z = np.sign(alpha) * self.angular_speed
            else:
                cmd = self.follow_wall(self.wall_follow_side)

        self.cmd_pub.publish(cmd)

    def is_obstacle_ahead(self) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        # Cono frontal adaptado dinámicamente según 'front_angle'
        half_fwd = max(1, int(math.radians(self.front_angle / 2.0) / inc))
        
        # Obtener los puntos frontales (considerando desbordamiento del arreglo circular)
        idxs = [(0 + i) % n for i in range(-half_fwd, half_fwd + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)] # Limpieza de ruidos e infinitos

        return bool(np.any(vals < self.obstacle_distance))

    def _sector_min_range(self, center_deg: float, half_width_deg: float) -> float:
        if self.latest_scan is None:
            return float('inf')

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        center_idx = int(round(math.radians(center_deg) / inc)) % n
        half_width = max(1, int(math.radians(half_width_deg) / inc))
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return float('inf')
        return float(np.min(vals))

    def select_wall_side(self) -> str:
        left_dist = self._sector_min_range(90.0, 30.0)
        right_dist = self._sector_min_range(-90.0, 30.0)

        if not math.isfinite(left_dist) and not math.isfinite(right_dist):
            return self.wall_follow_side

        if left_dist <= right_dist:
            return 'LEFT'
        return 'RIGHT'

    def is_path_to_goal_clear(self, heading: float) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        # Detectar el índice del LiDAR que apunta directamente hacia el ángulo de la meta
        goal_deg = self.normalize_angle(heading - self.theta)
        idx_goal = int(round(goal_deg / inc)) % n

        # Analizar un cono de tolerancia de 20 grados apuntando a la meta
        half_width = max(1, int(math.radians(20.0 / 2.0) / inc))
        idxs = [(idx_goal + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return True
        return float(np.min(vals)) > self.obstacle_distance

    def follow_wall(self, wall_side: str = 'RIGHT') -> Twist:
        twist = Twist()
        
        if self.latest_scan is None:
            return twist

        scan_front = self._sector_min_range(0.0, self.front_angle / 2.0)
        scan_left = self._sector_min_range(90.0, 30.0)
        scan_right = self._sector_min_range(-90.0, 30.0)

        # SMOOTHED WALL-FOLLOWING (dynamic side selection)
        frente_libre = scan_front > self.obstacle_distance

        if not frente_libre:
            # Obstacle directly ahead: stop and pivot left to avoid
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if wall_side == 'RIGHT' else -self.angular_speed
        else:
            # If we have a finite distance to the chosen wall, use P-control
            if wall_side == 'LEFT':
                if math.isfinite(scan_left):
                    error = scan_left - self.wall_dist_target
                    ang = self.kp_wall * error
                else:
                    # No wall detected: gently steer left to re-acquire it
                    ang = 0.3
            else:
                if math.isfinite(scan_right):
                    error = scan_right - self.wall_dist_target
                    ang = -self.kp_wall * error
                else:
                    # No wall detected: gently steer right to re-acquire it
                    ang = -0.3

            # Limit and smooth angular velocity to avoid jitter
            ang = max(min(ang, self.wall_ang_limit), -self.wall_ang_limit)
            ang = 0.4 * ang + 0.6 * self.prev_angular
            self.prev_angular = ang

            # Forward speed reduced when turning sharply or if front is near
            if abs(ang) > 0.45:
                lin = self.linear_speed * 0.45
            else:
                lin = self.linear_speed * 0.85

            if scan_front < (self.obstacle_distance + 0.05):
                lin *= 0.5

            twist.linear.x = max(0.0, min(lin, self.linear_speed))
            twist.angular.z = ang

        return twist

    def stop_handler(self, signum, frame):
        self.get_logger().info('Señal de parada recibida. Finalizando nodo...')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceBug0()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Cierre limpio ejecutado.')
    finally:
        # Forzar parada absoluta del robot en los actuadores al apagar
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()