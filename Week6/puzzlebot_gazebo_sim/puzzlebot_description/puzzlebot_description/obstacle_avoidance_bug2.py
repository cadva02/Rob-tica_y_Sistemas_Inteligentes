import math
import signal
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleAvoidanceBug2(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_avoidance_bug2')

        # --- PARÁMETROS CONFIGURABLES ---
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('goal_x', 1.45)
        self.declare_parameter('goal_y', 1.20)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.25)
        self.declare_parameter('obstacle_distance', 0.) # Umbral frontal (40 cm)
        self.declare_parameter('wall_dist_target', 0.35)   # Distancia deseada a la pared derecha (35 cm)
        self.declare_parameter('front_angle', 40.0)
        # Control parameters for smoother wall-following
        self.declare_parameter('kp_wall', 1.0)
        self.declare_parameter('wall_ang_limit', 0.8)
        self.declare_parameter('wall_follow_deadband', 0.03)
        self.declare_parameter('wall_recovery_angular', 0.20)
        self.declare_parameter('wall_follow_smoothing', 0.6)
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
        self.wall_follow_deadband = float(self.get_parameter('wall_follow_deadband').value)
        self.wall_recovery_angular = float(self.get_parameter('wall_recovery_angular').value)
        self.wall_follow_smoothing = float(self.get_parameter('wall_follow_smoothing').value)

        # --- ESTADO INTERNO DEL ROBOT ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.have_goal = True  # Inicia en True porque toma los del parámetro inicial
        self.latest_scan = None
        self.goal_reached = False
        
        # Estados Bug 2: 'GIRAR_HACIA_META', 'AVANZAR_A_META', 'WALL_FOLLOWING'
        self.state = 'GIRAR_HACIA_META'
        self.start_x = 0.0
        self.start_y = 0.0
        self.hit_point_x = 0.0
        self.hit_point_y = 0.0
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
        self.get_logger().info('Obstacle Avoidance con Bug 2 Inicializado.')

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def log_state_transition(self, from_state: str, to_state: str, reason: str) -> None:
        self.get_logger().info(f'[BUG 2] {from_state} -> {to_state}: {reason}')

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
        self.start_x = self.x if self.have_odom else 0.0
        self.start_y = self.y if self.have_odom else 0.0
        self.hit_point_x = 0.0
        self.hit_point_y = 0.0
        self.hit_distance = float('inf')
        self.get_logger().warn(
        f'NEW GOAL CALLBACK: '
        f'({new_x:.2f}, {new_y:.2f}, {new_theta:.2f})'
        )
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
        if self.have_goal and self.state == 'GIRAR_HACIA_META' and self.start_x == 0.0 and self.start_y == 0.0:
            self.start_x = self.x
            self.start_y = self.y

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

        self.get_logger().info(
        f'Pose=({self.x:.2f},{self.y:.2f}) '
        f'Goal=({self.goal_x:.2f},{self.goal_y:.2f}) '
        f'Distance={distance:.3f}'
        )

        # CONDICIÓN GLOBAL DE PARADA
        if distance < self.goal_tolerance:
            self.get_logger().info('¡Meta alcanzada con éxito!')
            self.goal_reached = True
            
            # Publicar confirmación en /goal_reached
            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            self.get_logger().warn(
                f'GOAL CONDITION TRIGGERED! '
                f'distance={distance:.3f}'
            )
            
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

        # --- MÁQUINA DE ESTADOS BUG 2 ---
        if self.state == 'GIRAR_HACIA_META':
            if obstacle_ahead:
                # Registrar el punto donde golpeamos el obstáculo
                self.hit_point_x = self.x
                self.hit_point_y = self.y
                self.hit_distance = distance
                self.wall_follow_side = self.select_wall_side()
                self.log_state_transition('GIRAR_HACIA_META', 'WALL_FOLLOWING', 'obstáculo detectado al girar hacia meta')
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 2] Obstáculo detectado al girar. Hit point: ({self.hit_point_x:.2f}, {self.hit_point_y:.2f})')
                cmd = self.follow_wall(self.wall_follow_side)
            else:
                if abs(alpha) > 0.15:
                    cmd.linear.x = 0.0
                    cmd.angular.z = np.sign(alpha) * self.angular_speed
                else:
                    self.log_state_transition('GIRAR_HACIA_META', 'AVANZAR_A_META', 'orientación hacia la meta completada')
                    self.state = 'AVANZAR_A_META'

        elif self.state == 'AVANZAR_A_META':
            if obstacle_ahead:
                # Registrar el punto donde golpeamos el obstáculo
                self.hit_point_x = self.x
                self.hit_point_y = self.y
                self.hit_distance = distance
                self.wall_follow_side = self.select_wall_side()
                self.log_state_transition('AVANZAR_A_META', 'WALL_FOLLOWING', 'obstáculo detectado en avance hacia meta')
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 2] Obstáculo detectado en camino libre. Hit point: ({self.hit_point_x:.2f}, {self.hit_point_y:.2f})')
                cmd = self.follow_wall(self.wall_follow_side)
            else:
                if abs(alpha) > 0.30:
                    self.log_state_transition('AVANZAR_A_META', 'GIRAR_HACIA_META', 'corrección de rumbo necesaria')
                    self.state = 'GIRAR_HACIA_META'
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0

        elif self.state == 'WALL_FOLLOWING':
            # Bug 2: Verificar si el robot está en la línea M (línea recta start -> goal)
            # y si el camino hacia la meta está despejado
            on_m_line = self.is_on_m_line()
            path_clear = self.is_path_to_goal_clear(heading)
            
            # Condición de escape de Bug 2: 
            # El robot está nuevamente en la línea M, el camino está despejado, 
            # no hay obstáculo adelante y está más cerca de la meta que el punto de impacto.
            if (
                    on_m_line
                    and path_clear
                    and not obstacle_ahead
                    and distance + 0.05 < self.hit_distance
                    and abs(alpha) < 0.50
                ):
                self.log_state_transition('WALL_FOLLOWING', 'GIRAR_HACIA_META', 'escape válido en línea M')
                self.get_logger().info('[BUG 2] Escape válido. Volviendo a orientarse a la meta.')
                self.state = 'GIRAR_HACIA_META'
                self.hit_point_x = 0.0
                self.hit_point_y = 0.0
                self.hit_distance = float('inf')
                # record wall exit time to avoid immediate re-entry
                self.last_wall_exit_time = now
                
                # Giro inmediato para romper inercia de la pared
                cmd.linear.x = self.linear_speed * 0.5

                if self.wall_follow_side == 'RIGHT':
                    cmd.angular.z = self.angular_speed
                else:
                    cmd.angular.z = -self.angular_speed
            else:
                cmd = self.follow_wall(self.wall_follow_side)

        self.cmd_pub.publish(cmd)

    def is_obstacle_ahead(self) -> bool:
        if self.latest_scan is None:
            return False

        half_fwd = max(1, int(math.radians(self.front_angle / 2.0) / self.latest_scan.angle_increment))
        center_idx = self._angle_to_index(0.0)
        idxs = [(center_idx + i) for i in range(-half_fwd, half_fwd + 1)]
        idxs = [max(0, min(len(self.latest_scan.ranges) - 1, idx)) for idx in idxs]

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        return bool(np.any(vals < self.obstacle_distance))

    def _angle_to_index(self, angle: float) -> int:
        if self.latest_scan is None:
            return 0
        idx = int(round((angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment))
        return max(0, min(len(self.latest_scan.ranges) - 1, idx))

    def _sector_min_range(self, center_deg: float, half_width_deg: float) -> float:
        if self.latest_scan is None:
            return float('inf')

        center_rad = math.radians(center_deg)
        center_idx = self._angle_to_index(center_rad)
        half_width = max(1, int(math.radians(half_width_deg) / self.latest_scan.angle_increment))
        idxs = [(center_idx + i) for i in range(-half_width, half_width + 1)]
        idxs = [max(0, min(len(self.latest_scan.ranges) - 1, idx)) for idx in idxs]

        ranges = np.array(self.latest_scan.ranges, dtype=float)
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

    def is_on_m_line(self, tolerance: float = 0.15) -> bool:
        """
        Verifica si el robot está en la línea M (línea recta desde el punto de inicio al objetivo).
        """
        dx_line = self.goal_x - self.start_x
        dy_line = self.goal_y - self.start_y
        line_length = math.hypot(dx_line, dy_line)

        if line_length < 1e-6:
            return False

        dx_robot = self.x - self.start_x
        dy_robot = self.y - self.start_y

        cross_product = abs(dx_line * dy_robot - dy_line * dx_robot)
        distance_to_line = cross_product / line_length

        dot_product = dx_line * dx_robot + dy_line * dy_robot
        on_segment = 0 <= dot_product <= (line_length ** 2)

        return (distance_to_line < tolerance) and on_segment

    def is_path_to_goal_clear(self, heading: float) -> bool:
        if self.latest_scan is None:
            return False

        goal_angle = self.normalize_angle(heading - self.theta)
        idx_goal = self._angle_to_index(goal_angle)

        half_width = max(1, int(math.radians(20.0 / 2.0) / self.latest_scan.angle_increment))
        idxs = [(idx_goal + i) for i in range(-half_width, half_width + 1)]
        idxs = [max(0, min(len(self.latest_scan.ranges) - 1, idx)) for idx in idxs]

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return True
        return float(np.min(vals)) > (self.obstacle_distance + 0.08)

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
                    ang = 0.0 if abs(error) < self.wall_follow_deadband else self.kp_wall * error
                else:
                    ang = self.wall_recovery_angular
            else:
                if math.isfinite(scan_right):
                    error = scan_right - self.wall_dist_target
                    ang = 0.0 if abs(error) < self.wall_follow_deadband else -self.kp_wall * error
                else:
                    ang = -self.wall_recovery_angular

            # Limit and smooth angular velocity to avoid jitter
            ang = max(min(ang, self.wall_ang_limit), -self.wall_ang_limit)
            ang = self.wall_follow_smoothing * self.prev_angular + (1.0 - self.wall_follow_smoothing) * ang
            self.prev_angular = ang

            if abs(ang) < 0.05:
                ang = 0.0

            # Forward speed reduced when turning sharply or if front is near
            if abs(ang) > 0.40:
                lin = self.linear_speed * 0.40
            elif abs(ang) > 0.20:
                lin = self.linear_speed * 0.65
            else:
                lin = self.linear_speed * 0.90

            if scan_front < (self.obstacle_distance + 0.10):
                lin *= 0.6

            twist.linear.x = max(0.0, min(lin, self.linear_speed))
            twist.angular.z = ang

        return twist

    def stop_handler(self, signum, frame):
        self.get_logger().info('Señal de parada recibida. Finalizando nodo...')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceBug2()
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
