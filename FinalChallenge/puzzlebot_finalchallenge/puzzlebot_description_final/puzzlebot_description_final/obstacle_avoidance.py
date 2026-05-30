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
        self.declare_parameter('obstacle_distance', 0.40) 
        self.declare_parameter('wall_dist_target', 0.35)   
        self.declare_parameter('front_angle', 40.0)
        self.declare_parameter('kp_wall', 1.0)
        self.declare_parameter('wall_ang_limit', 0.8)
        self.declare_parameter('obstacle_detection_count', 3)
        self.declare_parameter('wall_exit_suppression', 1.0)
        
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('goal_topic', 'next_point')
        self.declare_parameter('odom_topic', 'odom')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 20.0)

        # Asignación de variables
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

        # --- ESTADO INTERNO ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.have_goal = True  
        self.latest_scan = None
        self.goal_reached = False
        
        self.state = 'GIRAR_HACIA_META'
        self.hit_distance = float('inf')
        self.prev_angular = 0.0
        self.wall_follow_side = 'RIGHT' # Lado elegido (se congelará al impactar)
        self.kp_wall = float(self.get_parameter('kp_wall').value)
        self.wall_ang_limit = float(self.get_parameter('wall_ang_limit').value)
        self.obstacle_ahead_count = 0
        self.last_wall_exit_time = -1e9

        # --- PUBLICADORES Y SUSCRIPTORES ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Vector3, self.goal_topic, self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        self.get_logger().info('Obstacle Avoidance con Bug 0 Corregido e Inicializado.')

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def goal_callback(self, msg: Vector3) -> None:
        new_x = float(msg.x)
        new_y = float(msg.y)
        new_theta = float(msg.z)

        dist = math.hypot(new_x - self.goal_x, new_y - self.goal_y)
        ang_diff = abs(self.normalize_angle(new_theta - self.goal_theta))

        if dist < 0.01 and ang_diff < 0.01:
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
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def control_loop(self) -> None:
        cmd = Twist()

        if not self.have_odom or not self.have_goal or self.latest_scan is None:
            self.cmd_pub.publish(cmd)
            return

        if self.goal_reached:
            self.cmd_pub.publish(cmd)
            return

        distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        alpha = self.normalize_angle(heading - self.theta)

        # CONDICIÓN GLOBAL DE PARADA
        if distance < self.goal_tolerance:
            self.get_logger().info('¡Meta alcanzada con éxito!')
            self.goal_reached = True
            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            self.cmd_pub.publish(cmd)
            return

        # Evaluación del obstáculo al frente (para entrar al modo evasión)
        obstacle_ahead_raw = self.is_obstacle_ahead()
        if obstacle_ahead_raw:
            self.obstacle_ahead_count = min(self.obstacle_detection_count, self.obstacle_ahead_count + 1)
        else:
            self.obstacle_ahead_count = max(0, self.obstacle_ahead_count - 1)

        obstacle_ahead = self.obstacle_ahead_count >= self.obstacle_detection_count

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_wall_exit_time) < self.wall_exit_suppression and self.state != 'WALL_FOLLOWING':
            obstacle_ahead = False

        # --- CORRECCIÓN 2: CONDICIÓN DE SALIDA AUTÉNTICA DE BUG 0 ---
        # Salir de WALL_FOLLOWING sólo si la trayectoria directa a la meta está limpia
        if self.state == 'WALL_FOLLOWING' and self.is_path_to_goal_clear(heading):
            self.get_logger().info('[BUG 0] Trayectoria hacia la meta despejada. Retomando rumbo directo.')
            self.state = 'GIRAR_HACIA_META'
            self.hit_distance = float('inf')
            self.last_wall_exit_time = now

        # --- MÁQUINA DE ESTADOS BUG 0 ---
        if self.state == 'GIRAR_HACIA_META':
            if obstacle_ahead:
                self.hit_distance = distance
                # CORRECCIÓN 1: Seleccionar el lado SÓLO al inicio del impacto
                self.wall_follow_side = self.select_wall_side()
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 0] Obstáculo detectado al girar. Siguiendo pared por la {self.wall_follow_side}.')
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
                # CORRECCIÓN 1: Seleccionar el lado SÓLO al inicio del impacto
                self.wall_follow_side = self.select_wall_side()
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 0] Obstáculo detectado. Siguiendo pared por la {self.wall_follow_side}.')
                cmd = self.follow_wall(self.wall_follow_side)
            else:
                if abs(alpha) > 0.30:
                    self.state = 'GIRAR_HACIA_META'
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0

        elif self.state == 'WALL_FOLLOWING':
            # CORRECCIÓN 1: Ya no llamamos a select_wall_side() aquí. Mantenemos el estado congelado.
            cmd = self.follow_wall(self.wall_follow_side)

        self.cmd_pub.publish(cmd)

    def is_obstacle_ahead(self) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        half_fwd = max(1, int(math.radians(self.front_angle / 2.0) / inc))
        idxs = [(0 + i) % n for i in range(-half_fwd, half_fwd + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)] 

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
            return 'RIGHT' # Por defecto si no ve nada claro

        if left_dist <= right_dist:
            return 'LEFT'
        return 'RIGHT'

    def is_path_to_goal_clear(self, heading: float) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        # MANTENER EN RADIANES NATIVOS
        goal_rad = self.normalize_angle(heading - self.theta)
        
        # Analizar un cono de 20 grados apuntando dinámicamente hacia la posición de la meta
        half_width = max(1, int(math.radians(20.0 / 2.0) / inc))
        center_idx = int(round(goal_rad / inc)) % n  # <--- Usar goal_rad directamente aquí
        
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return True
        
        return float(np.min(vals)) > (self.obstacle_distance + 0.05)

    def follow_wall(self, wall_side: str = 'RIGHT') -> Twist:
        twist = Twist()

        if self.latest_scan is None:
            return twist
        
        # Factor de simetría (Standard: CCW positivo girando a la izquierda)
        side_sign = 1.0 if wall_side == 'LEFT' else -1.0

        # Lectura de sectores
        d_ps5 = self._sector_min_range(90.0 * side_sign, 15.0)   # Lateral
        d_ps6 = self._sector_min_range(45.0 * side_sign, 15.0)   # Diagonal frontal
        d_ps7 = self._sector_min_range(0.0, 15.0)                # Frente estricto

        if not math.isfinite(d_ps5): d_ps5 = float('inf')
        if not math.isfinite(d_ps6): d_ps6 = float('inf')
        if not math.isfinite(d_ps7): d_ps7 = float('inf')

        # Distancia medida al muro (ponderada para suavizar)
        measured_wall_dist = d_ps5 if d_ps5 != float('inf') else d_ps6

        # --- CORRECCIÓN DE SIGNOS: CONTROL PROPORCIONAL NATIVO (IZQUIERDO) ---
        if measured_wall_dist == float('inf'):
            correction_w = 0.0
        else:
            # --- EL CAMBIO CLAVE AQUÍ ---
            # Definimos Error = Target - Medida
            # Si medimos 0.20m (Cerca) y target es 0.35m: Error = +0.15 (Positivo)
            # Nativamente (Lado Izquierdo), un error positivo debe hacernos girar a la 
            # DERECHA (alejar la nariz del muro). En ROS, Derecha es NEGATIVO.
            error_dist = self.wall_dist_target - measured_wall_dist

            # kp_dist debe ser negativo para que error positivo -> giro negativo (Derecha)
            k_p_dist = -3.0 # Aumenté un poco la agresividad de corrección
            correction_w = error_dist * k_p_dist

        # --- LÓGICA DE VELOCIDADES EN CONFIGURACIÓN IZQUIERDA NATIVA ---
        v_base = self.linear_speed 
        FRONT_THRESHOLD = self.obstacle_distance 

        if d_ps7 < FRONT_THRESHOLD:
            # ¡MURO DE FRENTE! Evadir suavemente.
            v = v_base * 0.2
            # Nativamente (Izquierda), ante muro frontal, girar a la DERECHA (Negativo)
            w = -self.angular_speed * 0.7 
        else:
            # SEGUIMIENTO NORMAL
            v = v_base
            w = correction_w
            
            # Corrección si pierde el muro en esquina exterior
            if d_ps5 == float('inf') and d_ps6 == float('inf'):
                v = v_base * 0.6
                # Nativamente (Izquierda), girar a la IZQUIERDA (Positivo) para buscarlo
                w = self.angular_speed * 0.5 

        # --- EL TRUCO DE LA SIMETRÍA SPEJO ---
        # Multiplicamos la w nativa izquierda por el factor del lado.
        # Si seguimos por la IZQUIERDA: w_final = w * 1.0 (Sin cambios)
        # Si seguimos por la DERECHA: w_final = w * -1.0 (Invierte todos los giros)
        w_final = w * side_sign

        # --- CINEMÁTICA DIFERENCIAL (Asegurar que ROS 'cmd_vel' sea estándar) ---
        HALF_WHEEL_DIST = 0.026
        WHEEL_RADIUS = 0.02
        
        # Conversión de v, w_final a velocidades de ruedas
        left_wheel_speed = (v - w_final * HALF_WHEEL_DIST) / WHEEL_RADIUS
        right_wheel_speed = (v + w_final * HALF_WHEEL_DIST) / WHEEL_RADIUS

        # Limitar ruedas a 6.28 rad/s
        MAX_WHEEL_SPEED = 6.28
        left_wheel_speed = max(min(left_wheel_speed, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)
        right_wheel_speed = max(min(right_wheel_speed, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)

        # Volver a calcular velocidades lineales y angulares reales resultantes
        linear_speed_res = (left_wheel_speed + right_wheel_speed) * 0.5 * WHEEL_RADIUS
        angular_speed_res = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / (2.0 * HALF_WHEEL_DIST)

        # Publicar Twist final (Asegurar lineal siempre adelante en follow_wall)
        twist.linear.x = max(linear_speed_res, 0.0) 
        twist.angular.z = angular_speed_res
        
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
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()