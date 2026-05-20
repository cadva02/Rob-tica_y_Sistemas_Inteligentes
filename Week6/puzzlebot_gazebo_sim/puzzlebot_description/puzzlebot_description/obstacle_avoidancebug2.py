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
        self.declare_parameter('obstacle_distance', 0.40) # Umbral frontal (40 cm)
        self.declare_parameter('wall_dist_target', 0.35)   # Distancia deseada a la pared derecha (35 cm)
        self.declare_parameter('front_angle', 40.0)
        
        # Parámetro extra para el umbral de cruce con la M-line
        self.declare_parameter('m_line_tolerance', 0.10)   # 10 cm de tolerancia para re-interceptar la línea
        
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
        self.m_line_tolerance = float(self.get_parameter('m_line_tolerance').value)
        
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # --- ESTADO INTERNO DEL ROBOT ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.have_odom = False
        self.have_goal = False  # Cambiado a False para esperar a guardar el start_x inicial en el odom_callback
        self.latest_scan = None
        self.goal_reached = False
        
        # Puntos de origen para calcular la M-Line (se fijan al recibir odom/meta)
        self.start_x = 0.0
        self.start_y = 0.0
        self.m_line_initialized = False
        
        # Estados Bug 2: 'GIRAR_HACIA_META', 'AVANZAR_A_META', 'WALL_FOLLOWING'
        self.state = 'GIRAR_HACIA_META'
        self.hit_distance = float('inf')

        # --- PUBLICADORES Y SUSCRIPTORES ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Use relative topic name so it matches other nodes/subscriptions
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

    def goal_callback(self, msg: Vector3) -> None:
        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.goal_theta = float(msg.z)
        
        # Forzar a recalcular la M-line con la posición actual del robot
        self.start_x = self.x
        self.start_y = self.y
        self.m_line_initialized = True
        
        self.have_goal = True
        self.goal_reached = False
        self.state = 'GIRAR_HACIA_META'
        self.hit_distance = float('inf')
        self.get_logger().info(f'Nueva meta recibida: x={self.goal_x:.2f}, y={self.goal_y:.2f}. M-Line recalculada.')

    def odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Inicializar el punto de partida la primera vez si no se ha recibido un goal por tópico
        if not self.m_line_initialized:
            self.start_x = self.x
            self.start_y = self.y
            self.m_line_initialized = True
            self.have_goal = True # Permite arrancar usando los parámetros iniciales del launch/nodo

        # Conversión de Cuaternión a Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def is_on_m_line(self) -> bool:
        """
        Calcula la distancia perpendicular desde la posición actual del robot 
        a la recta (M-line) definida por (start_x, start_y) y (goal_x, goal_y).
        """
        # Ecuación de la recta: AB x AC (producto cruz modificado para distancia en 2D)
        # Linea de A(start) a B(goal). Punto actual C(x, y)
        num = abs((self.goal_y - self.start_y) * self.x - (self.goal_x - self.start_x) * self.y + self.goal_x * self.start_y - self.goal_y * self.start_x)
        den = math.hypot(self.goal_y - self.start_y, self.goal_x - self.start_x)
        
        if den == 0.0:
            return False
            
        distance_to_line = num / den
        return distance_to_line < self.m_line_tolerance

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
        obstacle_ahead = self.is_obstacle_ahead()

        # --- MÁQUINA DE ESTADOS BUG 2 ---
        if self.state == 'GIRAR_HACIA_META':
            if obstacle_ahead:
                self.hit_distance = distance
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 2] Obstáculo detectado al girar. Pasando a WALL_FOLLOWING (hit_dist={distance:.2f}m).')
                cmd = self.follow_wall()
            else:
                if abs(alpha) > 0.15:
                    cmd.linear.x = 0.0
                    cmd.angular.z = np.sign(alpha) * self.angular_speed
                else:
                    self.state = 'AVANZAR_A_META'

        elif self.state == 'AVANZAR_A_META':
            if obstacle_ahead:
                self.hit_distance = distance
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 2] Obstáculo detectado en M-line. Siguiendo pared (hit_dist={distance:.2f}m).')
                cmd = self.follow_wall()
            else:
                if abs(alpha) > 0.30:
                    self.state = 'GIRAR_HACIA_META'
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0

        elif self.state == 'WALL_FOLLOWING':
            # Evaluar las condiciones estrictas de escape de Bug 2:
            # 1. Estar en la M-line nuevamente.
            # 2. Estar notablemente más cerca de la meta que la distancia de impacto original (hit_distance).
            # 3. Que el frente o camino inmediato hacia la meta no esté bloqueado para evitar bucles instantáneos.
            on_m_line = self.is_on_m_line()
            path_clear = self.is_path_to_goal_clear(heading)
            
            if on_m_line and (distance < (self.hit_distance - 0.20)) and not obstacle_ahead and path_clear:
                self.get_logger().info('[BUG 2] Cruce de M-line válido y más cercano detectado. Escapando de la pared.')
                self.state = 'GIRAR_HACIA_META'
                self.hit_distance = float('inf')
                
                # Romper inercia limpiamente deteniendo el robot para reorientarse
                cmd.linear.x = 0.0
                cmd.angular.z = np.sign(alpha) * self.angular_speed
            else:
                cmd = self.follow_wall()

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

    def follow_wall(self) -> Twist:
        twist = Twist()
        
        if self.latest_scan is None:
            return twist

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment

        # Segmentar sectores precisos: Frente y Costado Derecho (-90 grados exactos)
        half_fwd = max(1, int(math.radians(self.front_angle / 2.0) / inc))
        half_side = max(1, int(math.radians(30.0 / 2.0) / inc))
        idx_right = int(round(math.radians(-90.0) / inc)) % n

        # Calcular distancias mínimas reales filtradas
        idxs_fwd = [(0 + i) % n for i in range(-half_fwd, half_fwd + 1)]
        vals_fwd = ranges[idxs_fwd]
        vals_fwd = vals_fwd[(vals_fwd > 0.02) & np.isfinite(vals_fwd)]
        scan_front = float(np.min(vals_fwd)) if len(vals_fwd) > 0 else float('inf')

        idxs_right = [(idx_right + i) % n for i in range(-half_side, half_side + 1)]
        vals_right = ranges[idxs_right]
        vals_right = vals_right[(vals_right > 0.02) & np.isfinite(vals_right)]
        scan_right = float(np.min(vals_right)) if len(vals_right) > 0 else float('inf')

        # COMPORTAMIENTO REACTIVO DE SEGUIMIENTO (Flanco Derecho)
        frente_libre = scan_front > self.obstacle_distance

        if not frente_libre:
            # Muro directo al frente: Detener avance lineal y pivotar rápido a la izquierda
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            # Frente despejado: Avanza y corrige trayectoria con histéresis discreta
            twist.linear.x = self.linear_speed * 0.8  # Reducción controlada en curvas
            
            if scan_right > (self.wall_dist_target + 0.06):
                # Se aleja de la pared derecha -> Cerrarse a la derecha (-)
                twist.angular.z = -self.angular_speed * 0.5
            elif scan_right < (self.wall_dist_target - 0.06):
                # Se pega demasiado a la pared derecha -> Abrirse a la izquierda (+)
                twist.angular.z = self.angular_speed * 0.5
            else:
                # En margen ideal de costeo
                twist.angular.z = 0.0

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