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
        self.declare_parameter('linear_speed', 0.55)
        self.declare_parameter('angular_speed', 15.0)
        self.declare_parameter('goal_x', 1.45)
        self.declare_parameter('goal_y', 1.20)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.25)
        self.declare_parameter('obstacle_distance', 0.80) # Umbral frontal (40 cm)
        self.declare_parameter('wall_dist_target', 0.30)   # Distancia deseada a la pared derecha (35 cm)
        self.declare_parameter('front_angle', 70.0)
        # Control parameters for wall-following (PID tunable)
        # PID defaults adjusted: balance between responsiveness and damping
        self.declare_parameter('kp_wall', 2.0)
        self.declare_parameter('ki_wall', 0.0)
        self.declare_parameter('kd_wall', 5.5)
        self.declare_parameter('pid_integral_limit', 0.1)
        # derivative smoothing (0-1), and angular smoothing for output
        # lower pid_deriv_alpha -> derivative reacts faster
        self.declare_parameter('pid_deriv_alpha', 0.12)
        # increase angular smoothing alpha -> smoother angular output to reduce overshoot
        self.declare_parameter('angular_smooth_alpha', 0.5)
        # scale PID output (slightly reduced)
        self.declare_parameter('pid_output_scale', 0.8)
        # Parameters for obstacle detection smoothing and suppression
        self.declare_parameter('obstacle_detection_count', 3)
        self.declare_parameter('wall_exit_suppression', 1.0)
        self.declare_parameter('wall_exit_confirm_count', 3)
        # tighten target to follow closer to wall
        self.declare_parameter('wall_follow_tighten', 0.75)
        self.declare_parameter('wall_exit_distance_epsilon', 0.05)
        self.declare_parameter('wall_max_follow_distance', 0.3)
        self.declare_parameter('wall_side_memory_time', 5.0)
        self.declare_parameter('wall_side_exit_distance_limit', 1.0)
        self.declare_parameter('wall_default_side', 'LEFT')
        self.declare_parameter('goal_reached_confirm_count', 10)
        self.declare_parameter('goal_reached_min_time_sec', 1.0)
        
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
        self.wall_follow_side = 'RIGHT'
        # PID state for wall-following controller
        self.kp_wall = float(self.get_parameter('kp_wall').value)
        self.ki_wall = float(self.get_parameter('ki_wall').value)
        self.kd_wall = float(self.get_parameter('kd_wall').value)
        self.pid_integral_limit = float(self.get_parameter('pid_integral_limit').value)
        self.pid_deriv_alpha = float(self.get_parameter('pid_deriv_alpha').value)
        self.angular_smooth_alpha = float(self.get_parameter('angular_smooth_alpha').value)
        self.pid_output_scale = float(self.get_parameter('pid_output_scale').value)
        self.wall_pid_integral = 0.0
        self.wall_pid_prev_error = 0.0
        self.wall_pid_prev_time = None
        self.wall_pid_derivative = 0.0
        self.prev_angular = 0.0
        # Corner handling state to avoid abrupt 90-deg turns
        self.wall_corner_phase = 'NONE'  # NONE -> TURN1 -> ADVANCE -> TURN2
        self.wall_corner_cycles = 0
        # smoothing for obstacle detection
        self.obstacle_ahead_count = 0
        self.last_wall_exit_time = -1e9
        # confirmation counter to avoid oscillatory exits from wall following
        self.wall_exit_confirm_count = int(self.get_parameter('wall_exit_confirm_count').value)
        self.wall_exit_counter = 0
        self.wall_follow_tighten = float(self.get_parameter('wall_follow_tighten').value)
        self.wall_exit_distance_epsilon = float(self.get_parameter('wall_exit_distance_epsilon').value)
        self.wall_max_follow_distance = float(self.get_parameter('wall_max_follow_distance').value)
        self.wall_side_memory_time = float(self.get_parameter('wall_side_memory_time').value)
        self.wall_side_memory_until = -1e9
        self.wall_side_exit_distance_limit = float(self.get_parameter('wall_side_exit_distance_limit').value)
        self.wall_default_side = str(self.get_parameter('wall_default_side').value).upper()
        self.goal_reached_confirm_count = int(self.get_parameter('goal_reached_confirm_count').value)
        self.goal_reached_min_time_sec = float(self.get_parameter('goal_reached_min_time_sec').value)
        self.goal_reached_counter = 0
        self.last_goal_change_time = float(self.get_clock().now().nanoseconds) * 1e-9

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
        self.goal_reached_counter = 0
        self.last_goal_change_time = float(self.get_clock().now().nanoseconds) * 1e-9
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
        # Require both position and heading to be close, and keep that condition
        # stable for a few cycles. This avoids false goal hits from EKF jumps.
        goal_close = (distance < self.goal_tolerance) and (abs(alpha) < self.yaw_tolerance)
        now = self.get_clock().now().nanoseconds / 1e9

        if goal_close and (now - self.last_goal_change_time) >= self.goal_reached_min_time_sec:
            self.goal_reached_counter = min(self.goal_reached_confirm_count, self.goal_reached_counter + 1)
        else:
            self.goal_reached_counter = 0

        if self.goal_reached_counter >= self.goal_reached_confirm_count:
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

        # El wall follower solo debe ser una evasión temporal.
        # Salimos de WALL_FOLLOWING solo si el frente está libre Y el camino hacia la meta
        # está despejado de forma sostenida (evita oscilaciones al intentar girar hacia meta)
        path_clear = False
        try:
            path_clear = self.is_path_to_goal_clear(heading)
        except Exception:
            path_clear = False

        if self.state == 'WALL_FOLLOWING':
            # Only exit wall-following if front is clear AND we have progressed
            # closer to the goal than when we first hit the obstacle.
            progressed = False
            try:
                progressed = distance < (self.hit_distance - self.wall_exit_distance_epsilon)
            except Exception:
                progressed = False

            # Default: require front free, path_clear and some progress
            exit_condition = (not obstacle_ahead) and path_clear and progressed

            # Additional rule: if we are following RIGHT wall but the goal lies to the LEFT
            # and there are no obstacles on the LEFT, allow exit (and viceversa).
            # This only applies when the goal is reasonably close (configurable limit).
            try:
                goal_rel = self.normalize_angle(heading - self.theta)
                if distance < self.wall_side_exit_distance_limit:
                    # check left clearance
                    left_dist = self._sector_min_range(90.0, 60.0)
                    left_clear = (not math.isfinite(left_dist)) or (left_dist > self.obstacle_distance)
                    # check right clearance
                    right_dist = self._sector_min_range(-90.0, 60.0)
                    right_clear = (not math.isfinite(right_dist)) or (right_dist > self.obstacle_distance)

                    if self.wall_follow_side == 'RIGHT' and goal_rel > 0.0 and left_clear:
                        self.get_logger().info('[BUG 0] Goal está a la izquierda y lado izquierdo libre -> permitiendo salida de WALL_FOLLOWING')
                        exit_condition = True
                    if self.wall_follow_side == 'LEFT' and goal_rel < 0.0 and right_clear:
                        self.get_logger().info('[BUG 0] Goal está a la derecha y lado derecho libre -> permitiendo salida de WALL_FOLLOWING')
                        exit_condition = True
            except Exception:
                pass

            if exit_condition:
                # confirmar varias veces para evitar oscilaciones
                self.wall_exit_counter = min(self.wall_exit_confirm_count, self.wall_exit_counter + 1)
            else:
                self.wall_exit_counter = 0

            if self.wall_exit_counter >= self.wall_exit_confirm_count:
                self.get_logger().info('[BUG 0] Frente despejado sostenidamente y progreso detectado. Retomando seguimiento de setpoint.')
                self.state = 'GIRAR_HACIA_META'
                self.hit_distance = float('inf')
                self.last_wall_exit_time = now
                self.wall_exit_counter = 0

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
            cmd = self.follow_wall(self.wall_follow_side)

        self.cmd_pub.publish(cmd)

    def is_obstacle_ahead(self) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment
        # Cono frontal adaptado dinámicamente según 'front_angle'
        angle_min = float(self.latest_scan.angle_min)
        half_fwd = max(1, int(math.radians(self.front_angle / 2.0) / inc))

        # Calcular índice central que corresponde a 0 rad (frente del robot)
        center_idx = int(round((0.0 - angle_min) / inc)) % n
        # Obtener los puntos frontales (considerando desbordamiento del arreglo circular)
        idxs = [(center_idx + i) % n for i in range(-half_fwd, half_fwd + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)] # Limpieza de ruidos e infinitos

        return bool(np.any(vals < self.obstacle_distance))

    def _sector_min_range(self, center_deg: float, half_width_deg: float) -> float:
        if self.latest_scan is None:
            return float('inf')

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment
        angle_min = float(self.latest_scan.angle_min)
        center_rad = math.radians(center_deg)
        center_idx = int(round((center_rad - angle_min) / inc)) % n
        half_width = max(1, int(math.radians(half_width_deg) / inc))
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return float('inf')
        return float(np.min(vals))

    def select_wall_side(self) -> str:
        # Use wider sectors to determine which side has the nearest obstacle.
        # Left: approx 45..135 deg, Right: approx -135..-45 deg
        now = self.get_clock().now().nanoseconds / 1e9
        left_dist = self._sector_min_range(90.0, 60.0)
        right_dist = self._sector_min_range(-90.0, 60.0)
        front_dist = self._sector_min_range(0.0, 35.0)

        if not math.isfinite(left_dist) and not math.isfinite(right_dist):
            # Only keep the previous side for a short time after losing both walls.
            # After the timeout, drop the memory and fall back to the configured default side.
            if now <= self.wall_side_memory_until:
                return self.wall_follow_side
            return self.wall_default_side

        # If both walls are visible and the scene is almost symmetric, keep the
        # current wall only for a short time; otherwise pick the closest one.
        if math.isfinite(front_dist) and abs(left_dist - right_dist) < 0.20 and front_dist < self.obstacle_distance:
            self.wall_side_memory_until = now + self.wall_side_memory_time
            return 'LEFT'

        if math.isfinite(left_dist) and not math.isfinite(right_dist):
            self.wall_side_memory_until = now + self.wall_side_memory_time
            return 'LEFT'

        if math.isfinite(right_dist) and not math.isfinite(left_dist):
            self.wall_side_memory_until = now + self.wall_side_memory_time
            return 'RIGHT'

        if left_dist <= right_dist:
            self.wall_side_memory_until = now + self.wall_side_memory_time
            return 'LEFT'
        self.wall_side_memory_until = now + self.wall_side_memory_time
        return 'RIGHT'

    def is_path_to_goal_clear(self, heading: float) -> bool:
        if self.latest_scan is None:
            return False

        ranges = np.array(self.latest_scan.ranges, dtype=float)
        n = len(ranges)
        inc = self.latest_scan.angle_increment
        # Detectar el índice del LiDAR que apunta directamente hacia el ángulo de la meta
        angle_min = float(self.latest_scan.angle_min)
        goal_rad_rel = self.normalize_angle(heading - self.theta)
        idx_goal = int(round((goal_rad_rel - angle_min) / inc)) % n

        # Analizar un cono de tolerancia de 20 grados apuntando a la meta
        half_width = max(1, int(math.radians(20.0 / 2.0) / inc))
        idxs = [(idx_goal + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.02) & np.isfinite(vals)]

        if len(vals) == 0:
            return True
        return float(np.min(vals)) > self.obstacle_distance

    def follow_wall(self, wall_side: str = 'LEFT') -> Twist:
        twist = Twist()

        if self.latest_scan is None:
            return twist

        # Determine which side to follow if caller passed 'NEAREST' or no preference
        if wall_side is None or wall_side.upper() == 'NEAREST':
            wall_side = self.select_wall_side()

        # Choose sector for the selected side and measure distance
        if wall_side == 'LEFT':
            side_center = 90.0
            sign = 1.0
        else:
            side_center = -90.0
            sign = -1.0

        measured = self._sector_min_range(side_center, 60.0)
        if not math.isfinite(measured):
            measured = float('inf')

        # Front clearance
        front_min = self._sector_min_range(0.0, 35.0)

        # Simple threshold steering (no PID): ensure robot doesn't drift farther
        # than `wall_max_follow_distance` from the wall. If too far -> steer
        # towards wall; if too close -> steer away; otherwise go straight.
        active_target = max(0.01, self.wall_dist_target * self.wall_follow_tighten)
        min_allowed = max(0.01, active_target * 0.6)
        max_allowed = self.wall_max_follow_distance

        front_close = math.isfinite(front_min) and front_min < self.obstacle_distance * 0.35

        if front_close:
            # Corner handling sequence: first 45 deg turn, short advance, second 45 deg turn.
            # This avoids trying to do the whole 90 deg at once.
            if self.wall_corner_phase == 'NONE':
                self.wall_corner_phase = 'TURN1'
                self.wall_corner_cycles = 0

            self.wall_corner_cycles += 1

            if self.wall_corner_phase == 'TURN1':
                linear = 0.0
                angular = sign * (self.angular_speed * 0.50)
                if self.wall_corner_cycles >= 3:
                    self.wall_corner_phase = 'ADVANCE'
                    self.wall_corner_cycles = 0
            elif self.wall_corner_phase == 'ADVANCE':
                linear = self.linear_speed * 0.08
                angular = sign * (self.angular_speed * 0.20)
                if self.wall_corner_cycles >= 4:
                    self.wall_corner_phase = 'TURN2'
                    self.wall_corner_cycles = 0
            else:
                linear = 0.0
                angular = sign * (self.angular_speed * 0.90)
                if self.wall_corner_cycles >= 3:
                    self.wall_corner_phase = 'NONE'
                    self.wall_corner_cycles = 0

            # Reset PID memory so it does not fight the cornering turn.
            self.wall_pid_integral = 0.0
            self.wall_pid_derivative = 0.0
            self.wall_pid_prev_error = 0.0
            self.wall_pid_prev_time = self.get_clock().now().nanoseconds / 1e9
            self.prev_angular = angular
        else:
            # Once the front is clear, leave the cornering mode and resume distance PID.
            self.wall_corner_phase = 'NONE'
            self.wall_corner_cycles = 0

            # PID controller to compute angular velocity to keep measured close to active_target
            # Error defined as (measured - active_target); for LEFT sign=1, RIGHT sign=-1
            error = (measured - active_target)
            now_t = self.get_clock().now().nanoseconds / 1e9
            if self.wall_pid_prev_time is None:
                dt = 1.0 / max(1.0, self.publish_rate)
            else:
                dt = max(1e-3, now_t - self.wall_pid_prev_time)

            # Integrator with anti-windup
            self.wall_pid_integral += error * dt
            # clamp integral
            if self.wall_pid_integral > self.pid_integral_limit:
                self.wall_pid_integral = self.pid_integral_limit
            elif self.wall_pid_integral < -self.pid_integral_limit:
                self.wall_pid_integral = -self.pid_integral_limit

            # Derivative (raw)
            derivative_raw = 0.0
            if self.wall_pid_prev_time is not None:
                derivative_raw = (error - self.wall_pid_prev_error) / dt

            # low-pass filter the derivative to reduce noise amplification
            self.wall_pid_derivative = (self.pid_deriv_alpha * derivative_raw) + ((1.0 - self.pid_deriv_alpha) * self.wall_pid_derivative)

            pid_out = (self.kp_wall * error) + (self.ki_wall * self.wall_pid_integral) + (self.kd_wall * self.wall_pid_derivative)

            # scale down PID output to avoid large jumps and apply side sign
            pid_out *= self.pid_output_scale
            angular_cmd = sign * pid_out

            # smooth angular command to avoid step changes
            angular = (self.prev_angular * (1.0 - self.angular_smooth_alpha)) + (angular_cmd * self.angular_smooth_alpha)

            # Clamp angular to node-configured angular_speed
            angular = max(min(angular, self.angular_speed), -self.angular_speed)

            # Save PID state
            self.wall_pid_prev_error = error
            self.wall_pid_prev_time = now_t
            self.prev_angular = angular

            # Keep a moderate forward motion while aligning to wall distance.
            linear = self.linear_speed * 0.38

        self.get_logger().debug(
            f"follow_wall side={wall_side} measured={measured:.3f} min_allowed={min_allowed:.3f} max_allowed={max_allowed:.3f} -> lin={linear:.3f} ang={angular:.3f}"
        )

        twist.linear.x = linear
        twist.angular.z = angular
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