import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DRONE_PROTOTYPE_2_MASS = 46/1000 #kg
DRONE_PROTOTYPE_0_MASS = 63/1000 #kg
BALLOON_MASS = .0545/2 #kg
m = DRONE_PROTOTYPE_2_MASS + BALLOON_MASS

fb = 9.802 #m/s^2
G = 9.81 #m/s^2
VOLTAGE_CONSTANT = .00714
U_ERROR = 3.0
MAX_VOLTAGE = 0.4
MAX_YAW_VOLTAGE = 0.4
MAX_SK_VOLTAGE = 0.05
Dz = 0.0480
Dwy = 0.000980
b = .000980

Fb = m*(G-fb)

H_ENV = 80/100 #m
H_GON = 4/100 #m
D_ENV = 81.28/2/100 #m

D_MT = H_ENV/2 + H_GON #distance between CM and gondola
D_VM = 0.0971 #m, found from paper, distance between CM and Center of lift

Icm = 0.005821/2 #Icm, should be roughly correct


def _reshape_vector(data, rows, cols, name):
    arr = np.array(data, dtype=float)
    if arr.size != rows * cols:
        raise ValueError(f"{name} size {arr.size} != {rows * cols}")
    return arr.reshape(rows, cols)


class RunSim(Node):
    def __init__(self):
        super().__init__("node")
        self.pos_publisher = self.create_publisher(Float32MultiArray, "/sim/robot_pos", 4)
        self.wrench_publisher = self.create_publisher(Float32MultiArray, "/sim/robot_wrench", 4)
        self.subscriber = self.create_subscription(
            Float32MultiArray, "/controls/control_update", self.update_control, 5
        )
        self.create_subscription(PointStamped, "/clicked_point", self.start, 1)

        self.declare_parameter("num_blimps", 2)
        self.num_blimps = int(self.get_parameter("num_blimps").value)

        self.declare_parameter("dt", 0.05)
        self.dt = float(self.get_parameter("dt").value)

        self.declare_parameter("map_size", [10.0, 10.0])
        self.map_size = self.get_parameter("map_size").value
        self.world_width = float(self.map_size[0])
        self.world_height = float(self.map_size[1])

        default_positions = np.array(
            [[float(i), 0.0, 0.0, 0.0, 0.0, 0.0] for i in range(self.num_blimps)],
            dtype=float,
        ).flatten()
        self.declare_parameter("blimp_positions", list(default_positions))
        initial_state = _reshape_vector(
            self.get_parameter("blimp_positions").value,
            self.num_blimps,
            6,
            "blimp_positions",
        )
        self.position_n = initial_state[:, :3].copy()
        self.euler_n = initial_state[:, 3:6].copy()  # roll, pitch, yaw

        # 6DOF dynamic state in body frame.
        self.linear_velocity_b = np.zeros((self.num_blimps, 3), dtype=float)
        self.angular_velocity_b = np.zeros((self.num_blimps, 3), dtype=float)

        self.declare_parameter("mass", m)
        self.mass = float(self.get_parameter("mass").value)
        self.inv_mass = 1.0 / max(self.mass, 1e-9)

        self.declare_parameter("inertia_diag", [Icm, Icm, Icm])
        self.inertia_diag = np.array(self.get_parameter("inertia_diag").value, dtype=float)
        self.inertia_diag = np.maximum(self.inertia_diag, 1e-9)
        self.inv_inertia_diag = 1.0 / self.inertia_diag

        self.declare_parameter("linear_drag", [Dz, Dz, Dz])
        self.linear_drag = np.array(self.get_parameter("linear_drag").value, dtype=float)
        self.declare_parameter("angular_drag", [b, b, Dwy])
        self.angular_drag = np.array(self.get_parameter("angular_drag").value, dtype=float)

        self.declare_parameter("gravity", G)
        self.gravity = float(self.get_parameter("gravity").value)
        # Use fb to represent buoyancy-equivalent upward acceleration.
        self.declare_parameter("buoyancy_force", self.mass * fb)
        self.buoyancy_force = float(self.get_parameter("buoyancy_force").value)
        self.declare_parameter("cg_offset_b", [0.0, 0.0, D_VM])
        self.cg_offset_b = np.array(self.get_parameter("cg_offset_b").value, dtype=float)

        # Motor order: [vertical_0, vertical_1, front_left, front_right, rear_right, rear_left]
        default_positions_b = [
            0.0, 0.03, 0.0,
            0.0, -0.03, 0.0,
            0.10, 0.10, -D_MT,
            0.10, -0.10, -D_MT,
            -0.10, -0.10, -D_MT,
            -0.10, 0.10, -D_MT,
        ]
        d = np.sqrt(0.5)
        default_dirs_b = [
            0.0, 0.0, -1.0,
            0.0, 0.0, -1.0,
            d, d, 0.0,
            d, -d, 0.0,
            -d, -d, 0.0,
            -d, d, 0.0,
        ]
        self.declare_parameter("motor_positions_b", default_positions_b)
        self.declare_parameter("motor_dirs_b", default_dirs_b)
        self.declare_parameter("motor_bidirectional", [True, True, False, False, False, False])
        self.declare_parameter("motor_voltage_max", [1.0] * 6)
        # Quadratic fit coefficients [a2, a1, a0] per motor.
        # Derived from controller conversion: thrust ~= (VOLTAGE_CONSTANT*100*|u|)^2.
        default_thrust_quad = (VOLTAGE_CONSTANT * 100.0) ** 2
        self.declare_parameter("motor_thrust_curve_coeffs", [default_thrust_quad, 0.0, 0.0] * 6)

        self.motor_positions_b = _reshape_vector(
            self.get_parameter("motor_positions_b").value, 6, 3, "motor_positions_b"
        )
        motor_dirs = _reshape_vector(
            self.get_parameter("motor_dirs_b").value, 6, 3, "motor_dirs_b"
        )
        norms = np.linalg.norm(motor_dirs, axis=1, keepdims=True)
        self.motor_dirs_b = motor_dirs / np.maximum(norms, 1e-9)
        self.motor_bidirectional = np.array(
            self.get_parameter("motor_bidirectional").value, dtype=bool
        )
        self.motor_voltage_max = np.array(
            self.get_parameter("motor_voltage_max").value, dtype=float
        )
        self.motor_thrust_curve_coeffs = _reshape_vector(
            self.get_parameter("motor_thrust_curve_coeffs").value,
            6,
            3,
            "motor_thrust_curve_coeffs",
        )

        self.control_lock = threading.Lock()
        self.control = np.zeros(6 * self.num_blimps, dtype=float)

        self.thread = threading.Thread(target=self.run_sim, daemon=True)
        self.thread_started = False
        self.last_debug_t = 0.0

    def start(self, _msg):
        if self.thread_started:
            return
        self.thread_started = True
        self.thread.start()

    def update_control(self, msg):
        with self.control_lock:
            self.control = np.array(msg.data, dtype=float)

    def _rotation_matrix(self, roll, pitch, yaw):
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        return np.array(
            [
                [cy * cp, -sy * cr + cy * sp * sr, sy * sr + cy * sp * cr],
                [sy * cp, cy * cr + sy * sp * sr, -cy * sr + sy * sp * cr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )

    def _euler_rate_matrix(self, roll, pitch):
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        cp_safe = cp if abs(cp) > 1e-3 else (1e-3 if cp >= 0 else -1e-3)
        tp = np.sin(pitch) / cp_safe
        return np.array(
            [
                [1.0, sr * tp, cr * tp],
                [0.0, cr, -sr],
                [0.0, sr / cp_safe, cr / cp_safe],
            ],
            dtype=float,
        )

    def _thrust_from_voltage(self, v_cmd, motor_idx):
        vmax = max(float(self.motor_voltage_max[motor_idx]), 1e-6)
        v_limited = float(np.clip(v_cmd, -vmax, vmax))
        if not self.motor_bidirectional[motor_idx]:
            v_limited = max(0.0, v_limited)
        coeffs = self.motor_thrust_curve_coeffs[motor_idx]
        mag = abs(v_limited)
        thrust_mag = coeffs[0] * mag * mag + coeffs[1] * mag + coeffs[2]
        thrust_mag = max(0.0, thrust_mag)
        if self.motor_bidirectional[motor_idx]:
            return np.sign(v_limited) * thrust_mag
        return thrust_mag

    def _compute_motor_wrench(self, voltages):
        thrusts = np.array(
            [self._thrust_from_voltage(v, i) for i, v in enumerate(voltages)],
            dtype=float,
        )
        motor_forces = thrusts[:, None] * self.motor_dirs_b
        f_motor = np.sum(motor_forces, axis=0)
        tau_motor = np.sum(np.cross(self.motor_positions_b, motor_forces), axis=0)
        return thrusts, f_motor, tau_motor

    def _apply_world_bounds(self, idx):
        x, y, z = self.position_n[idx]
        clamped = False

        x_new = min(max(x, 0.0), self.world_width)
        y_new = min(max(y, 0.0), self.world_width)
        z_new = min(max(z, 0.0), self.world_height)
        if x_new != x or y_new != y or z_new != z:
            clamped = True
        self.position_n[idx] = np.array([x_new, y_new, z_new], dtype=float)

        # Add damping when near boundaries to avoid integration bounce.
        margin = 0.1
        near_bound = (
            x_new < margin
            or y_new < margin
            or z_new < margin
            or (self.world_width - x_new) < margin
            or (self.world_width - y_new) < margin
            or (self.world_height - z_new) < margin
        )
        if clamped:
            self.linear_velocity_b[idx] *= 0.5
            self.angular_velocity_b[idx] *= 0.8
        elif near_bound:
            self.linear_velocity_b[idx] *= 0.95

    def _publish(self, wrench_packets):
        pose_msg = Float32MultiArray()
        pose_payload = np.hstack([self.position_n, self.euler_n]).flatten()
        pose_msg.data = list(pose_payload.astype(float))
        self.pos_publisher.publish(pose_msg)

        wrench_msg = Float32MultiArray()
        wrench_msg.data = list(np.array(wrench_packets, dtype=float).flatten())
        self.wrench_publisher.publish(wrench_msg)

    def run_sim(self):
        while True:
            dt = self.dt
            if dt <= 0.0 or dt >= 0.1:
                time.sleep(max(self.dt, 0.01))
                continue

            with self.control_lock:
                control_now = self.control.copy()
            expected = 6 * self.num_blimps
            if control_now.size != expected:
                self.get_logger().warn(
                    f"Expected {expected} control values (6 per blimp), got {control_now.size}. Skipping step."
                )
                time.sleep(self.dt)
                continue

            wrench_packets = []
            for blimp_idx in range(self.num_blimps):
                i0 = 6 * blimp_idx
                voltages = control_now[i0 : i0 + 6]
                thrusts, f_motor, tau_motor = self._compute_motor_wrench(voltages)

                roll, pitch, yaw = self.euler_n[blimp_idx]
                v_b = self.linear_velocity_b[blimp_idx]
                omega_b = self.angular_velocity_b[blimp_idx]

                r_nb = self._rotation_matrix(roll, pitch, yaw)
                gravity_n = np.array([0.0, 0.0, self.gravity], dtype=float)
                buoyancy_n = np.array([0.0, 0.0, -self.buoyancy_force], dtype=float)
                gravity_b = r_nb.T @ (self.mass * gravity_n)
                buoyancy_b = r_nb.T @ buoyancy_n

                f_drag = -self.linear_drag * v_b
                m_drag = -self.angular_drag * omega_b
                m_restoring = np.cross(self.cg_offset_b, gravity_b)

                f_env = f_drag + gravity_b + buoyancy_b
                m_env = m_drag + m_restoring

                # 6DOF Newton-Euler in body frame.
                v_dot = self.inv_mass * (f_env + f_motor) - np.cross(omega_b, v_b)
                i_omega = self.inertia_diag * omega_b
                omega_dot = self.inv_inertia_diag * (
                    m_env + tau_motor - np.cross(omega_b, i_omega)
                )

                self.linear_velocity_b[blimp_idx] = v_b + v_dot * dt
                self.angular_velocity_b[blimp_idx] = omega_b + omega_dot * dt

                pos_dot_n = r_nb @ self.linear_velocity_b[blimp_idx]
                euler_dot = self._euler_rate_matrix(roll, pitch) @ self.angular_velocity_b[blimp_idx]

                self.position_n[blimp_idx] += pos_dot_n * dt
                self.euler_n[blimp_idx] += euler_dot * dt
                self.euler_n[blimp_idx] = (self.euler_n[blimp_idx] + np.pi) % (2.0 * np.pi) - np.pi

                self._apply_world_bounds(blimp_idx)
                
                packet = np.concatenate(
                    [
                        thrusts,
                        f_motor,
                        tau_motor,
                        self.linear_velocity_b[blimp_idx],
                        self.angular_velocity_b[blimp_idx],
                    ]
                )
                wrench_packets.append(packet)

            now_t = self.get_clock().now().nanoseconds / 1e9
            if now_t - self.last_debug_t > 1.0 and self.num_blimps > 0:
                self.last_debug_t = now_t
                sample = wrench_packets[0]
                self.get_logger().info(
                    "sim[0] v_in="
                    f"{np.round(control_now[:6], 3).tolist()} thrust="
                    f"{np.round(sample[:6], 4).tolist()} F="
                    f"{np.round(sample[6:9], 4).tolist()} tau="
                    f"{np.round(sample[9:12], 4).tolist()}"
                )

            self._publish(wrench_packets)
            time.sleep(self.dt)


def main(args=None):
    rclpy.init(args=args)
    sim = RunSim()
    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    sim.thread.join(timeout=2.0)
    sim.destroy_node()
    rclpy.shutdown()

