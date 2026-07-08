import math
import numpy as np
from typing import List, Tuple, Optional

class RocketPlant:
    def __init__(self, scenario: dict):
        self.mass_dry = scenario.get("mass_dry_kg", 8.0)
        self.mass_propellant = scenario.get("mass_propellant_kg", 2.0)
        self.mass = self.mass_dry + self.mass_propellant
        self.cg_offset = scenario.get("cg_offset_m", 0.0)

        moi = scenario.get("moi", {})
        self.Ixx = moi.get("Ixx", 0.05)
        self.Iyy = moi.get("Iyy", 0.8)
        self.Izz = moi.get("Izz", 0.8)

        aero = scenario.get("aero", {})
        self.Cd0 = aero.get("Cd0", 0.45)
        self.Cl_alpha = aero.get("Cl_alpha", 2.5)
        self.Cl_delta = aero.get("Cl_delta", 1.8)
        self.Cm_alpha = aero.get("Cm_alpha", -2.0)
        self.Cn_delta = aero.get("Cn_delta", 1.5)
        self.ref_area = aero.get("ref_area_m2", 0.005)
        self.ref_length = aero.get("ref_length_m", 0.08)
        self.x_cp_minus_cg = aero.get("x_cp_minus_cg_m", 0.15)

        motor = scenario.get("motor", {})
        self.thrust_profile = motor.get("thrust_profile_n", [(0.0, 0.0), (0.5, 500.0), (2.0, 500.0), (2.5, 0.0)])
        self.burn_time = motor.get("burn_time_s", 2.5)
        self.propellant_flow_rate = self.mass_propellant / self.burn_time if self.burn_time > 0 else 0.0

        baro = scenario.get("baro", {})
        self.baro_noise_std = baro.get("noise_std_m", 0.5)
        self.baro_bias = baro.get("bias_m", 0.0)

        accel_noise = scenario.get("accel_noise_std", 0.05)
        self.accel_noise_std = accel_noise

        self.g = scenario.get("gravity_m_s2", 9.80665)
        self.rho0 = scenario.get("air_density_kg_m3", 1.225)
        self.scale_height = scenario.get("scale_height_m", 8500.0)

        self.wind_schedule = scenario.get("wind_schedule", [])

        self.dt = 0.01
        self.time = 0.0
        self.altitude = 0.0
        self.velocity_ned = np.zeros(3)
        self.position_ned = np.zeros(3)
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity_body = np.zeros(3)
        self.thrust_current = 0.0
        self.mach = 0.0
        self.airspeed = 0.0
        self.density = self.rho0

        self.wind_ned = np.zeros(3)

        self._log: List[dict] = []

    @staticmethod
    def quat_mult(q1, q2):
        w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
        w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ])

    @staticmethod
    def quat_normalize(q):
        n = np.linalg.norm(q)
        if n < 1e-10:
            return np.array([0.0, 0.0, 0.0, 1.0])
        return q / n

    @staticmethod
    def quat_to_dcm(q):
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        return np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy + qw*qz), 2*(qx*qz - qw*qy)],
            [2*(qx*qy - qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz + qw*qx)],
            [2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx**2 + qy**2)],
        ])

    @staticmethod
    def quat_to_euler(q):
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        sinp = 2*(qw*qy - qz*qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        return roll, pitch, yaw

    def dcm_to_quat(self, R):
        tr = np.trace(R)
        if tr > 0:
            s = 0.5 / math.sqrt(tr + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return self.quat_normalize(np.array([qx, qy, qz, qw]))

    def get_thrust(self, t: float) -> float:
        if not self.thrust_profile:
            return 0.0
        for i in range(len(self.thrust_profile) - 1):
            t0, f0 = self.thrust_profile[i]
            t1, f1 = self.thrust_profile[i + 1]
            if t0 <= t <= t1:
                frac = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return f0 + frac * (f1 - f0)
        return 0.0

    def get_wind(self, t: float) -> np.ndarray:
        w = np.zeros(3)
        for entry in self.wind_schedule:
            t_start = entry.get("time_s", 0.0)
            t_end = entry.get("time_end_s", t_start + 1.0)
            if t_start <= t <= t_end:
                w[0] += entry.get("wind_x_m_s", 0.0)
                w[1] += entry.get("wind_y_m_s", 0.0)
                w[2] += entry.get("wind_z_m_s", 0.0)
        return w

    def air_density(self, alt: float) -> float:
        return self.rho0 * math.exp(-max(alt, 0.0) / self.scale_height)

    def step(self, servo_deflections_deg: np.ndarray, dt: float = 0.01) -> dict:
        self.dt = dt
        self.time += dt
        self.wind_ned = self.get_wind(self.time)
        self.density = self.air_density(self.altitude)

        dcm = self.quat_to_dcm(self.quaternion)
        R_body_to_ned = dcm

        self.thrust_current = self.get_thrust(self.time)
        if self.thrust_current > 0 and self.mass_propellant > 0:
            dm = self.propellant_flow_rate * dt
            self.mass_propellant = max(0.0, self.mass_propellant - dm)
            self.mass = self.mass_dry + self.mass_propellant

        thrust_body = np.array([0.0, 0.0, -self.thrust_current])
        thrust_ned = R_body_to_ned @ thrust_body

        vel_body = R_body_to_ned.T @ (self.velocity_ned - self.wind_ned)
        self.airspeed = np.linalg.norm(vel_body)
        sound_speed = 343.0
        self.mach = self.airspeed / sound_speed

        q_dyn = 0.5 * self.density * self.airspeed ** 2

        alpha_body = 0.0
        if abs(vel_body[2]) > 0.1:
            alpha_body = math.atan2(-vel_body[0], vel_body[2])

        F_aero_body = np.zeros(3)
        F_aero_body[0] = -q_dyn * self.ref_area * self.Cd0 * vel_body[0] / max(self.airspeed, 0.01)
        F_aero_body[1] = -q_dyn * self.ref_area * self.Cd0 * vel_body[1] / max(self.airspeed, 0.01)
        F_aero_body[2] = -q_dyn * self.ref_area * self.Cd0 * vel_body[2] / max(self.airspeed, 0.01)

        F_aero_body[0] += q_dyn * self.ref_area * self.Cl_alpha * alpha_body

        F_aero_ned = R_body_to_ned @ F_aero_body

        gravity_ned = np.array([0.0, 0.0, self.mass * self.g])

        F_total_ned = thrust_ned + F_aero_ned + gravity_ned
        accel_ned = F_total_ned / self.mass

        non_grav_ned = thrust_ned + F_aero_ned
        specific_force_ned = non_grav_ned / self.mass

        accel_body = R_body_to_ned.T @ accel_ned
        specific_force_body = R_body_to_ned.T @ specific_force_ned

        M_body = np.zeros(3)
        delta_pitch = 0.0
        delta_yaw = 0.0
        delta_roll = 0.0

        surface_map = [
            (0, 'pitch', +1.0),
            (1, 'pitch', -1.0),
            (2, 'yaw',   +1.0),
            (3, 'yaw',   -1.0),
            (4, 'roll',  +1.0),
            (5, 'roll',  -1.0),
            (6, 'roll',  +1.0),
            (7, 'roll',  -1.0),
        ]

        for idx, axis, sign in surface_map:
            defl_rad = math.radians(servo_deflections_deg[idx] - 90.0)
            if axis == 'pitch':
                delta_pitch += sign * defl_rad
            elif axis == 'yaw':
                delta_yaw += sign * defl_rad
            elif axis == 'roll':
                delta_roll += sign * defl_rad

        delta_pitch *= 0.5
        delta_yaw *= 0.5
        delta_roll *= 0.25

        M_body[1] = q_dyn * self.ref_area * self.ref_length * self.Cm_alpha * alpha_body
        M_body[1] += q_dyn * self.ref_area * self.ref_length * self.Cn_delta * delta_pitch
        M_body[2] += q_dyn * self.ref_area * self.ref_length * self.Cn_delta * delta_yaw
        M_body[0] += q_dyn * self.ref_area * self.ref_length * self.Cl_delta * delta_roll

        ang_accel = np.array([
            M_body[0] / self.Ixx,
            M_body[1] / self.Iyy,
            M_body[2] / self.Izz,
        ])
        self.angular_velocity_body += ang_accel * dt

        omega = self.angular_velocity_body
        omega_quat = np.array([omega[0] * 0.5, omega[1] * 0.5, omega[2] * 0.5, 0.0])
        q_dot = self.quat_mult(self.quaternion, omega_quat)
        self.quaternion += q_dot * dt
        self.quaternion = self.quat_normalize(self.quaternion)

        self.velocity_ned += accel_ned * dt
        self.position_ned += self.velocity_ned * dt
        self.altitude = -self.position_ned[2]

        roll, pitch, yaw = self.quat_to_euler(self.quaternion)

        baro_alt = self.altitude + self.baro_bias + np.random.normal(0, self.baro_noise_std)
        noisy_accel = specific_force_body + np.random.normal(0, self.accel_noise_std, 3)

        state = {
            "time": self.time,
            "altitude": self.altitude,
            "baro_altitude": baro_alt,
            "velocity_z": -self.velocity_ned[2],
            "airspeed": self.airspeed,
            "mach": self.mach,
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "yaw": math.degrees(yaw),
            "qx": self.quaternion[0],
            "qy": self.quaternion[1],
            "qz": self.quaternion[2],
            "qw": self.quaternion[3],
            "ax": noisy_accel[0],
            "ay": noisy_accel[1],
            "az": noisy_accel[2],
            "thrust": self.thrust_current,
            "mass": self.mass,
            "omega_x": self.angular_velocity_body[0],
            "omega_y": self.angular_velocity_body[1],
            "omega_z": self.angular_velocity_body[2],
            "g_load": np.linalg.norm(specific_force_body) / self.g,
            "wind_x": self.wind_ned[0],
            "wind_y": self.wind_ned[1],
            "wind_z": self.wind_ned[2],
            "density": self.density,
            "q_dynamic": q_dyn,
        }
        self._log.append(state)
        return state

    def get_log(self) -> List[dict]:
        return self._log

    def reset(self, scenario: Optional[dict] = None):
        if scenario is not None:
            self.__init__(scenario)
            return
        self.time = 0.0
        self.altitude = 0.0
        self.velocity_ned = np.zeros(3)
        self.position_ned = np.zeros(3)
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity_body = np.zeros(3)
        self.thrust_current = 0.0
        self.mach = 0.0
        self.airspeed = 0.0
        self.mass = self.mass_dry + self.mass_propellant
        self._log = []
