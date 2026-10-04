"""
================================================================================
BI-COPTER FLIGHT CORE (Pure Python Control & Stabilization Engine)
================================================================================

Hey there! This module is the mathematical "brain" of our bi-copter drone.
It handles all the flight stabilization physics, PID control loops, and the
actuator mixer that turns pilot commands into actual motor speeds and servo angles.

Why is this file pure Python with zero ROS dependencies?
-------------------------------------------------------
We intentionally separated the flight physics from ROS 2. This means you can:
1. Run automated unit tests in milliseconds (see test/test_flight_core.py).
2. Experiment with PID gains or math without launching Gazebo.
3. Easily port this exact control logic to MicroPython, C++, or physical microcontrollers.

Vehicle Physical Layout (Body Frame FLU: +X Forward, +Y Left, +Z Up):
---------------------------------------------------------------------
              Left Motor (0, CCW)          Right Motor (1, CW)
                     (O)  <----- 2*d ----->  (O)          d = arm (0.161 m)
                      |                       |
                   Servo 0                 Servo 1        Tilts about +Y axis
                                                          (+δ tilts thrust forward)

How does a Bi-Copter fly with only 2 propellers and 2 servos?
-------------------------------------------------------------
Unlike a standard 4-rotor quadcopter, a bi-copter is an under-actuated, tilting-rotor
aircraft. We control its 4 degrees of freedom using an exact physical allocation:

1. Altitude (Z)  -> Collective Thrust: Both motors spin up/down together (T0 + T1).
2. Roll (Phi)    -> Differential Thrust: One motor spins faster than the other (T0 - T1).
                    Because of the 0.161m arm, roll response is very snappy and powerful!
3. Pitch (Theta) -> Common Servo Tilt: Both servos tilt forward or backward together.
                    Since props are mounted near the tilt axis (lever arm lp ~ 1.7 cm),
                    pitch authority comes from tilting the entire thrust vector.
4. Yaw (Psi)     -> Differential Servo Tilt: Left servo tilts backward while right
                    servo tilts forward, producing an instant horizontal turning couple!

Control Pipeline (Modeled after Pixhawk / PX4 Architecture):
------------------------------------------------------------
[Pilot / Teleop Input]
       │
       ▼
1. Altitude Controller (Pixhawk ALTCTL):
   Calculates collective thrust needed to hold target height or commanded vertical rate.
       │
       ▼
2. Attitude P Controller:
   Compares target roll/pitch angles with current IMU angles -> desired angular rates.
       │
       ▼
3. Rate PID Controller (with D-term low-pass filter & anti-windup):
   Compares target rates with gyro angular velocities -> desired body torques (tau_x, tau_y, tau_z).
       │
       ▼
4. Bi-Copter Actuator Mixer:
   Inverts the physical geometry to solve for:
   - Motor 0 speed (rad/s) & Motor 1 speed (rad/s)
   - Servo 0 angle (rad) & Servo 1 angle (rad)
================================================================================
"""

import math
from dataclasses import dataclass, field


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_euler(w, x, y, z):
    """ZYX euler angles (roll, pitch, yaw) from a body->world quaternion."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def quat_rotate(w, x, y, z, v):
    """Rotate vector v (body) into the world frame with quaternion (w,x,y,z)."""
    vx, vy, vz = v
    # t = 2 * q_vec x v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class PID:
    """PID with integrator clamp, optional derivative-on-measurement and D low-pass."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_limit=1.0, out_limit=None, d_lpf_hz=30.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.out_limit = out_limit
        self.d_lpf_hz = d_lpf_hz
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None
        self.d_filt = 0.0

    def update(self, error, dt, measurement=None, integrate=True):
        """If `measurement` is given, D acts on -d(measurement)/dt (no set-point kick)."""
        if dt <= 0.0:
            return self.kp * error
        if integrate and self.ki > 0.0:
            self.integral = clamp(self.integral + error * dt, -self.i_limit / self.ki, self.i_limit / self.ki)
        sig = error if measurement is None else -measurement
        d_raw = 0.0 if self.prev is None else (sig - self.prev) / dt
        self.prev = sig
        if self.d_lpf_hz and self.d_lpf_hz > 0.0:
            alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.d_lpf_hz))
            self.d_filt += alpha * (d_raw - self.d_filt)
        else:
            self.d_filt = d_raw
        out = self.kp * error + self.ki * self.integral + self.kd * self.d_filt
        if self.out_limit is not None:
            out = clamp(out, -self.out_limit, self.out_limit)
        return out


@dataclass
class VehicleParams:
    mass: float = 0.481              # kg  (from CAD)
    ixx: float = 5.85e-3             # kg m^2 about CoM
    iyy: float = 5.89e-3
    izz: float = 9.41e-3
    arm: float = 0.161               # m, |y| of each rotor
    pitch_lever: float = 0.0854      # m, prop hub height above CoM (0.1378 - 0.0525)
    k_thrust: float = 6.0e-6         # N / (rad/s)^2   (must match model.sdf motorConstant)
    k_moment: float = 0.016          # m               (must match model.sdf momentConstant)
    max_rotor_speed: float = 1000.0  # rad/s
    min_rotor_speed: float = 100.0   # rad/s idle when armed
    max_tilt: float = 0.5236         # rad (servo limit, 30 deg)
    gravity: float = 9.81


@dataclass
class Gains:
    # attitude angle loops  (rate_sp = k * angle_error)
    roll_p: float = 6.0
    pitch_p: float = 4.0
    yaw_p: float = 2.5
    # body-rate loops  (output = angular acceleration rad/s^2)
    roll_rate: tuple = (12.0, 4.0, 0.25)     # kp, ki, kd
    pitch_rate: tuple = (8.0, 3.0, 0.15)
    yaw_rate: tuple = (4.0, 1.0, 0.0)
    max_rate_rp: float = 3.0                  # rad/s
    max_rate_yaw: float = 1.5
    # altitude loop (output = vertical acceleration m/s^2)
    alt: tuple = (4.0, 1.5, 3.0)
    max_climb_acc: float = 4.0
    # horizontal position loop (output = horizontal acceleration m/s^2)
    pos_p: float = 0.8
    vel_p: float = 1.6
    vel_i: float = 0.2
    max_tilt_angle: float = 0.25             # rad, max commanded roll/pitch


@dataclass
class State:
    pos: tuple = (0.0, 0.0, 0.0)          # world ENU [m]
    vel: tuple = (0.0, 0.0, 0.0)          # world ENU [m/s]
    quat: tuple = (1.0, 0.0, 0.0, 0.0)    # w, x, y, z  body -> world
    gyro: tuple = (0.0, 0.0, 0.0)         # body rates [rad/s]


@dataclass
class Setpoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 1.0
    yaw: float = 0.0
    roll: float = 0.0        # Commanded roll angle [rad] (Attitude mode)
    pitch: float = 0.0       # Commanded pitch angle [rad] (Attitude mode)
    thrust: float = 0.0      # Commanded collective thrust [N] (0.0 uses hover thrust m*g)
    yaw_rate: float = 0.0    # Commanded yaw rate [rad/s] (Attitude mode)


@dataclass
class Output:
    motor_speed: list = field(default_factory=lambda: [0.0, 0.0])   # rad/s [left, right]
    servo: list = field(default_factory=lambda: [0.0, 0.0])         # rad   [left, right]
    thrust: list = field(default_factory=lambda: [0.0, 0.0])        # N
    rpy: tuple = (0.0, 0.0, 0.0)
    rpy_sp: tuple = (0.0, 0.0, 0.0)
    torque: tuple = (0.0, 0.0, 0.0)
    collective: float = 0.0


class BiCopterController:
    def __init__(self, params: VehicleParams = None, gains: Gains = None):
        self.p = params or VehicleParams()
        self.g = gains or Gains()
        self.position_hold = False
        self.altitude_hold = True  # Pixhawk Altitude Mode (ALTCTL): auto altitude hold + manual attitude
        self._build()

    def _build(self):
        g = self.g
        self.pid_roll_rate = PID(*g.roll_rate, i_limit=3.0, d_lpf_hz=30.0)
        self.pid_pitch_rate = PID(*g.pitch_rate, i_limit=3.0, d_lpf_hz=30.0)
        self.pid_yaw_rate = PID(*g.yaw_rate, i_limit=2.0, d_lpf_hz=20.0)
        self.pid_alt = PID(*g.alt, i_limit=3.0, d_lpf_hz=10.0)
        self.pid_vx = PID(g.vel_p, g.vel_i, 0.0, i_limit=1.5)
        self.pid_vy = PID(g.vel_p, g.vel_i, 0.0, i_limit=1.5)

    def reset(self):
        for pid in (self.pid_roll_rate, self.pid_pitch_rate, self.pid_yaw_rate,
                    self.pid_alt, self.pid_vx, self.pid_vy):
            pid.reset()

    # ------------------------------------------------------------------ #
    def update(self, s: State, sp: Setpoint, dt: float, armed: bool = True) -> Output:
        p, g = self.p, self.g
        out = Output()
        roll, pitch, yaw = quat_to_euler(*s.quat)
        out.rpy = (roll, pitch, yaw)
        if not armed:
            self.reset()
            out.rpy_sp = (0.0, 0.0, yaw)
            return out

        on_ground = s.pos[2] < 0.10 and (sp.z < 0.15 and sp.thrust < 0.8 * p.mass * p.gravity)
        integrate = not on_ground

        # Ground safety: if vehicle is on the ground and commanded idle/low, hold zero thrust
        if on_ground and sp.z < 0.15 and sp.thrust < 0.8 * p.mass * p.gravity:
            self.reset()
            out.rpy_sp = (0.0, 0.0, yaw)
            out.motor_speed = [0.0, 0.0]
            out.servo = [0.0, 0.0]
            out.thrust = [0.0, 0.0]
            return out

        if self.position_hold:
            # ---------------- Full Position Hold Mode (GPS / MoCap) ----------------
            ez = sp.z - s.pos[2]
            az = self.pid_alt.update(ez, dt, measurement=s.pos[2], integrate=integrate)
            az = clamp(az, -0.6 * p.gravity, g.max_climb_acc)
            tilt_comp = max(math.cos(roll) * math.cos(pitch), 0.65)
            collective = clamp(p.mass * (p.gravity + az) / tilt_comp, 0.0, 9.5)

            vx_sp = clamp(g.pos_p * (sp.x - s.pos[0]), -2.0, 2.0)
            vy_sp = clamp(g.pos_p * (sp.y - s.pos[1]), -2.0, 2.0)
            ax = self.pid_vx.update(vx_sp - s.vel[0], dt, integrate=integrate)
            ay = self.pid_vy.update(vy_sp - s.vel[1], dt, integrate=integrate)
            a_fwd = math.cos(yaw) * ax + math.sin(yaw) * ay
            a_left = -math.sin(yaw) * ax + math.cos(yaw) * ay
            pitch_sp = clamp(math.atan2(a_fwd, p.gravity), -g.max_tilt_angle, g.max_tilt_angle)
            roll_sp = clamp(-math.atan2(a_left, p.gravity), -g.max_tilt_angle, g.max_tilt_angle)
            out.rpy_sp = (roll_sp, pitch_sp, sp.yaw)

        elif self.altitude_hold and sp.thrust <= 0.0:
            # ---------------- Pixhawk Altitude Mode (ALTCTL) ----------------
            # Auto altitude hold (No drifting into sky!) + Pure Manual Attitude Sticks
            ez = sp.z - s.pos[2]
            az = self.pid_alt.update(ez, dt, measurement=s.pos[2], integrate=integrate)
            az = clamp(az, -0.6 * p.gravity, g.max_climb_acc)
            tilt_comp = max(math.cos(roll) * math.cos(pitch), 0.65)
            collective = clamp(p.mass * (p.gravity + az) / tilt_comp, 0.0, 9.5)

            # Direct attitude setpoints from pilot (roll, pitch) - NO horizontal position lock
            roll_sp = clamp(sp.roll, -g.max_tilt_angle, g.max_tilt_angle)
            pitch_sp = clamp(sp.pitch, -g.max_tilt_angle, g.max_tilt_angle)
            out.rpy_sp = (roll_sp, pitch_sp, sp.yaw)

        else:
            # ---------------- Pixhawk Pure Manual Throttle Mode (STABILIZED) ----------------
            collective = sp.thrust
            if collective > 0.0:
                tilt_comp = clamp(math.cos(roll) * math.cos(pitch), 0.75, 1.0)
                collective = collective / tilt_comp
                # Aerodynamic vertical damping: opposes runaway upward/downward drift
                vz = s.vel[2]
                collective = clamp(collective - 0.40 * vz, 0.0, 9.5)

            roll_sp = clamp(sp.roll, -g.max_tilt_angle, g.max_tilt_angle)
            pitch_sp = clamp(sp.pitch, -g.max_tilt_angle, g.max_tilt_angle)
            out.rpy_sp = (roll_sp, pitch_sp, sp.yaw)

        # Attitude P loop -> desired angular rates
        rate_sp_x = clamp(g.roll_p * (roll_sp - roll), -g.max_rate_rp, g.max_rate_rp)
        rate_sp_y = clamp(g.pitch_p * (pitch_sp - pitch), -g.max_rate_rp, g.max_rate_rp)
        if abs(sp.yaw_rate) > 1e-4:
            rate_sp_z = clamp(sp.yaw_rate, -g.max_rate_yaw, g.max_rate_yaw)
        else:
            rate_sp_z = clamp(g.yaw_p * wrap_pi(sp.yaw - yaw), -g.max_rate_yaw, g.max_rate_yaw)

        # ---------------- rates -> torque ----------------
        wx, wy, wz = s.gyro
        alpha_x = self.pid_roll_rate.update(rate_sp_x - wx, dt, measurement=wx, integrate=integrate)
        alpha_y = self.pid_pitch_rate.update(rate_sp_y - wy, dt, measurement=wy, integrate=integrate)
        alpha_z = self.pid_yaw_rate.update(rate_sp_z - wz, dt, measurement=wz, integrate=integrate)
        tau = (p.ixx * alpha_x, p.iyy * alpha_y, p.izz * alpha_z)

        out.torque = tau
        out.collective = collective
        self.mix(collective, tau, out)
        return out

    # ------------------------------------------------------------------ #
    def mix(self, collective, tau, out: Output):
        """Exact inverse of the actuator model (see module doc)."""
        p = self.p
        tx, ty, tz = tau
        d, lp = p.arm, p.pitch_lever

        # vertical thrust components (roll by differential thrust)
        tv0 = 0.5 * collective + tx / (2.0 * d)
        tv1 = 0.5 * collective - tx / (2.0 * d)
        # rotor drag yaw feed-forward: left CCW -> -kM*T0, right CW -> +kM*T1
        tz_servo = tz - p.k_moment * (tv1 - tv0)
        # horizontal thrust components (pitch by common tilt, yaw by differential tilt)
        a0 = +ty / (2.0 * lp) - tz_servo / (2.0 * d)
        a1 = +ty / (2.0 * lp) + tz_servo / (2.0 * d)

        t_min = p.k_thrust * p.min_rotor_speed ** 2
        t_max = p.k_thrust * p.max_rotor_speed ** 2
        tan_max = math.tan(p.max_tilt)
        speeds, servos, thrusts = [], [], []
        for tv, a in ((tv0, a0), (tv1, a1)):
            tv = clamp(tv, t_min, t_max)
            a = clamp(a, -tv * tan_max, tv * tan_max)
            delta = math.atan2(a, tv)
            t = math.hypot(tv, a)
            t = clamp(t, t_min, t_max)
            speeds.append(math.sqrt(t / p.k_thrust))
            servos.append(delta)
            thrusts.append(t)
        out.motor_speed = speeds
        out.servo = servos
        out.thrust = thrusts
