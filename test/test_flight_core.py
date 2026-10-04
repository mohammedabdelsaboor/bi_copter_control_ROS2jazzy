#!/usr/bin/env python3
"""
================================================================================
BI-COPTER FLIGHT CORE TEST SUITE (Automated Physics & Control Verification)
================================================================================

Welcome engineer! This module contains the automated pytest verification suite
for `flight_core.py`.

Why fast unit tests matter for flight software:
----------------------------------------------
Testing flight control algorithms inside a full 3D physics simulator like Gazebo
takes dozens of seconds to launch and involves sensor noise and rendering overhead.
Because our `flight_core.py` engine is pure Python with zero ROS dependencies,
this test suite runs in under 50 milliseconds! It gives you instant mathematical
certainty that any change to PID gains, geometry, or mixer formulas won't crash
the drone before you ever open Gazebo.

What this test suite verifies:
------------------------------
1. Math Primitives (`test_clamp`, `test_wrap_pi`):
   Ensures angle wrapping between [-π, +π] and strict saturation bounds.
2. 3D Rotation Math (`test_quat_to_euler`):
   Validates quaternion-to-Euler conversion against known 30°, 45°, and 60°
   rotations about the X, Y, and Z axes.
3. PID Controller Dynamics (`test_pid`):
   Tests proportional, integral, and derivative responses, as well as the
   anti-windup integrator clamping.
4. Hover Equilibrium (`test_mixer_pure_hover`):
   Verifies that hovering commands equal motor speeds (~627 rad/s) and zero
   servo tilt angles.
5. 4-DOF Mixer Physics:
   - Roll Moment (`test_mixer_roll`): Verified via differential rotor thrust.
   - Pitch Moment (`test_mixer_pitch`): Verified via common servo tilt.
   - Yaw Couple (`test_mixer_yaw`): Verified via differential servo tilt.
6. Actuator Hardware Limits (`test_actuator_limits`):
   Ensures that extreme torque commands cannot exceed the mechanical 30° servo
   limit or max motor speed limit.
7. Pixhawk Stabilized Mode (`test_attitude_thrust_mode`):
   Confirms that in Pixhawk attitude/thrust mode, collective thrust and stick
   angles are obeyed directly while horizontal position errors are ignored.

How to run:
-----------
    $ pytest bi_copter_control/test/test_flight_core.py
================================================================================
"""

import math
import pytest

from bi_copter_control.flight_core import (
    PID,
    BiCopterController,
    VehicleParams,
    Gains,
    State,
    Setpoint,
    quat_to_euler,
    clamp,
    wrap_pi,
)


def test_clamp():
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-2.0, 0.0, 10.0) == 0.0
    assert clamp(15.0, 0.0, 10.0) == 10.0


def test_wrap_pi():
    assert abs(wrap_pi(0.0)) < 1e-6
    assert abs(wrap_pi(2.0 * math.pi)) < 1e-6
    assert abs(wrap_pi(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6


def test_quat_to_euler():
    # Identity
    r, p, y = quat_to_euler(1.0, 0.0, 0.0, 0.0)
    assert abs(r) < 1e-5 and abs(p) < 1e-5 and abs(y) < 1e-5

    # 45 deg roll (rot about X)
    a = math.radians(45)
    r, p, y = quat_to_euler(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)
    assert abs(math.degrees(r) - 45.0) < 0.1

    # 30 deg pitch (rot about Y)
    a = math.radians(30)
    r, p, y = quat_to_euler(math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)
    assert abs(math.degrees(p) - 30.0) < 0.1

    # 60 deg yaw (rot about Z)
    a = math.radians(60)
    r, p, y = quat_to_euler(math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))
    assert abs(math.degrees(y) - 60.0) < 0.1


def test_pid():
    pid = PID(kp=2.0, ki=1.0, kd=0.5, i_limit=5.0)
    out = pid.update(error=1.0, dt=0.01)
    # P = 2.0*1.0 = 2.0; I = 1.0*0.01 = 0.01; D on error
    assert out > 2.0

    # Test integrator clamp
    for _ in range(1000):
        pid.update(error=10.0, dt=0.1)
    assert abs(pid.integral) <= 5.0 / 1.0 + 1e-5


def test_mixer_pure_hover():
    ctrl = BiCopterController()
    p = ctrl.p
    collective = p.mass * p.gravity
    tau = (0.0, 0.0, 0.0)
    from bi_copter_control.flight_core import Output
    out = Output()
    ctrl.mix(collective, tau, out)

    # Hover: both motors equal speed
    assert abs(out.motor_speed[0] - out.motor_speed[1]) < 1e-3
    assert out.motor_speed[0] > 600.0  # Approx 627 rad/s for 0.48 kg
    # Servos should be zero tilt
    assert abs(out.servo[0]) < 1e-4
    assert abs(out.servo[1]) < 1e-4


def test_mixer_roll():
    ctrl = BiCopterController()
    p = ctrl.p
    collective = p.mass * p.gravity
    # Command positive roll moment (tilt right wing down -> left thrust higher)
    tau = (+0.05, 0.0, 0.0)
    from bi_copter_control.flight_core import Output
    out = Output()
    ctrl.mix(collective, tau, out)

    assert out.motor_speed[0] > out.motor_speed[1]
    # Servos should remain approximately upright
    assert abs(out.servo[0]) < 0.05
    assert abs(out.servo[1]) < 0.05


def test_mixer_pitch():
    ctrl = BiCopterController()
    p = ctrl.p
    collective = p.mass * p.gravity
    # Command pitch moment
    tau = (0.0, +0.02, 0.0)
    from bi_copter_control.flight_core import Output
    out = Output()
    ctrl.mix(collective, tau, out)

    # Both servos tilt in same direction
    assert out.servo[0] > 0.0
    assert out.servo[1] > 0.0
    assert abs(out.servo[0] - out.servo[1]) < 1e-3


def test_mixer_yaw():
    ctrl = BiCopterController()
    p = ctrl.p
    collective = p.mass * p.gravity
    # Command yaw moment
    tau = (0.0, 0.0, +0.02)
    from bi_copter_control.flight_core import Output
    out = Output()
    ctrl.mix(collective, tau, out)

    # Servos tilt in opposite directions
    assert (out.servo[1] - out.servo[0]) > 0.0


def test_actuator_limits():
    ctrl = BiCopterController()
    p = ctrl.p
    # Command extreme torque
    tau = (1.0, 1.0, 1.0)
    from bi_copter_control.flight_core import Output
    out = Output()
    ctrl.mix(collective=20.0, tau=tau, out=out)

    # Servos should not exceed max_tilt (30 deg = 0.5236 rad)
    assert abs(out.servo[0]) <= p.max_tilt + 1e-5
    assert abs(out.servo[1]) <= p.max_tilt + 1e-5
    # Motors should not exceed max_rotor_speed
    assert out.motor_speed[0] <= p.max_rotor_speed
    assert out.motor_speed[1] <= p.max_rotor_speed


def test_attitude_thrust_mode():
    """Verify Pixhawk-style direct attitude and collective thrust control (no position control)."""
    ctrl = BiCopterController()
    ctrl.position_hold = False  # Pixhawk Stabilized Mode

    s = State(
        pos=(100.0, -50.0, 25.0),  # Arbitrary position far from origin
        vel=(0.0, 0.0, 0.0),
        quat=(1.0, 0.0, 0.0, 0.0), # Flat attitude
        gyro=(0.0, 0.0, 0.0),
    )

    # Command: 5.5 N collective thrust, 0.1 rad roll, 0.05 rad pitch, 0.4 rad/s yaw rate
    sp = Setpoint(
        x=0.0, y=0.0, z=0.0,  # Position setpoints MUST be ignored
        roll=0.1,
        pitch=0.05,
        thrust=5.5,
        yaw_rate=0.4,
    )

    out = ctrl.update(s, sp, dt=0.01, armed=True)

    # Collective thrust must reflect commanded 5.5 N (not position error!)
    assert abs(out.collective - 5.5) < 0.1
    # Roll and pitch setpoints must match commanded angles
    assert abs(out.rpy_sp[0] - 0.1) < 1e-4
    assert abs(out.rpy_sp[1] - 0.05) < 1e-4
    # Motors must be running and servos active
    assert out.motor_speed[0] > 0.0 and out.motor_speed[1] > 0.0
    assert abs(out.servo[0]) > 0.0 or abs(out.servo[1]) > 0.0
