#!/usr/bin/env python3
"""
================================================================================
BI-COPTER TELEOPERATION CONSOLE (Interactive RC Transmitter & Live HUD)
================================================================================

Hey pilot! Welcome to your cockpit. This node transforms your keyboard into a
high-precision Radio Control (RC) transmitter (Mode 2) for flying our bi-copter
in Gazebo Sim, complete with a real-time ASCII telemetry heads-up display (HUD).

Why this node exists:
--------------------
Flying an under-actuated tilting-rotor aircraft like a bi-copter requires
responsive, intuitive controls. Instead of clunky sliders or awkward GUIs,
this node provides:
1. Spring-Centered Sticks: Just like the gimbals on a Radiomaster TX16S or
   FrSky RC transmitter, roll, pitch, and yaw spring back to level (0°) the
   moment you release the keys.
2. Pixhawk Altitude Mode (ALTCTL): The flight controller automatically handles
   vertical stabilization and hover. You tell it how high you want to fly, and
   it holds altitude rock-solid without runaway climbing or dropping.
3. Direct Terminal Control (/dev/tty): Standard ROS 2 launch files buffer stdin
   into pipes, causing sluggish keyboard delay. We open `/dev/tty` directly in
   raw mode to give you instant, zero-latency stick response.

How the Control Sticks Work:
----------------------------
- Pitch (W / S or Up / Down): Tilts the drone nose forward or backward.
- Roll  (A / D or Left / Right): Banks the drone left or right.
- Yaw   (Q / E): Rotates the drone heading left (CCW) or right (CW) at a controlled rate.
- Climb (R / F): Commands the altitude hold controller to step up (+0.2m) or down (-0.2m).
- Space / H: Levels out immediately and locks into a stable 1.0 m hover.

Safety & Automation Features:
-----------------------------
- Auto-Takeoff on First Input: If the drone is sitting disarmed on the launch pad,
  pressing any flight key (W, A, S, D, R, T) will automatically arm the motors,
  spool up smoothly, and gently lift off to a stable 1.0 m hover.
- Auto-Land (L): Gradually descends at 0.15 m/s until ground contact is detected,
  then disarms the motors safely.
- Emergency Kill Switch (X): Instantly cuts all motor power to 0 N and disarms.
- Live HUD: Continuously displays target altitude, stick angles, actual drone RPY,
  motor RPM (rad/s), and servo tilt angles in degrees right on the bottom line.
================================================================================
"""

import json
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


BANNER = """
\033[1;36m================================================================================\033[0m
\033[1;37m        BI-COPTER PIXHAWK FLIGHT CONTROL & TELEOPERATION CONSOLE                \033[0m
\033[1;36m================================================================================\033[0m
\033[1;33mFlight Sticks (Pixhawk ALTCTL - Auto Altitude Hold & Manual Attitude):\033[0m
    \033[1m[ W ]\033[0m / \033[1m[ S ]\033[0m  or  \033[1m[ ↑ ]\033[0m / \033[1m[ ↓ ]\033[0m  : Pitch Forward (Nose Down) / Pitch Backward (Nose Up)
    \033[1m[ A ]\033[0m / \033[1m[ D ]\033[0m  or  \033[1m[ ← ]\033[0m / \033[1m[ → ]\033[0m  : Roll Bank Left / Bank Right
    \033[1m[ R ]\033[0m / \033[1m[ F ]\033[0m                    : Climb UP (+0.2m) / Descend DOWN (-0.2m)
    \033[1m[ Q ]\033[0m / \033[1m[ E ]\033[0m                    : Yaw Rotate LEFT / RIGHT (Rate)
    \033[1m[ SPACE ]\033[0m or \033[1m[ H ]\033[0m              : Level Out (Zero Angles, Reset to 1.0m Hover)

\033[1;33mFlight Operations:\033[0m
    \033[1;32m[ T ]\033[0m : Auto TAKEOFF to 1.0m Hover (Lifts off smoothly and holds height!)
    \033[1;34m[ L ]\033[0m : Auto LAND (Gentle descent to ground & Disarm)
    \033[1;35m[ M ]\033[0m : Toggle ARM / DISARM
    \033[1;31m[ X ]\033[0m : EMERGENCY KILL SWITCH (Cut Motors & Disarm)

\033[1;33mStick Sensitivity:\033[0m
    \033[1m[ + ]\033[0m / \033[1m[ - ]\033[0m  : Increase / Decrease Max Tilt Angle (±1.0°)

\033[1;32mTip: Press [W], [S], [A], [D] or [T] to automatically takeoff from ground!\033[0m
\033[1;30mPress Ctrl+C to exit.\033[0m
\033[1;36m--------------------------------------------------------------------------------\033[0m
"""


class BiCopterTeleopNode(Node):
    def __init__(self):
        super().__init__("bicopter_teleop")

        # Vehicle physical constants
        self.mass = 0.481           # kg
        self.gravity = 9.81         # m/s^2
        self.hover_thrust = self.mass * self.gravity  # ~4.72 N

        # Commanded setpoints
        self.target_alt = 1.0       # Target hover altitude [m]
        self.cmd_pitch = 0.0        # Target pitch angle [rad] (springs to 0)
        self.cmd_roll = 0.0         # Target roll angle [rad] (springs to 0)
        self.cmd_yaw_rate = 0.0     # Target yaw rate [rad/s] (springs to 0)

        # Stick limits & sensitivity
        self.tilt_limit_deg = 12.0  # Maximum roll/pitch tilt angle
        self.yaw_rate_limit = 1.0   # rad/s

        # Flight state
        self.armed = False          # Start safely on ground disarmed
        self.last_key_time = time.time()
        self.landing_active = False

        # Live feedback from simulation
        self.current_alt = 0.0
        self.current_climb = 0.0
        self.current_rpy = [0.0, 0.0, 0.0]
        self.motor_rad_s = [0.0, 0.0]
        self.servos_deg = [0.0, 0.0]
        self.controller_thrust = 0.0

        # ROS 2 Publishers
        self.pub_att = self.create_publisher(Twist, "/cmd_attitude_thrust", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_arm = self.create_publisher(Bool, "/arm", 10)
        self.pub_mode = self.create_publisher(String, "/mode", 10)

        # ROS 2 Subscribers (Live telemetry feedback)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.sub_telem = self.create_subscription(String, "/telemetry", self.telem_callback, 10)

        # 20 Hz Command Publisher Loop
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.current_alt = p.z
        self.current_climb = v.z

        # Quat to Euler
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        self.current_rpy = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

        # Check landing touchdown condition
        if self.landing_active:
            if self.current_alt < 0.10 and abs(self.current_climb) < 0.10:
                self.landing_active = False
                self.armed = False
                self.send_arm(False)

    def telem_callback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.motor_rad_s = d.get("motor_rad_s", self.motor_rad_s)
            self.servos_deg = d.get("servos_deg", self.servos_deg)
            self.controller_thrust = d.get("thrust_N", self.controller_thrust)
            if "armed" in d and not self.armed:
                self.armed = d["armed"]
        except Exception:
            pass

    def control_loop(self):
        now = time.time()

        # Spring-center attitude sticks: decay pitch, roll, yaw rate to 0 if keys released
        if now - self.last_key_time > 0.15:
            self.cmd_pitch *= 0.60
            self.cmd_roll *= 0.60
            self.cmd_yaw_rate *= 0.60
            if abs(self.cmd_pitch) < 0.005:
                self.cmd_pitch = 0.0
            if abs(self.cmd_roll) < 0.005:
                self.cmd_roll = 0.0
            if abs(self.cmd_yaw_rate) < 0.005:
                self.cmd_yaw_rate = 0.0

        # Construct Attitude & Altitude command message
        cmd = Twist()
        cmd.linear.x = float(self.cmd_roll)
        cmd.linear.y = float(self.cmd_pitch)
        cmd.linear.z = float(self.target_alt if self.armed else 0.0)
        cmd.angular.x = float(self.cmd_roll)
        cmd.angular.y = float(self.cmd_pitch)
        cmd.angular.z = float(self.cmd_yaw_rate)

        # Publish to both topics for complete compatibility
        self.pub_att.publish(cmd)
        self.pub_cmd_vel.publish(cmd)

    def send_arm(self, state: bool):
        self.armed = state
        msg = Bool()
        msg.data = state
        self.pub_arm.publish(msg)

    def send_mode(self, mode: str):
        msg = String()
        msg.data = mode.upper()
        self.pub_mode.publish(msg)

    def level_out(self):
        """Instantly zero out all angular sticks and restore 1.0m hover."""
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw_rate = 0.0
        self.target_alt = 1.0
        self.landing_active = False

    def start_takeoff(self):
        """Smooth automated takeoff to 1.0m hover."""
        self.armed = True
        self.landing_active = False
        self.target_alt = 1.0
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw_rate = 0.0
        self.send_arm(True)
        self.send_mode("TAKEOFF")

    def start_landing(self):
        """Smooth descent to touchdown and disarm."""
        self.landing_active = True
        self.target_alt = 0.02
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw_rate = 0.0
        self.send_mode("LAND")

    def emergency_stop(self):
        """Immediate motor cut and disarm."""
        self.landing_active = False
        self.armed = False
        self.target_alt = 0.0
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw_rate = 0.0
        self.send_arm(False)


def get_key(fd):
    """Read a single key or escape sequence non-blockingly from file descriptor."""
    if fd is None:
        time.sleep(0.04)
        return ""
    rlist, _, _ = select.select([fd], [], [], 0.04)
    if rlist:
        try:
            key = os.read(fd, 1).decode('latin1')
            if key == "\x1b":  # Arrow key escape sequence
                rlist_seq, _, _ = select.select([fd], [], [], 0.02)
                if rlist_seq:
                    key += os.read(fd, 2).decode('latin1')
            return key
        except Exception:
            return ""
    return ""


def main(args=None):
    rclpy.init(args=args)
    node = BiCopterTeleopNode()

    # Open controlling TTY directly to capture keystrokes reliably even under ros2 launch
    tty_fd = None
    settings = None
    try:
        tty_file = open('/dev/tty', 'r')
        tty_fd = tty_file.fileno()
        settings = termios.tcgetattr(tty_fd)
        tty.setraw(tty_fd)
    except Exception:
        try:
            if sys.stdin.isatty():
                tty_fd = sys.stdin.fileno()
                settings = termios.tcgetattr(tty_fd)
                tty.setraw(tty_fd)
        except Exception:
            pass

    print(BANNER)
    status_line_len = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(tty_fd)

            if key:
                node.last_key_time = time.time()
                tilt_rad = math.radians(node.tilt_limit_deg)

                # Auto-takeoff on first flight movement if drone is sitting on ground
                if not node.armed and key in ["w", "W", "s", "S", "a", "A", "d", "D", "r", "R", "\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D"]:
                    node.start_takeoff()

                # --- Pitch Stick (Nose Down / Forward vs Nose Up / Backward) ---
                if key in ["w", "W", "\x1b[A"]:  # Pitch Forward / Nose Down
                    node.cmd_pitch = tilt_rad
                elif key in ["s", "S", "\x1b[B"]:  # Pitch Backward / Nose Up
                    node.cmd_pitch = -tilt_rad

                # --- Roll Stick (Bank Left vs Bank Right) ---
                elif key in ["a", "A", "\x1b[D"]:  # Roll Left
                    node.cmd_roll = -tilt_rad
                elif key in ["d", "D", "\x1b[C"]:  # Roll Right
                    node.cmd_roll = tilt_rad

                # --- Altitude / Vertical Climb Control ---
                elif key in ["r", "R"]:  # Climb UP
                    node.target_alt = min(5.0, round(node.target_alt + 0.20, 2))
                    node.landing_active = False
                elif key in ["f", "F"]:  # Descend DOWN
                    node.target_alt = max(0.20, round(node.target_alt - 0.20, 2))

                # --- Yaw Stick (Angular Rotation Rate) ---
                elif key in ["q", "Q"]:  # Rotate Yaw Left
                    node.cmd_yaw_rate = node.yaw_rate_limit
                elif key in ["e", "E"]:  # Rotate Yaw Right
                    node.cmd_yaw_rate = -node.yaw_rate_limit

                # --- Self-Leveling & Hover Reset ---
                elif key in [" ", "h", "H"]:  # Level Out
                    node.level_out()

                # --- Operational Commands ---
                elif key in ["t", "T"]:  # Takeoff
                    node.start_takeoff()
                elif key in ["l", "L"]:  # Land
                    node.start_landing()
                elif key in ["m", "M"]:  # Toggle Arm / Disarm
                    node.send_arm(not node.armed)
                elif key in ["x", "X"]:  # Emergency Kill Switch
                    node.emergency_stop()

                # --- Stick Sensitivity Adjustments ---
                elif key in ["+", "="]:
                    node.tilt_limit_deg = min(25.0, round(node.tilt_limit_deg + 1.0, 1))
                elif key in ["-", "_"]:
                    node.tilt_limit_deg = max(4.0, round(node.tilt_limit_deg - 1.0, 1))

                elif key in ["\x03", "q", "Q"] and not node.armed:  # Ctrl+C or Quit when disarmed
                    if key == "\x03":
                        break

            # Render live RC Transmitter Telemetry HUD
            arm_str = "\033[1;32mARMED (Flying)\033[0m" if node.armed else "\033[1;31mDISARMED (On Ground)\033[0m"
            p_deg = math.degrees(node.cmd_pitch)
            r_deg = math.degrees(node.cmd_roll)
            y_deg_s = math.degrees(node.cmd_yaw_rate)

            rpy = node.current_rpy
            m0, m1 = node.motor_rad_s[0], node.motor_rad_s[1]
            s0, s1 = node.servos_deg[0], node.servos_deg[1]

            status = (
                f"\r[{arm_str}] "
                f"TgtAlt: \033[1;33m{node.target_alt:4.2f}m\033[0m | "
                f"Sticks: [R:{r_deg:+4.1f}°, P:{p_deg:+4.1f}°, Y:{y_deg_s:+4.0f}°/s] | "
                f"Drone: Alt={node.current_alt:4.2f}m (vz:{node.current_climb:+4.1f}) "
                f"RPY=[{rpy[0]:+4.1f}°, {rpy[1]:+4.1f}°, {rpy[2]:+5.1f}°] "
                f"Mot=[{m0:3.0f}, {m1:3.0f}]r/s "
                f"Servos=[{s0:+4.1f}°, {s1:+4.1f}°]"
            )
            sys.stdout.write(status + " " * max(0, status_line_len - len(status)))
            sys.stdout.flush()
            status_line_len = len(status)

    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None and tty_fd is not None:
            termios.tcsetattr(tty_fd, termios.TCSADRAIN, settings)
        print("\n\033[1;33mTeleop node shut down. Terminal reset.\033[0m")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
