#!/usr/bin/env python3
"""
================================================================================
BI-COPTER FLIGHT CONTROLLER & BRIDGE NODE (ROS 2 + Gazebo + QGroundControl)
================================================================================

Welcome! This is the central hub of the bi-copter simulation. It connects
three major systems together in real time:
1. The Physics Simulator: Gazebo Sim (Harmonic)
2. The Flight Controller: Cascaded PIDs & Mixer (flight_core.py)
3. The Ground Station: QGroundControl (connected over MAVLink UDP 14550)

What this node does at 100 Hz:
------------------------------
1. Reads Drone State:
   Subscribes to Gazebo odometry/IMU to track real-time position, velocity,
   and 3D orientation (quaternion -> Euler angles).

2. Runs the Flight Stabilization Loop:
   Executes the BiCopterController logic:
   - Holds altitude safely (ALTCTL mode) so the drone never flies away into the sky.
   - Self-levels pitch and roll angles when pilot sticks are centered.
   - Calculates the exact required motor speeds and servo tilt angles.

3. Commands Actuators:
   Publishes commands directly to Gazebo:
   - Motor 0 & 1 rotational speeds -> /bi_copter/command/motor_speed (Actuators)
   - Servo 0 & 1 angular positions -> /model/bi_copter/servo_0 & servo_1 (Double)

4. Visualizes Aerodynamic Forces (Live 3D HUD):
   Publishes dynamic color-coded 3D thrust vector arrows in RViz2 and Gazebo:
   - Cyan arrow: Left rotor thrust
   - Amber arrow: Right rotor thrust
   - Floating text badges: Live Newtons and tilt angle (e.g., "LEFT: 2.36N (+5.2°)")

5. Bridges QGroundControl Telemetry (Built-in MAVLink Server):
   Broadcasts standard MAVLink packets on UDP port 14550 (HEARTBEAT, ATTITUDE,
   VFR_HUD, SYS_STATUS). Open QGroundControl on your computer and it will
   instantly recognize the drone as Vehicle 1 with a live Artificial Horizon!
================================================================================
"""

import json
import math
import os
import socket
import struct
import sys
import threading
import time

import gz.msgs10.actuators_pb2 as actuators_pb2
import gz.msgs10.double_pb2 as double_pb2
import gz.msgs10.marker_pb2 as gz_marker_pb2
import gz.msgs10.odometry_pb2 as odometry_pb2
import gz.transport13 as gz_transport

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from bi_copter_control.flight_core import (
    BiCopterController,
    State,
    Setpoint,
    quat_to_euler,
    clamp,
    wrap_pi,
)


# ============================================================================== #
# Pure-Python MAVLink Server (QGroundControl Telemetry & Flight Control Bridge)
# ============================================================================== #
def _crc16_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _crc16(buf, crc_extra):
    crc = 0xFFFF
    for b in buf:
        crc = _crc16_accumulate(b, crc)
    return _crc16_accumulate(crc_extra, crc)


class MavlinkServer:
    """Non-blocking MAVLink v1 server communicating with QGroundControl on UDP 14550."""

    def __init__(self, target_host="127.0.0.1", target_port=14550, bind_port=14540):
        self.target = (target_host, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)
        try:
            self.sock.bind(("0.0.0.0", bind_port))
        except Exception:
            try:
                self.sock.bind(("0.0.0.0", 0))
            except Exception:
                pass
        self.seq = 0
        self.connected = False

    def pack(self, msg_id, payload, crc_extra):
        seq = self.seq % 256
        self.seq += 1
        hdr = struct.pack("<BBBBBB", 0xFE, len(payload), seq, 1, 1, msg_id)
        crc = _crc16(hdr[1:] + payload, crc_extra)
        return hdr + payload + struct.pack("<H", crc)

    def send_heartbeat(self, armed=True):
        base_mode = (128 if armed else 0) | 1  # ARMED | CUSTOM_MODE
        custom_mode = 0x00010000  # PX4 MAIN_MODE_MANUAL (Stabilized)
        status = 4 if armed else 3
        payload = struct.pack("<IBBBBB", custom_mode, 2, 12, base_mode, status, 3)
        self._send(self.pack(0, payload, 50))

    def send_attitude(self, time_boot_ms, roll, pitch, yaw, p, q, r):
        payload = struct.pack(
            "<Iffffff",
            int(time_boot_ms),
            float(roll),
            float(pitch),
            float(yaw),
            float(p),
            float(q),
            float(r),
        )
        self._send(self.pack(30, payload, 39))

    def send_vfr_hud(self, airspeed, groundspeed, heading, throttle_pct, alt, climb):
        payload = struct.pack(
            "<ffffhH",
            float(airspeed),
            float(groundspeed),
            float(alt),
            float(climb),
            int(heading),
            int(clamp(throttle_pct, 0, 100)),
        )
        self._send(self.pack(74, payload, 20))

    def send_sys_status(self, voltage_mv=12400):
        payload = struct.pack(
            "<IIIHHhHhhhhhh",
            0x00200001,
            0x00200001,
            0x00200001,
            120,
            voltage_mv,
            1500,
            95,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._send(self.pack(1, payload, 124))

    def _send(self, pkt):
        try:
            self.sock.sendto(pkt, self.target)
        except Exception:
            pass

    def poll(self):
        """Read incoming MAVLink packets from QGroundControl."""
        arm_req = None
        manual_ctrl = None
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
                if not data:
                    break
                self.target = addr  # Lock onto sender address
                self.connected = True
                if len(data) >= 8 and data[0] in [0xFE, 0xFD]:
                    msg_id = data[5] if data[0] == 0xFE else data[7]
                    payload_len = data[1]
                    payload = data[6 : 6 + payload_len] if data[0] == 0xFE else data[10 : 10 + payload_len]

                    # COMMAND_LONG (400 = MAV_CMD_COMPONENT_ARM_DISARM)
                    if msg_id == 76 and len(payload) >= 32:
                        cmd = struct.unpack("<H", payload[30:32])[0]
                        param1 = struct.unpack("<f", payload[0:4])[0]
                        if cmd == 400:
                            arm_req = (param1 == 1.0)
                            ack = self.pack(77, struct.pack("<HB", 400, 0), 143)
                            self._send(ack)

                    # MANUAL_CONTROL (Joystick from QGC)
                    elif msg_id == 69 and len(payload) >= 11:
                        tgt, x, y, z, r, btns = struct.unpack("<BhhhhH", payload[:11])
                        manual_ctrl = (x / 1000.0, y / 1000.0, z / 1000.0, r / 1000.0)
            except (BlockingIOError, socket.error):
                break
        return arm_req, manual_ctrl


# ============================================================================== #
# Main Bi-Copter ROS 2 Controller Node
# ============================================================================== #
class BiCopterNode(Node):
    def __init__(self):
        super().__init__("bicopter_controller")

        # Declare parameters
        self.declare_parameter("flight_mode", "altitude")
        self.declare_parameter("takeoff_altitude", 1.0)
        self.declare_parameter("armed_on_start", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("model_name", "bi_copter")
        self.declare_parameter("world_name", "bicopter_world")
        self.declare_parameter("control_rate_hz", 100.0)

        self.flight_mode = str(self.get_parameter("flight_mode").value).lower()
        self.takeoff_alt = float(self.get_parameter("takeoff_altitude").value)
        self.armed = bool(self.get_parameter("armed_on_start").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.control_rate = float(self.get_parameter("control_rate_hz").value)

        # Core controller
        self.ctrl = BiCopterController()
        # Flight mode configuration:
        # "altitude": Pixhawk ALTCTL (auto altitude hold, manual attitude sticks, NO position lock)
        # "attitude_thrust": Pixhawk STABILIZED (direct manual collective thrust, manual attitude sticks)
        # "position": Full 3D Cartesian position lock
        if self.flight_mode in ["position", "pos", "position_hold"]:
            self.ctrl.position_hold = True
            self.ctrl.altitude_hold = True
        elif self.flight_mode in ["manual", "thrust", "attitude_thrust", "stabilized"]:
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = False
        else:  # Default: altitude (Pixhawk Altitude Mode / ALTCTL)
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = True

        # Hover collective thrust (m * g = 0.481 * 9.81 = 4.719 N)
        self.hover_thrust = self.ctrl.p.mass * self.ctrl.p.gravity

        # State and Setpoint
        self.state = State()
        self.state_lock = threading.Lock()
        self.odom_received = False

        self.sp = Setpoint(
            x=0.0, y=0.0, z=self.takeoff_alt if self.armed else 0.0, yaw=0.0,
            roll=0.0, pitch=0.0, thrust=0.0 if not self.armed else self.hover_thrust, yaw_rate=0.0
        )

        # Propeller rotation accumulation for RViz visual spinning
        self.prop_left_angle = 0.0
        self.prop_right_angle = 0.0

        # MAVLink Server for QGroundControl
        self.mavlink = MavlinkServer()
        self.start_boot_time = time.time()
        self.last_heartbeat_time = 0.0
        self.last_hud_time = 0.0

        # ROS 2 Publishers
        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_telemetry = self.create_publisher(String, "/telemetry", 10)
        self.pub_thrust_markers = self.create_publisher(MarkerArray, "/thrust_markers", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS 2 Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.sub_cmd_att = self.create_subscription(
            Twist, "/cmd_attitude_thrust", self.cmd_attitude_thrust_callback, 10
        )
        self.sub_setpoint = self.create_subscription(PoseStamped, "/setpoint_position", self.setpoint_callback, 10)
        self.sub_arm = self.create_subscription(Bool, "/arm", self.arm_callback, 10)
        self.sub_mode = self.create_subscription(String, "/mode", self.mode_callback, 10)

        # Gazebo Transport Node & Topics
        self.gz_node = gz_transport.Node()
        odom_topic = f"/model/{self.model_name}/odometry"
        motor_topic = f"/{self.model_name}/command/motor_speed"
        servo0_topic = f"/model/{self.model_name}/servo_0"
        servo1_topic = f"/model/{self.model_name}/servo_1"

        self.gz_pub_motor = self.gz_node.advertise(motor_topic, actuators_pb2.Actuators)
        self.gz_pub_servo0 = self.gz_node.advertise(servo0_topic, double_pb2.Double)
        self.gz_pub_servo1 = self.gz_node.advertise(servo1_topic, double_pb2.Double)
        self.gz_pub_marker = self.gz_node.advertise("/marker", gz_marker_pb2.Marker)

        self.gz_sub_odom = self.gz_node.subscribe(
            odometry_pb2.Odometry, odom_topic, self.gz_odom_callback
        )

        # High-frequency control timer (100 Hz)
        timer_period = 1.0 / self.control_rate
        self.last_update_time = time.time()
        self.timer = self.create_timer(timer_period, self.control_loop)

        mode_name = "ATTITUDE & THRUST (Pixhawk Stabilized)" if not self.ctrl.position_hold else "POSITION HOLD"
        self.get_logger().info(
            f"Bi-copter Controller ready! Mode: [{mode_name}] | Hover Thrust: {self.hover_thrust:.2f}N | QGC MAVLink: UDP 14550"
        )

    # ------------------------------------------------------------- #
    # Gazebo Odometry Callback
    # ------------------------------------------------------------- #
    def gz_odom_callback(self, msg: odometry_pb2.Odometry):
        with self.state_lock:
            p = msg.pose.position
            q = msg.pose.orientation
            v = msg.twist.linear
            w = msg.twist.angular
            self.state.pos = (p.x, p.y, p.z)
            self.state.quat = (q.w, q.x, q.y, q.z)
            self.state.vel = (v.x, v.y, v.z)
            self.state.gyro = (w.x, w.y, w.z)
            self.odom_received = True

    # ------------------------------------------------------------- #
    # Control Loop (100 Hz: Attitude PID -> Actuators -> RViz & QGC)
    # ------------------------------------------------------------- #
    def control_loop(self):
        if not self.odom_received:
            return

        now = time.time()
        dt = max(1e-4, min(0.05, now - self.last_update_time))
        self.last_update_time = now

        # 1. Process incoming commands from QGroundControl
        arm_req, manual_ctrl = self.mavlink.poll()
        if arm_req is not None:
            self.armed = arm_req
            self.get_logger().info(f"QGroundControl changed arm state to: {self.armed}")
            if not self.armed:
                self.ctrl.reset()

        if manual_ctrl is not None and not self.ctrl.position_hold:
            x, y, z, r = manual_ctrl
            # x = pitch forward/back, y = roll right/left, z = throttle, r = yaw
            self.sp.pitch = clamp(x * self.ctrl.g.max_tilt_angle, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)
            self.sp.roll = clamp(y * self.ctrl.g.max_tilt_angle, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)
            self.sp.thrust = clamp(z * (2.0 * self.hover_thrust), 0.0, 10.0)
            self.sp.yaw_rate = r * self.ctrl.g.max_rate_yaw

        with self.state_lock:
            s = State(self.state.pos, self.state.vel, self.state.quat, self.state.gyro)

        # 2. Run cascaded PID controller
        out = self.ctrl.update(s, self.sp, dt, armed=self.armed)

        # 3. Publish physical commands to Gazebo Sim
        act = actuators_pb2.Actuators()
        act.velocity.extend(out.motor_speed)
        self.gz_pub_motor.publish(act)

        d0 = double_pb2.Double()
        d0.data = out.servo[0]
        self.gz_pub_servo0.publish(d0)

        d1 = double_pb2.Double()
        d1.data = out.servo[1]
        self.gz_pub_servo1.publish(d1)

        # 4. Joint states for RViz articulation (servos tilt + props spin)
        self.prop_left_angle = (self.prop_left_angle + out.motor_speed[0] * dt) % (2.0 * math.pi)
        self.prop_right_angle = (self.prop_right_angle + out.motor_speed[1] * dt) % (2.0 * math.pi)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_tilt_joint", "right_tilt_joint", "left_prop_joint", "right_prop_joint"]
        js.position = [out.servo[0], out.servo[1], self.prop_left_angle, self.prop_right_angle]
        js.velocity = [0.0, 0.0, out.motor_speed[0], out.motor_speed[1]]
        self.pub_joint_states.publish(js)

        # 5. Odometry & TF
        ros_odom = Odometry()
        ros_odom.header.stamp = js.header.stamp
        ros_odom.header.frame_id = "odom"
        ros_odom.child_frame_id = "base_footprint"
        ros_odom.pose.pose.position.x = s.pos[0]
        ros_odom.pose.pose.position.y = s.pos[1]
        ros_odom.pose.pose.position.z = s.pos[2]
        ros_odom.pose.pose.orientation.w = s.quat[0]
        ros_odom.pose.pose.orientation.x = s.quat[1]
        ros_odom.pose.pose.orientation.y = s.quat[2]
        ros_odom.pose.pose.orientation.z = s.quat[3]
        ros_odom.twist.twist.linear.x = s.vel[0]
        ros_odom.twist.twist.linear.y = s.vel[1]
        ros_odom.twist.twist.linear.z = s.vel[2]
        ros_odom.twist.twist.angular.x = s.gyro[0]
        ros_odom.twist.twist.angular.y = s.gyro[1]
        ros_odom.twist.twist.angular.z = s.gyro[2]
        self.pub_odom.publish(ros_odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = js.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = s.pos[0]
            t.transform.translation.y = s.pos[1]
            t.transform.translation.z = s.pos[2]
            t.transform.rotation.w = s.quat[0]
            t.transform.rotation.x = s.quat[1]
            t.transform.rotation.y = s.quat[2]
            t.transform.rotation.z = s.quat[3]
            self.tf_broadcaster.sendTransform(t)

        # 6. Simulate & Publish Dynamic Thrust Force Vectors (RViz + Gazebo)
        self.publish_thrust_visuals(out, js.header.stamp, s)

        # 7. Broadcast MAVLink Telemetry to QGroundControl
        boot_ms = (now - self.start_boot_time) * 1000.0
        roll, pitch, yaw = out.rpy
        # Attitude at 20 Hz
        if int(now * 20) % 2 == 0:
            self.mavlink.send_attitude(boot_ms, roll, pitch, yaw, s.gyro[0], s.gyro[1], s.gyro[2])
        # VFR_HUD at 10 Hz
        if now - self.last_hud_time > 0.10:
            self.last_hud_time = now
            throttle_pct = int((out.collective / (2.0 * self.hover_thrust)) * 100.0) if self.armed else 0
            groundspeed = math.hypot(s.vel[0], s.vel[1])
            heading = int(math.degrees(yaw)) % 360
            self.mavlink.send_vfr_hud(groundspeed, groundspeed, heading, throttle_pct, s.pos[2], s.vel[2])
        # Heartbeat at 1 Hz
        if now - self.last_heartbeat_time > 1.0:
            self.last_heartbeat_time = now
            self.mavlink.send_heartbeat(self.armed)
            self.mavlink.send_sys_status()

        # 8. ROS 2 Telemetry JSON summary
        if int(now * 10) % 2 == 0:
            telem = {
                "mode": "ATTITUDE_THRUST" if not self.ctrl.position_hold else "POSITION_HOLD",
                "armed": self.armed,
                "altitude": round(s.pos[2], 3),
                "thrust_N": round(out.collective, 2),
                "rpy_deg": [round(math.degrees(r), 2) for r in out.rpy],
                "motor_rad_s": [round(m, 1) for m in out.motor_speed],
                "servos_deg": [round(math.degrees(sv), 2) for sv in out.servo],
                "qgc_connected": self.mavlink.connected,
            }
            msg_str = String()
            msg_str.data = json.dumps(telem)
            self.pub_telemetry.publish(msg_str)

    # ------------------------------------------------------------- #
    # Dynamic Thrust & Servo Angle Visualization
    # ------------------------------------------------------------- #
    def publish_thrust_visuals(self, out, stamp, s):
        """Simulate and render live thrust force vectors & servo angle HUD."""
        t0, t1 = out.thrust[0], out.thrust[1]
        deg0, deg1 = math.degrees(out.servo[0]), math.degrees(out.servo[1])
        scale_m = 0.08  # ~0.19m vector length at hover (2.36 N)
        l0 = max(0.02, t0 * scale_m) if self.armed else 0.001
        l1 = max(0.02, t1 * scale_m) if self.armed else 0.001

        m_array = MarkerArray()

        # Left Rotor Thrust Arrow (attached to left_prop_link)
        m_l = Marker()
        m_l.header.stamp = stamp
        m_l.header.frame_id = "left_prop_link"
        m_l.ns = "thrust_force"
        m_l.id = 0
        m_l.type = Marker.ARROW
        m_l.action = Marker.ADD
        m_l.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=float(l0))]
        m_l.scale.x = 0.018
        m_l.scale.y = 0.035
        m_l.scale.z = 0.045
        m_l.color.r, m_l.color.g, m_l.color.b, m_l.color.a = 0.0, 0.85, 1.0, (0.9 if self.armed else 0.2)
        m_array.markers.append(m_l)

        # Right Rotor Thrust Arrow (attached to right_prop_link)
        m_r = Marker()
        m_r.header.stamp = stamp
        m_r.header.frame_id = "right_prop_link"
        m_r.ns = "thrust_force"
        m_r.id = 1
        m_r.type = Marker.ARROW
        m_r.action = Marker.ADD
        m_r.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=float(l1))]
        m_r.scale.x = 0.018
        m_r.scale.y = 0.035
        m_r.scale.z = 0.045
        m_r.color.r, m_r.color.g, m_r.color.b, m_r.color.a = 1.0, 0.55, 0.0, (0.9 if self.armed else 0.2)
        m_array.markers.append(m_r)

        # Floating HUD Text
        m_txt0 = Marker()
        m_txt0.header.stamp = stamp
        m_txt0.header.frame_id = "left_prop_link"
        m_txt0.ns = "rotor_hud"
        m_txt0.id = 2
        m_txt0.type = Marker.TEXT_VIEW_FACING
        m_txt0.action = Marker.ADD
        m_txt0.pose.position.z = float(l0 + 0.06)
        m_txt0.scale.z = 0.040
        m_txt0.text = f"LEFT: {t0:.2f}N ({deg0:+.1f}°)"
        m_txt0.color.r, m_txt0.color.g, m_txt0.color.b, m_txt0.color.a = 0.1, 0.9, 1.0, 1.0
        m_array.markers.append(m_txt0)

        m_txt1 = Marker()
        m_txt1.header.stamp = stamp
        m_txt1.header.frame_id = "right_prop_link"
        m_txt1.ns = "rotor_hud"
        m_txt1.id = 3
        m_txt1.type = Marker.TEXT_VIEW_FACING
        m_txt1.action = Marker.ADD
        m_txt1.pose.position.z = float(l1 + 0.06)
        m_txt1.scale.z = 0.040
        m_txt1.text = f"RIGHT: {t1:.2f}N ({deg1:+.1f}°)"
        m_txt1.color.r, m_txt1.color.g, m_txt1.color.b, m_txt1.color.a = 1.0, 0.7, 0.1, 1.0
        m_array.markers.append(m_txt1)

        self.pub_thrust_markers.publish(m_array)

    # ------------------------------------------------------------- #
    # Command Callbacks
    # ------------------------------------------------------------- #
    def cmd_vel_callback(self, msg: Twist):
        """Teleoperation: Attitude & Thrust mode (Pixhawk) or Position integration."""
        dt = 0.05
        if not self.ctrl.position_hold:
            # Pixhawk Stabilized Mode:
            # Support both angular.x/y and linear.x/y
            if abs(msg.angular.x) > 1e-4 or abs(msg.angular.y) > 1e-4:
                roll_in = msg.angular.x
                pitch_in = msg.angular.y
            else:
                roll_in = msg.linear.x
                pitch_in = msg.linear.y

            self.sp.roll = clamp(roll_in, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)
            self.sp.pitch = clamp(pitch_in, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)

            # Vertical control: Altitude hold rate or manual collective thrust
            if self.ctrl.altitude_hold:
                if abs(msg.linear.z) > 1e-3:
                    self.sp.z = clamp(self.sp.z + msg.linear.z * dt * 1.5, 0.1, 8.0)
                self.sp.thrust = 0.0
            else:
                if msg.linear.z > 1.0:
                    self.sp.thrust = clamp(msg.linear.z, 0.0, 10.0)
                elif abs(msg.linear.z) > 1e-3:
                    self.sp.thrust = clamp(self.sp.thrust + msg.linear.z * dt * 4.0, 0.0, 10.0)

            self.sp.yaw_rate = msg.angular.z
            self.sp.yaw = (self.sp.yaw + msg.angular.z * dt + math.pi) % (2.0 * math.pi) - math.pi
        else:
            with self.state_lock:
                _, _, current_yaw = quat_to_euler(*self.state.quat)
            vx_w = math.cos(current_yaw) * msg.linear.x - math.sin(current_yaw) * msg.linear.y
            vy_w = math.sin(current_yaw) * msg.linear.x + math.cos(current_yaw) * msg.linear.y
            self.sp.x += vx_w * dt
            self.sp.y += vy_w * dt
            self.sp.z = clamp(self.sp.z + msg.linear.z * dt, 0.1, 10.0)
            self.sp.yaw = (self.sp.yaw + msg.angular.z * dt + math.pi) % (2.0 * math.pi) - math.pi

    def cmd_attitude_thrust_callback(self, msg: Twist):
        """Direct Pixhawk Attitude + Thrust input:
        linear.x or angular.x = roll [rad]
        linear.y or angular.y = pitch [rad]
        linear.z = collective thrust [N] (e.g. 4.72 N for hover) or target altitude if in altitude mode
        angular.z = yaw rate [rad/s]
        """
        roll = msg.angular.x if abs(msg.angular.x) > 1e-4 else msg.linear.x
        pitch = msg.angular.y if abs(msg.angular.y) > 1e-4 else msg.linear.y
        self.sp.roll = clamp(roll, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)
        self.sp.pitch = clamp(pitch, -self.ctrl.g.max_tilt_angle, self.ctrl.g.max_tilt_angle)
        if msg.linear.z > 0.0:
            if self.ctrl.altitude_hold and msg.linear.z <= 6.0:
                self.sp.z = clamp(msg.linear.z, 0.1, 8.0)
                self.sp.thrust = 0.0
            else:
                self.sp.thrust = clamp(msg.linear.z, 0.0, 10.0)
        self.sp.yaw_rate = msg.angular.z

    def setpoint_callback(self, msg: PoseStamped):
        """Waypoint navigation."""
        self.sp.x = msg.pose.position.x
        self.sp.y = msg.pose.position.y
        self.sp.z = max(0.1, msg.pose.position.z)
        q = msg.pose.orientation
        if abs(q.w**2 + q.x**2 + q.y**2 + q.z**2 - 1.0) < 0.1:
            _, _, yaw = quat_to_euler(q.w, q.x, q.y, q.z)
            self.sp.yaw = yaw

    def arm_callback(self, msg: Bool):
        self.armed = msg.data
        self.get_logger().info(f"Armed status changed to: {self.armed}")
        if not self.armed:
            self.ctrl.reset()

    def mode_callback(self, msg: String):
        mode = msg.data.upper()
        if mode in ["ALTITUDE", "ALTCTL"]:
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = True
            with self.state_lock:
                self.sp.z = max(0.5, self.state.pos[2])
            self.sp.thrust = 0.0
            self.sp.roll = 0.0
            self.sp.pitch = 0.0
            self.sp.yaw_rate = 0.0
            self.get_logger().info(f"Switched to ALTITUDE mode (Pixhawk ALTCTL - Auto-Hold at {self.sp.z:.2f}m)")
        elif mode in ["ATTITUDE", "STABILIZED", "MANUAL", "THRUST"]:
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = False
            self.sp.thrust = self.hover_thrust
            self.sp.roll = 0.0
            self.sp.pitch = 0.0
            self.sp.yaw_rate = 0.0
            self.get_logger().info("Switched to ATTITUDE & THRUST mode (Pixhawk Stabilized - Direct Manual Throttle)")
        elif mode in ["POSITION", "POSHOLD"]:
            self.ctrl.position_hold = True
            self.ctrl.altitude_hold = True
            with self.state_lock:
                self.sp.x = self.state.pos[0]
                self.sp.y = self.state.pos[1]
                self.sp.z = max(0.3, self.state.pos[2])
            self.get_logger().info("Switched to POSITION HOLD mode")
        elif mode == "TAKEOFF":
            self.armed = True
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = True
            self.sp.z = self.takeoff_alt
            self.sp.thrust = 0.0
            self.sp.roll = 0.0
            self.sp.pitch = 0.0
            self.sp.yaw_rate = 0.0
            self.get_logger().info(f"Commanded TAKEOFF to {self.takeoff_alt}m hover")
        elif mode == "LAND":
            self.ctrl.position_hold = False
            self.ctrl.altitude_hold = True
            self.sp.z = 0.02
            self.sp.thrust = 0.0
            self.get_logger().info("Commanded LAND")


def main(args=None):
    rclpy.init(args=args)
    node = BiCopterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
