"""
================================================================================
BI-COPTER CONTROL PYTHON MODULE
================================================================================

This module provides the core control logic, PID controllers, actuator mixer,
and ROS 2 nodes for the dual-tilt bi-copter drone.

Modules:
- `flight_core`: Pure Python flight stabilization physics and 4-DOF mixer.
- `bicopter_controller_node`: 100 Hz ROS 2 controller, Gazebo bridge, QGC MAVLink.
- `bicopter_teleop_node`: Interactive RC keyboard transmitter with ASCII HUD.
================================================================================
"""
