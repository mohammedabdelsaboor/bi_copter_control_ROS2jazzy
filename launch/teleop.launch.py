"""
================================================================================
BI-COPTER TELEOPERATION LAUNCHER
================================================================================

Welcome pilot! This launch file starts the interactive keyboard teleoperation
console for our bi-copter drone:
    $ ros2 launch bi_copter_control teleop.launch.py

What it does:
------------
1. Launches `bicopter_teleop_node.py` as an interactive ROS 2 node.
2. Enables `emulate_tty=True` to preserve ANSI terminal formatting, vibrant
   status colors, and real-time cursor updates.
3. Connects your keyboard sticks (Pitch, Roll, Yaw, Altitude) directly to
   the flight controller's `/cmd_attitude_thrust` and `/arm` topics.

Tip: Run this in a dedicated terminal window while `bi_copter_sim.launch.py`
is running in another window.
================================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bi_copter_control",
                executable="bicopter_teleop_node.py",
                name="bicopter_teleop",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
