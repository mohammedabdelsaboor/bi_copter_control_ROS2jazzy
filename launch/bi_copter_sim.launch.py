"""
================================================================================
BI-COPTER SIMULATION LAUNCHER (Master Simulation Orchestrator)
================================================================================

Hello! This launch file is the master orchestrator for our bi-copter system.
With a single command:
    $ ros2 launch bi_copter_control bi_copter_sim.launch.py

It spins up all the core subsystems needed for a realistic flight test:

1. Gazebo Sim (Harmonic Physics Engine):
   Loads the 3D world with physics ground plane and spawns the bi-copter model
   with its 2 tilting servo joints and 2 counter-rotating aerodynamic thrusters.

2. Robot State Publisher (RSP):
   Reads our URDF robot description and publishes the live TF coordinate frame
   hierarchy (base_link, servos, motors, sensors) for RViz2.

3. Bi-Copter Flight Controller & Bridge (bicopter_controller_node.py):
   Runs the high-speed 100 Hz cascaded PID loops (altitude, attitude, rates),
   converts torques to motor RPM / servo tilt angles, and broadcasts live
   MAVLink telemetry to QGroundControl on UDP 14550.

4. RViz2 Visualization (Optional GUI):
   Renders the robot model in 3D alongside live dynamic thrust force vectors
   (colored arrows showing magnitude and tilt angle) and IMU attitude.

Configurable Launch Arguments:
------------------------------
- headless          : Set to 'true' to run Gazebo in background without a 3D window (saves CPU/GPU).
- rviz              : Set to 'false' if you prefer to view the drone only in Gazebo.
- takeoff_altitude  : Target hover altitude in meters (default: 1.0 m).
- flight_mode       : 'altitude' (Pixhawk ALTCTL - auto-holds height, manual roll/pitch sticks),
                      'attitude_thrust' (manual throttle), or 'position'.
- armed             : 'false' (default) starts safely disarmed on the launch pad with 0 N thrust.
================================================================================
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workspace_src = os.path.dirname(pkg_dir)
    models_dir = os.path.join(pkg_dir, "models")
    urdf_path = os.path.join(pkg_dir, "urdf", "bi_copter.urdf")
    world_path = os.path.join(pkg_dir, "worlds", "bicopter_world.sdf")
    rviz_config_path = os.path.join(pkg_dir, "config", "bicopter.rviz")
    controller_script = os.path.join(
        pkg_dir, "bi_copter_control", "bicopter_controller_node.py"
    )

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    resource_paths = f"{workspace_src}:{models_dir}:{pkg_dir}"

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo headless without GUI",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 visualization",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="1.0",
                description="Target takeoff altitude [m]",
            ),
            DeclareLaunchArgument(
                "flight_mode",
                default_value="altitude",
                description="Control mode: altitude (Pixhawk Altitude Mode - manual roll/pitch, auto-hold height), attitude_thrust (manual throttle), or position",
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Arm motors immediately on launch (false starts safely on ground)",
            ),
            # Environment variables for Gazebo Sim
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_paths),
            SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1"),
            # Gazebo GUI
            ExecuteProcess(
                condition=UnlessCondition(LaunchConfiguration("headless")),
                cmd=["gz", "sim", "-r", world_path],
                output="screen",
            ),
            # Gazebo Headless
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration("headless")),
                cmd=["gz", "sim", "-s", "-r", world_path],
                output="screen",
            ),
            # Robot State Publisher (publishes /tf and /robot_description for RViz)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            # Bi-copter Controller & Bridge Node
            Node(
                package="bi_copter_control",
                executable="bicopter_controller_node.py",
                name="bicopter_controller",
                output="screen",
                parameters=[
                    {"flight_mode": LaunchConfiguration("flight_mode")},
                    {"takeoff_altitude": LaunchConfiguration("takeoff_altitude")},
                    {"armed_on_start": LaunchConfiguration("armed")},
                ],
            ),
            # RViz2 Node
            Node(
                condition=IfCondition(LaunchConfiguration("rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
