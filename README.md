# 🚁 Bi-Copter Autonomous Flight Control & Simulation
### ROS 2 Jazzy Jalisco • Gazebo Sim (Harmonic) • PX4 Autopilot Architecture • QGroundControl

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-Harmonic-FF6F00.svg?logo=gazebo)](https://gazebosim.org/)
[![PX4](https://img.shields.io/badge/PX4-SITL%20Ready-blue.svg?logo=drone)](https://px4.io/)
[![Unit Tests](https://img.shields.io/badge/Tests-10%2F10%20Passing-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

---

## 👋 Welcome to the Bi-Copter Project!

Hey there! Welcome to the **Bi-Copter Control & Simulation** repository. 

While traditional quadcopters use four fixed propellers, a **bi-copter (dual-tilt rotor aircraft)** achieves full 4-DOF flight using only **two propellers and two tilting servos**. By vectoring thrust dynamically, it is mechanically lean, aerodynamically efficient, and one of the most exciting, challenging control problems in modern aerial robotics.

This repository provides an end-to-end, production-grade flight software stack and physics simulation built for **ROS 2 Jazzy** and **Gazebo Sim (Harmonic)**, modeled directly after **Pixhawk / PX4 flight control architecture** and integrated with **QGroundControl**.

---

## 🎯 What Was Done in This Project

Here is a summary of the engineering work accomplished in this project:

### 1. Pure-Python Cascaded Flight Control Engine (`flight_core.py`)
- **Zero-Dependency Architecture**: Separated flight physics from ROS middleware so control math can be verified in milliseconds or ported to embedded microcontrollers.
- **Cascaded PID Loops**:
  - Outer-loop **Attitude P Controller**: Translates commanded roll/pitch angles into target angular rates.
  - Inner-loop **Rate PID Controller**: Compares gyro angular velocities against desired rates to output 3D body torques ($\tau_x, \tau_y, \tau_z$), equipped with derivative low-pass filtering and anti-windup clamping.
- **Physical 4-DOF Actuator Mixer**:
  - **Altitude ($Z$)**: Collective thrust ($T_0 + T_1$) via synchronized motor RPM.
  - **Roll ($\Phi$)**: Differential rotor thrust ($T_0 - T_1$) across the 0.322 m arm span.
  - **Pitch ($\Theta$)**: Common servo tilt ($\delta_0 = \delta_1$), pitching the thrust vector forward/backward.
  - **Yaw ($\Psi$)**: Differential servo tilt ($\delta_1 - \delta_0$) generating an instant horizontal couple, balanced against counter-rotating propeller reaction drag torques.

### 2. Rock-Solid Altitude Hold (`ALTCTL` Mode)
- Eliminated vertical runaway climbing ("shooting up into the sky") by adding active aerodynamic velocity damping ($-0.40 \cdot v_z$).
- Implemented ground-contact safety cuts: motors remain safely at 0 N until armed takeoff is commanded.
- Holds altitude at a steady 1.0 m hover while giving the pilot 100% manual stick control over roll and pitch without artificial GPS coordinate locks.

### 3. High-Fidelity Physics & Aerodynamics in Gazebo Harmonic
- **Aerodynamic Thrusters**: Simulated with `gz::sim::systems::MulticopterMotorModel` using quadratic thrust ($F = k_F \cdot \omega^2$) and rotor reaction torque ($M = k_M \cdot F$).
- **Articulated Tilt Servos**: Driven by velocity-limited `gz::sim::systems::JointPositionController` to replicate high-torque digital RC servos ($\pm 30^\circ$).
- **Clean Indoor Arena**: Designed an unobstructed safety flight cage (`bicopter_world.sdf`) with a uniform, solid floor free of distracting drawings or decals.

### 4. Live 3D Force & HUD Visualization
- **RViz2 Dynamic Thrust Arrows**: Color-coded 3D vectors attached to the motor links (Cyan for Left, Amber for Right) that scale dynamically with real-time Newtons.
- **Floating Telemetry Badges**: Displays live thrust and tilt angles in 3D space (e.g. `LEFT: 2.36N (+5.2°)`).

### 5. Seamless QGroundControl (QGC) Telemetry Bridge
- Embedded lightweight **MAVLink server over UDP 14550**.
- Opening QGroundControl connects to the simulated bi-copter instantly as Vehicle 1.
- Provides a live Artificial Horizon / Primary Flight Display (PFD), compass heading, climb rate, and battery voltage monitoring.

### 6. Interactive RC Keyboard Transmitter (`bicopter_teleop_node.py`)
- **Direct Terminal Control (`/dev/tty`)**: Bypasses standard ROS launch pipe buffering for zero-latency stick response.
- **Mode 2 Spring-Centered Sticks**: Pitch, roll, and yaw automatically spring back to 0° level when keys are released.
- **Auto-Takeoff**: Simply pressing any flight key (`W`, `A`, `S`, `D`, `T`) from the ground spools up the motors smoothly and lifts off to a 1.0 m hover.
- **Real-Time ASCII HUD**: Live bottom-line terminal readout displaying stick positions, drone attitude, altitude, motor RPM, and servo deflections.

### 7. Automated Test Suite (10/10 Passing)
- Fast pytest test suite (`test_flight_core.py`) verifying math primitives, quaternion rotations, PID anti-windup, hover equilibrium, mixer physics, and actuator saturation limits in just 20 milliseconds.

---

## 📐 Vehicle Specifications

| Parameter | Value | Details |
| :--- | :--- | :--- |
| **Total Mass** | `0.481 kg` | Extracted from SolidWorks CAD assembly densities |
| **Arm Half-Span ($d$)** | `0.161 m` | Centerline to rotor axis (total span: 0.322 m) |
| **Tilt Arm Offset ($l_p$)** | `0.017 m` | Distance from servo pivot to propeller hub |
| **Propellers** | `10x5.5 inch` | Rotor 0: CCW (Left) • Rotor 1: CW (Right) |
| **Hover Thrust** | `4.72 N` | ~2.36 N per motor at ~627 rad/s (~6000 RPM) |
| **Servo Tilt Range** | `±30°` | `±0.524 rad` about the transverse Y axis |
| **Nominal Flight Mode** | `ALTCTL` | Pixhawk Altitude Hold Mode (Manual roll/pitch, auto-hold height) |

---

## 📂 Clean Repository Structure

```
bi_copter_control/
├── CMakeLists.txt              # ROS 2 Jazzy build & installation configuration
├── package.xml                 # Package metadata and ROS 2 dependencies
├── README.md                   # Humanoid documentation & quick-start guide
├── .gitignore                  # Clean repository filter (ignores build, install, log, caches)
├── bi_copter_control/          # Core Python modules
│   ├── __init__.py
│   ├── flight_core.py          # Cascaded PID controller + 4-DOF bi-copter mixer (Pure Python)
│   ├── bicopter_controller_node.py # 100 Hz ROS 2 node + Gazebo bridge + QGC MAVLink server
│   └── bicopter_teleop_node.py   # Interactive Mode 2 RC keyboard transmitter with ASCII HUD
├── config/
│   └── bicopter.rviz           # Pre-configured RViz2 layout (TF + Model + Thrust Vectors)
├── launch/
│   ├── bi_copter_sim.launch.py # Master launcher: Gazebo Sim + RSP + Controller + RViz2
│   └── teleop.launch.py        # Teleop launcher with emulate_tty=True
├── models/
│   └── bi_copter/              # Gazebo SDF physics model with motor & servo plugins
├── meshes/                     # CAD STL meshes (base_link, brackets L1..L3, propellers L4..L5)
├── urdf/
│   └── bi_copter.urdf          # Kinematic description for Robot State Publisher & RViz2
├── worlds/
│   └── bicopter_world.sdf      # Clean safety net flight arena with solid floor
├── test/
│   └── test_flight_core.py     # 10/10 automated physics & mixer verification tests
└── pixhawk_px4/                # PX4 Autopilot SITL & QGroundControl integration
    ├── 4005_bicopter           # Official PX4 ROMFS airframe definition (CA_AIRFRAME = 8)
    ├── bicopter.params         # Parameter presets for QGroundControl
    ├── README_PIXHAWK.md       # PX4 integration guide
    ├── run_px4_sitl.sh         # Standalone PX4 SITL launcher
    └── setup_px4_sitl.sh       # Turnkey PX4 SITL setup script
```

---

## 🚀 Quick Start Guide

### 1. Build the Package
```bash
source /opt/ros/jazzy/setup.bash
cd ~/graduation/bi_copter_control
colcon build --packages-select bi_copter_control --symlink-install
```

### 2. Launch the Master Simulation
In your primary terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/graduation/bi_copter_control/install/setup.bash
ros2 launch bi_copter_control bi_copter_sim.launch.py
```
*What happens:*
- **Gazebo Sim** opens with the safety flight cage.
- **RViz2** visualizes the bi-copter with dynamic, color-coded thrust force arrows.
- The drone sits safely disarmed on the launch floor (0 N thrust) awaiting your command.

### 3. Fly with the RC Keyboard Transmitter
In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/graduation/bi_copter_control/install/setup.bash
ros2 launch bi_copter_control teleop.launch.py
```
*Simply press **`W`**, **`A`**, **`S`**, **`D`**, or **`T`** to automatically arm the motors and lift off to a stable 1.0 m hover!*

### 4. Connect QGroundControl (Optional)
1. Open **QGroundControl** on your computer.
2. It will automatically detect the bi-copter on UDP port `14550`.
3. View the live **Primary Flight Display (PFD) / Artificial Horizon**, monitor climb rate, or fly with a USB gamepad.

### 5. Run the Automated Test Suite
Verify all physics, mixer mathematics, and PID limits in 20 milliseconds:
```bash
PYTHONPATH=bi_copter_control pytest bi_copter_control/test/test_flight_core.py
```

---

## 🎮 Flight Stick Controls

The teleoperation console behaves just like the spring-centered gimbals on a real Mode 2 RC transmitter:

| Key | RC Stick | Action |
| :---: | :--- | :--- |
| **`W`** / **`S`** (or **`↑`** / **`↓`**) | **Pitch Stick** | Pitch Forward (Nose Down) / Pitch Backward (Nose Up). *Spring-centers to 0°*. |
| **`A`** / **`D`** (or **`←`** / **`→`**) | **Roll Stick** | Bank Left / Bank Right via differential rotor thrust. *Spring-centers to 0°*. |
| **`R`** / **`F`** | **Altitude Stick** | Climb UP (+0.20 m) / Descend DOWN (-0.20 m). Locks height automatically. |
| **`Q`** / **`E`** | **Yaw Stick** | Rotate Heading Left / Right at 1.0 rad/s. *Spring-centers to 0 rad/s*. |
| **`Space`** / **`H`** | **Level Out** | Instantly zeroes tilt angles and locks into a stable 1.0 m hover. |
| **`T`** | **Auto-Takeoff** | Smoothly spools up motors and climbs to 1.0 m hover. |
| **`L`** | **Auto-Land** | Gentle controlled descent until touchdown, followed by automatic motor disarm. |
| **`M`** | **Arm / Disarm** | Toggle motor arming state safely. |
| **`X`** | **Kill Switch** | Emergency motor cutoff (instant 0 N). |
| **`+`** / **`-`** | **Sensitivity** | Increase / decrease maximum stick tilt angle (±1.0°). |

---

## 🧑‍💻 Author & Contact

Developed with care by **Mohammed Abdelsabour**  
- **Email**: mohammedabdelsaboor@gmail.com  
- **GitHub**: [@mohammedabdelsaboor](https://github.com/mohammedabdelsaboor)  
- **Repository**: [bi_copter_control_ROS2jazzy](https://github.com/mohammedabdelsaboor/bi_copter_control_ROS2jazzy)
