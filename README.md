# Bi-Copter Autonomous Flight Control and Simulation System
### ROS 2 Jazzy Jalisco | Gazebo Sim (Harmonic) | PX4 Autopilot Architecture | QGroundControl

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-Harmonic-FF6F00.svg)](https://gazebosim.org/)
[![PX4](https://img.shields.io/badge/PX4-SITL%20Ready-blue.svg)](https://px4.io/)
[![Unit Tests](https://img.shields.io/badge/Tests-10%2F10%20Passing-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

---

## Overview

This repository contains an end-to-end flight control, dynamics simulation, and teleoperation framework for an under-actuated **dual-tilt bi-copter** unmanned aerial vehicle (UAV).

Unlike conventional quadcopters that rely on four or more coplanar rotors, a bi-copter achieves complete four-degree-of-freedom (4-DOF) attitude and altitude authority using only **two counter-rotating propellers and two single-axis tilt servos**. This configuration reduces vehicle weight and aerodynamic drag while introducing non-linear cross-axis coupling that requires precise control allocation and dynamic vectoring.

The system is developed on **ROS 2 Jazzy Jalisco**, simulated with high physical fidelity in **Gazebo Sim (Harmonic)**, structured around the **Pixhawk / PX4 Autopilot** control architecture, and integrated with **QGroundControl** via native MAVLink telemetry.

---

## Project Accomplishments and Technical Architecture

### 1. Cascaded Flight Control Engine (`flight_core.py`)
- **Independent Core Implementation**: Designed as a pure Python engine with zero ROS dependencies to facilitate rapid unit testing, deterministic profiling, and cross-platform embedded portability.
- **Cascaded Loop Topology**:
  - Outer Loop (**Attitude Proportional Controller**): Computes target angular velocities from commanded roll and pitch setpoints.
  - Inner Loop (**Angular Rate PID Controller**): Computes required three-axis body torques ($\tau_x, \tau_y, \tau_z$) based on gyro rate errors, featuring derivative low-pass filtering and anti-windup integration bounds.
- **Physical Actuator Allocation Matrix**:
  - **Altitude ($Z$)**: Collective thrust modulation ($T_0 + T_1$) via synchronized rotor speeds.
  - **Roll ($\Phi$)**: Differential rotor thrust ($T_0 - T_1$) across the transverse moment arm ($2d = 0.322\,\text{m}$).
  - **Pitch ($\Theta$)**: Common servo deflection ($\delta_0 = \delta_1$), rotating the net thrust vector longitudinally.
  - **Yaw ($\Psi$)**: Differential servo deflection ($\delta_1 - \delta_0$) generating a horizontal turning couple, balanced against rotor aerodynamic reaction torque.

### 2. Altitude Hold Stabilization (`ALTCTL` Mode)
- **Active Vertical Damping**: Implemented aerodynamic velocity damping ($-0.40 \cdot v_z$) to counteract vertical runaway climb and maintain a stable 1.0 m hover.
- **Ground Safety Logic**: Automatic motor cutoff prevents ground spin-up until an explicit takeoff or arming command is executed.
- **Decoupled Manual Flight**: Provides full manual authority over pitch and roll angles while the autopilot automatically maintains target height without horizontal GPS position locks.

### 3. High-Fidelity Physics and Aerodynamics Simulation
- **Rotor Dynamics**: Modeled in Gazebo Harmonic using `gz::sim::systems::MulticopterMotorModel` with quadratic thrust generation ($F = k_F \cdot \omega^2$) and rotor reaction torque ($M = k_M \cdot F$).
- **Servo Actuation**: Simulated using velocity-constrained `gz::sim::systems::JointPositionController` to reproduce digital servo kinematics with a $\pm 30^\circ$ physical travel limit.
- **Structured Arena Environment**: Developed an indoor flight facility world (`bicopter_world.sdf`) with a protective truss-net enclosure and a uniform, clean floor plane.

### 4. Live 3D Force Vector Visualization
- **RViz2 Dynamic Markers**: Real-time rendering of instantaneous rotor thrust vectors attached directly to the propeller links.
- **Force and Deflection Badges**: Visual indicators display real-time thrust magnitude in Newtons and servo deflection in degrees (e.g., `LEFT: 2.36N (+5.2°)`).

### 5. Ground Station Integration via MAVLink UDP
- **Embedded Telemetry Server**: Implemented an asynchronous MAVLink server broadcasting on UDP port `14550`.
- **QGroundControl Compatibility**: Automatic discovery by QGroundControl as Vehicle 1, streaming live Primary Flight Display (PFD) / Artificial Horizon data, heading, climb rate, and battery voltage status.

### 6. Interactive Terminal Teleoperation Console (`bicopter_teleop_node.py`)
- **Direct Terminal Interface (`/dev/tty`)**: Directly samples terminal input in raw mode to eliminate launch-pipeline buffering and provide immediate stick response.
- **Mode 2 RC Stick Emulation**: Pitch, roll, and yaw controls automatically spring-center to neutral (0°) upon key release.
- **Automated Maneuvers**: Single-key takeoff to 1.0 m hover, controlled landing with automated disarm, and emergency motor cutoff.
- **Terminal HUD**: Real-time ASCII status line displaying stick commands, drone orientation, altitude, motor speeds, and servo angles.

### 7. Automated Verification Suite
- Comprehensive pytest test suite (`test_flight_core.py`) validating math utilities, coordinate transformations, PID behavior, hover equilibrium, mixer physics, and saturation bounds with 10/10 tests passing in under 25 milliseconds.

---

## Vehicle Specifications

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Total Mass ($m$)** | `0.481 kg` | Derived from CAD solid model component densities |
| **Arm Half-Span ($d$)** | `0.161 m` | Centerline to rotor axis (total transverse span: 0.322 m) |
| **Servo Pivot Offset ($l_p$)** | `0.017 m` | Vertical offset from tilt pivot to propeller hub |
| **Propellers** | `10x5.5 inch` | Rotor 0: Counter-Clockwise (Left) / Rotor 1: Clockwise (Right) |
| **Hover Thrust ($T_{\text{hover}}$)** | `4.72 N` | Nominal hover requirement (~2.36 N per motor at ~627 rad/s) |
| **Maximum Servo Deflection** | `±30.0°` | `±0.524 rad` about the transverse body axis ($+Y$) |
| **Primary Control Mode** | `ALTCTL` | Pixhawk Altitude Control (Manual attitude, automated height hold) |

---

## Control Allocation Summary

The actuator mixer inverts the physical geometry to calculate individual rotor speeds ($\omega_0, \omega_1$) and servo angles ($\delta_0, \delta_1$):

$$\begin{aligned}
T_0 &= \frac{T_{\text{coll}}}{2} + \frac{\tau_x}{2d} \\
T_1 &= \frac{T_{\text{coll}}}{2} - \frac{\tau_x}{2d} \\
\delta_{\text{pitch}} &= \frac{\tau_y}{T_{\text{coll}} \cdot l_p} \\
\delta_{\text{yaw}} &= \frac{\tau_z}{T_{\text{coll}} \cdot d} \\
\delta_0 &= \delta_{\text{pitch}} - \delta_{\text{yaw}} \\
\delta_1 &= \delta_{\text{pitch}} + \delta_{\text{yaw}}
\end{aligned}$$

Rotor angular speeds are then computed via the aerodynamic lift constant:

$$\omega_i = \sqrt{\frac{T_i}{k_F}}$$

---

## Repository Structure

```
bi_copter_control/
├── CMakeLists.txt              # ROS 2 Jazzy build and install configuration
├── package.xml                 # Package manifest and dependencies
├── README.md                   # Technical documentation and user guide
├── .gitignore                  # Git exclusion rules
├── bi_copter_control/          # Python control modules
│   ├── __init__.py
│   ├── flight_core.py          # Cascaded PID controller and 4-DOF mixer (Pure Python)
│   ├── bicopter_controller_node.py # 100 Hz ROS 2 node, Gazebo bridge, QGC MAVLink server
│   └── bicopter_teleop_node.py   # Mode 2 RC keyboard teleoperation console
├── config/
│   └── bicopter.rviz           # Pre-configured RViz2 layout (TF, Robot Model, Force Vectors)
├── launch/
│   ├── bi_copter_sim.launch.py # Master launch: Gazebo Sim, RSP, Controller, RViz2
│   └── teleop.launch.py        # Interactive teleoperation launcher
├── models/
│   └── bi_copter/              # Gazebo SDF model with motor and servo plugins
├── meshes/                     # CAD STL geometry files (base_link, L1 through L5)
├── urdf/
│   └── bi_copter.urdf          # Kinematic description for Robot State Publisher and RViz2
├── worlds/
│   └── bicopter_world.sdf      # Flight laboratory simulation world
├── test/
│   └── test_flight_core.py     # Automated unit test suite (10/10 passing)
└── pixhawk_px4/                # PX4 Autopilot SITL configuration
    ├── 4005_bicopter           # PX4 ROMFS airframe definition (CA_AIRFRAME = 8)
    ├── bicopter.params         # Parameter presets for QGroundControl
    ├── README_PIXHAWK.md       # PX4 integration documentation
    ├── run_px4_sitl.sh         # Standalone PX4 SITL launcher
    └── setup_px4_sitl.sh       # Automated PX4 SITL setup script
```

---

## Execution Guide

### 1. Build the Package
```bash
source /opt/ros/jazzy/setup.bash
cd ~/graduation/bi_copter_control
colcon build --packages-select bi_copter_control --symlink-install
```

### 2. Launch the Master Simulation
In the primary terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/graduation/bi_copter_control/install/setup.bash
ros2 launch bi_copter_control bi_copter_sim.launch.py
```
*Components started:*
- **Gazebo Sim**: Loads the world and spawns the bi-copter on the launch floor.
- **Robot State Publisher**: Broadcasts the dynamic transform hierarchy (`/tf`).
- **Bi-Copter Controller**: Initializes PID loops, connects to Gazebo transport, and opens MAVLink UDP 14550.
- **RViz2**: Displays vehicle geometry and real-time thrust vectors.

### 3. Run the Teleoperation Console
In a separate terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/graduation/bi_copter_control/install/setup.bash
ros2 launch bi_copter_control teleop.launch.py
```
*Note: Pressing any flight stick key (`W`, `A`, `S`, `D`) or `T` while sitting on the ground initiates an automated takeoff to 1.0 m hover.*

### 4. Connect QGroundControl
1. Launch **QGroundControl** on the local host or connected network.
2. The application automatically connects via UDP port `14550`.
3. Displays live Artificial Horizon, heading, climb rate, and arming state.

### 5. Run the Automated Verification Suite
Execute the unit tests without launching the simulator:
```bash
PYTHONPATH=bi_copter_control pytest bi_copter_control/test/test_flight_core.py
```

---

## Teleoperation Key Reference

| Key | Function | Dynamic Response |
| :--- | :--- | :--- |
| **`W`** / **`S`** (or **`Up`** / **`Down`**) | **Pitch Control** | Commanded pitch forward / backward. Springs to 0° neutral on release. |
| **`A`** / **`D`** (or **`Left`** / **`Right`**) | **Roll Control** | Commanded bank left / right via differential thrust. Springs to 0° neutral on release. |
| **`R`** / **`F`** | **Altitude Adjust** | Step target altitude up (+0.20 m) or down (-0.20 m). Autopilot locks height. |
| **`Q`** / **`E`** | **Yaw Rate Control** | Commanded yaw rotation rate ($\pm 1.0\,\text{rad/s}$). Springs to 0 neutral on release. |
| **`Space`** / **`H`** | **Level Out** | Resets attitude to 0° and restores stable 1.0 m hover. |
| **`T`** | **Auto Takeoff** | Automated smooth climb to 1.0 m hover altitude. |
| **`L`** | **Auto Land** | Controlled descent to ground followed by automated motor disarm. |
| **`M`** | **Arm / Disarm** | Toggles motor arming state. |
| **`X`** | **Emergency Stop** | Immediate motor power cut (0 N thrust). |
| **`+`** / **`-`** | **Sensitivity** | Increments or decrements maximum stick tilt limit ($\pm 1.0^\circ$). |

---

## Author and Contact

**Mohammed Abdelsabour**  
Department of Mechanical / Mechatronics Engineering  
- **Email**: mohammedabdelsaboor@gmail.com  
- **GitHub**: [mohammedabdelsaboor](https://github.com/mohammedabdelsaboor)  
- **Repository**: [bi_copter_control_ROS2jazzy](https://github.com/mohammedabdelsaboor/bi_copter_control_ROS2jazzy)
