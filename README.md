# Bi-Copter Control & Simulation Package (ROS 2 Jazzy, Gazebo Harmonic & QGroundControl)

A formal, production-grade simulation and PID flight stabilization package for a dual-propeller dual-tilt-servo bi-copter drone modeled directly after Pixhawk / PX4 flight control architecture and QGroundControl telemetry.

---

## Key Features

1. **Pixhawk-Standard Attitude & Thrust Control (Stabilized Mode)**:
   - **Direct Collective Thrust**: Direct throttle lift control ($T_{\text{hover}} \approx 4.72\,\text{N}$ for $0.481\,\text{kg}$ mass). No artificial coordinate locks or position hold delays.
   - **Attitude PID**: Roll angle PID, Pitch angle PID, and Yaw rate PID with instant horizontal self-leveling on stick release.
   - **Actuator Mixer**: Exact allocation mapping Roll $\to$ differential motor speed, Pitch $\to$ common servo tilt, and Yaw $\to$ differential servo tilt + rotor drag counter-torque.

2. **Accurate Thrust Force & Tilt Servo Simulation**:
   - `gz::sim::systems::MulticopterMotorModel`: Exact aerodynamic quadratic thrust ($F = k_F \cdot \omega^2$) and drag reaction torque ($M = k_M \cdot F$).
   - `gz::sim::systems::JointPositionController`: Velocity-limited physical position servos on `left_tilt_joint` and `right_tilt_joint` ($\pm 30^\circ$).
   - **Live Force & Angle Visualizer**: Dynamic thrust arrows in RViz2 and Gazebo attached directly to the rotating motor links, scaling in real-time with actual Newtons ($F_0, F_1$), accompanied by floating telemetry tags (`LEFT: 2.36 N (+5.2°)`, `RIGHT: 2.36 N (+5.2°)`).

3. **Direct QGroundControl (QGC) MAVLink Integration**:
   - Built-in lightweight MAVLink UDP broadcast on port `14550`.
   - Launching QGroundControl automatically connects to the simulated bi-copter as vehicle 1.
   - Displays live Primary Flight Display (PFD) / Artificial Horizon, compass heading, battery voltage, climb rate, and armed status.
   - Supports arming/disarming and flying with QGroundControl virtual on-screen thumbsticks or a connected USB gamepad.

4. **Streamlined & Minimal Architecture**:
   - Zero clutter: all redundant, unused scripts and duplicate launchers have been eliminated.
   - 1 Master launch file (`bi_copter_sim.launch.py`).
   - 1 Interactive RC teleop console (`teleop.launch.py`).

---

## Package Structure

```
bi_copter_control/
├── CMakeLists.txt              # Standard build configuration
├── package.xml                 # ROS 2 package dependencies
├── bi_copter_control/
│   ├── __init__.py
│   ├── flight_core.py          # Cascaded Attitude PID + direct thrust + exact bi-copter mixer
│   ├── bicopter_controller_node.py # Controller + Gazebo bridge + QGC MAVLink server + Thrust visualizer
│   └── bicopter_teleop_node.py   # RC transmitter-style keyboard console with live HUD
├── config/
│   └── bicopter.rviz           # Pre-configured RViz2 (Robot Model + TF + Thrust Vectors)
├── launch/
│   ├── bi_copter_sim.launch.py # Master launch: Gazebo + Controller + RSP + RViz2
│   └── teleop.launch.py        # Interactive keyboard teleoperation
├── models/
│   └── bi_copter/              # Clean CAD physics model (SDF) with motor & servo plugins
├── meshes/                     # SolidWorks CAD STLs (base_link, L1..L5)
├── urdf/
│   └── bi_copter.urdf          # Clean URDF for Robot State Publisher & RViz2
├── worlds/
│   └── bicopter_world.sdf      # Empty Drone Flight Laboratory & Safety Net Arena
├── test/
│   └── test_flight_core.py     # 10/10 automated flight core unit tests
└── pixhawk_px4/
    ├── 4005_bicopter           # Official PX4 airframe definition (CA_AIRFRAME = 8)
    ├── bicopter.params         # QGroundControl parameter presets
    ├── README_PIXHAWK.md       # SITL & hardware documentation
    └── run_px4_sitl.sh         # Standalone PX4 SITL launcher
```

---

## Quick Start Guide

### 1. Launch the Main Simulation
```bash
source /opt/ros/jazzy/setup.bash
source /home/sheka/graduation/bi_copter_control/install/setup.bash
ros2 launch bi_copter_control bi_copter_sim.launch.py
```
- Gazebo Sim opens with the empty safety net arena.
- RViz2 displays the 3D bi-copter with dynamic thrust vectors and articulated servos.
- The controller arms and stabilizes the bi-copter at hover thrust ($4.72\,\text{N}$).

### 2. Connect QGroundControl
1. Open **QGroundControl** on your computer.
2. QGC will automatically connect via UDP port `14550`:
   - "Connected to vehicle 1 (Multirotor, PX4)"
   - View real-time roll, pitch, heading, altitude, and throttle percentage on the PFD.
   - Arm/Disarm and fly with on-screen joysticks.

### 3. Fly with the RC Keyboard Teleop Console
In a second terminal:
```bash
ros2 launch bi_copter_control teleop.launch.py
```

| Key | Control | Function |
| :--- | :--- | :--- |
| **`W`** / **`S`** (or **`↑`** / **`↓`**) | **Pitch Forward / Back** | Tilts nose down / forward or nose up / backward (spring-centers to $0^\circ$) |
| **`A`** / **`D`** (or **`←`** / **`→`**) | **Roll Left / Right** | Banks left or right via differential thrust (spring-centers to $0^\circ$) |
| **`R`** / **`F`** | **Climb UP / DOWN** | Commands Pixhawk ALTCTL target altitude ($\pm 0.20\,\text{m}$) |
| **`Q`** / **`E`** | **Yaw Left / Right** | Rotates yaw heading rate ($\pm 1.0\,\text{rad/s}$, spring-centers to $0$) |
| **`Space`** / **`H`** | **Level Out** | Instantly zeroes tilt angles; resets and locks to stable $1.0\,\text{m}$ hover |
| **`T`** / **`L`** | **Auto Takeoff / Land** | Automated smooth ascent to $1.0\,\text{m}$ hover / soft touchdown & disarm |
| **`M`** / **`X`** | **Arm / Kill Switch** | Toggle arming / emergency instant motor cutoff |
| **`+`** / **`-`** | **Stick Sensitivity** | Increase / decrease max tilt angle ($\pm 1^\circ$) |

*Tip: If the drone is sitting disarmed on the launch pad, simply pressing **`W`**, **`A`**, **`S`**, **`D`**, or **`T`** will automatically initiate a smooth takeoff to 1.0 m hover!*

### 4. Running the Automated Flight Verification Suite
Run the 10/10 automated physics and mixer verification tests in 20 milliseconds:
```bash
PYTHONPATH=bi_copter_control pytest bi_copter_control/test/test_flight_core.py
```

### 5. Direct ROS 2 Topic Command
Send attitude and thrust commands directly:
```bash
ros2 topic pub --once /cmd_attitude_thrust geometry_msgs/msg/Twist '{
  linear: {x: 0.0, y: 0.174, z: 5.2},
  angular: {z: 0.0}
}'
```
*(linear.x = roll rad, linear.y = pitch rad, linear.z = thrust N or target altitude, angular.z = yaw rate rad/s)*
