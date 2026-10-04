# Pixhawk / PX4 Autopilot Integration Guide for Bi-Copter

This guide explains how to run the dual-motor dual-servo tilt-rotor bi-copter in **PX4 SITL simulation** (with Gazebo Harmonic) and how to configure a **physical Pixhawk flight controller**.

---

## 1. Flight Dynamics & Actuator Allocation

A bi-copter uses 4 control actuators:
- **Left Motor (CCW)**: Rotor 0 at $y = +0.161\,\text{m}$
- **Right Motor (CW)**: Rotor 1 at $y = -0.161\,\text{m}$
- **Left Tilt Servo**: Servo 1 tilting left motor forward/backward ($\pm 30^\circ$)
- **Right Tilt Servo**: Servo 2 tilting right motor forward/backward ($\pm 30^\circ$)

### Control Allocation Matrix:
| Axis | Mechanism | Actuation |
| :--- | :--- | :--- |
| **Altitude ($Z$)** | Collective Thrust | Increase both motor speeds ($T_0 + T_1$) |
| **Roll ($\Phi$)** | Differential Thrust | Left vs Right motor speed difference ($T_0 - T_1$) |
| **Pitch ($\Theta$)** | Common Servo Tilt | Both servos tilt in the same direction ($\delta_0 = \delta_1$) |
| **Yaw ($\Psi$)** | Differential Servo Tilt | Servos tilt in opposite directions ($\delta_1 - \delta_0$) + Rotor drag reaction |

---

## 2. PX4 Configuration Parameters

In PX4 v1.14 and v1.15+, dynamic control allocation is enabled via the `CA_AIRFRAME` parameter:

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `CA_AIRFRAME` | `8` | **Multirotor with Tilt** allocator |
| `CA_ROTOR_COUNT` | `2` | 2 thrust rotors |
| `CA_SV_TL_COUNT` | `2` | 2 tilt servos |
| `CA_ROTOR0_TILT` | `1` | Assign Left Rotor to Tilt Servo 1 |
| `CA_ROTOR0_PY` | `0.161` | Left rotor arm position ($+Y$) |
| `CA_ROTOR0_KM` | `0.016` | CCW rotor drag torque constant |
| `CA_ROTOR1_TILT` | `2` | Assign Right Rotor to Tilt Servo 2 |
| `CA_ROTOR1_PY` | `-0.161` | Right rotor arm position ($-Y$) |
| `CA_ROTOR1_KM` | `-0.016` | CW rotor drag torque constant |
| `CA_SV_TL0_CT` | `3` | Tilt Servo 1 control function: **Yaw and Pitch** |
| `CA_SV_TL0_MINA` | `-30.0` | Max backward tilt angle ($-30^\circ$) |
| `CA_SV_TL0_MAXA` | `30.0` | Max forward tilt angle ($+30^\circ$) |
| `CA_SV_TL1_CT` | `3` | Tilt Servo 2 control function: **Yaw and Pitch** |
| `CA_SV_TL1_MINA` | `-30.0` | Max backward tilt angle ($-30^\circ$) |
| `CA_SV_TL1_MAXA` | `30.0` | Max forward tilt angle ($+30^\circ$) |

---

## 3. Running PX4 SITL Simulation

To run PX4 software-in-the-loop directly with the Gazebo model and world created in this package:

```bash
# Clone PX4-Autopilot if you don't have it yet:
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot

# Run the automated launcher:
cd /home/sheka/graduation/bi_copter_control/src/bi_copter_control/pixhawk_px4
./run_px4_sitl.sh ~/PX4-Autopilot
```

Once PX4 SITL starts, you will get the `pxh>` shell. You can arm and takeoff:
```
pxh> commander arm
pxh> commander takeoff
```
Or connect **QGroundControl** to plan missions, joystick fly, or inspect live telemetry.

---

## 4. Hardware Setup on Physical Pixhawk

When building the physical drone:

### 4.1 Wiring Pinouts (MAIN PWM Outputs on Pixhawk):
```
Pixhawk FMU MAIN Port:
  Pin 1 (PWM 1) ----> Left ESC Signal (Motor 1)
  Pin 2 (PWM 2) ----> Right ESC Signal (Motor 2)
  Pin 3 (PWM 3) ----> Left Tilt Servo Signal (PWM 50Hz / 333Hz)
  Pin 4 (PWM 4) ----> Right Tilt Servo Signal (PWM 50Hz / 333Hz)
  5V Power Rail ----> External 5V BEC (Do NOT power servos from Pixhawk RC rail!)
  Ground        ----> Common Ground to battery, ESCs, and BEC
```

### 4.2 Actuator Setup in QGroundControl:
1. Open **QGroundControl** $\to$ **Vehicle Setup** $\to$ **Actuators**.
2. Select **Multirotor with Tilt** airframe.
3. Configure outputs:
   - Output 1: Motor 1
   - Output 2: Motor 2
   - Output 3: Tilt Servo 1
   - Output 4: Tilt Servo 2
4. Move sliders to verify:
   - Servo 1 & 2 center at $0^\circ$ (rotors pointing straight up).
   - Positive pitch input tilts both servos forward.
   - Positive yaw input tilts left servo backward, right servo forward.

### 4.3 Recommended Initial PID Gains:
- **Roll Rate PID**: $P = 0.12$, $I = 0.08$, $D = 0.003$
- **Pitch Rate PID**: $P = 0.10$, $I = 0.06$, $D = 0.002$
- **Yaw Rate PID**: $P = 0.15$, $I = 0.05$, $D = 0.0$
- **Attitude P**: Roll $P = 6.0$, Pitch $P = 5.0$, Yaw $P = 2.5$
