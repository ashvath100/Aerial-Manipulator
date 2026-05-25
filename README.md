# Aerial-Manipulator# SDU UAS — Drone Arm Control System

ROS2 software stack for a drone-mounted 2-DOF robotic arm with gimbal stabilisation. The system uses a mobile Jacobian controller to autonomously position the arm end-effector above a sequence of targets (pole tips) using a PX4-powered drone.

> **AI Assistance Notice:** Parts of the software, diagrams, and documentation in this project were developed with the assistance of Claude (Anthropic). All outputs were reviewed and validated by the project team.

---

## System Overview

```
MAVROS / PX4
    ├── /mavros/imu/data          →  gimbal_node
    ├── /mavros/state             →  offboard_ctrl, robot_state_machine
    └── /mavros/local_position/odom → offboard_ctrl

gimbal_node
    └── /gimbal_controller/commands → robot_state_machine (J0, J1)

offboard_ctrl
    ├── /arm_controller/commands    → robot_state_machine (J2, J3)
    └── /mavros/setpoint_position/local → PX4

robot_state_machine
    └── Dynamixel motors (J0, J1, J2, J3)
```

### Nodes

| Node | File | Purpose |
|------|------|---------|
| `gimbal_node` | `gimbal_node.py` | IMU-based 2-axis gimbal stabilisation via PID |
| `robot_state_machine` | `robot_state_machine.py` | Dynamixel motor controller with IDLE/HOME/JACOBIAN states |
| `offboard_ctrl` | `offboard_ctrl.py` | 5-DOF mobile Jacobian controller for drone + arm |

---

## Hardware

- **Drone:** PX4-powered UAV with MAVROS
- **Arm:** 2-DOF Dynamixel motor arm mounted below drone
- **Gimbal:** 2-axis gimbal (pitch + roll) stabilised against IMU
- **Motors:** Dynamixel (IDs 0–3), connected via `/dev/ttyUSB1` at 57600 baud

### DOF Layout

```
q = [x_b, y_b, z_b, j1, j2]
      ↑               ↑   ↑
   drone pos       arm joints
```

- `J0` — gimbal pitch
- `J1` — gimbal roll
- `J2` — arm joint 1
- `J3` — arm joint 2

---

## Dependencies

- ROS2 (Humble or later)
- MAVROS
- `mavros_msgs`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`
- `dynamixel_easy_sdk`
- Python: `numpy`

Install Python dependencies:
```bash
pip install numpy
```

---

## Running

### 1. Gimbal node

```bash
python3 gimbal_node.py
```

Stabilises gimbal at **pitch = 0°, roll = 90°**. Gains and limits can be overridden via ROS2 parameters:

```bash
ros2 run <pkg> gimbal_node --ros-args \
  -p pitch_kp:=3.0 \
  -p roll_kp:=3.0 \
  -p lp_alpha:=0.35
```

### 2. Robot state machine

```bash
python3 robot_state_machine.py
```

Runs the user sequence: `IDLE → HOME → IDLE → JACOBIAN_ENTRY → JACOBIAN`.
Automatically returns to `IDLE` if flight mode switches to `OFFBOARD`.

### 3. Offboard controller

**Simulation (Gazebo / SITL) — uses `nav_msgs/Odometry` for EE and targets:**
```bash
python3 offboard_ctrl.py --sim
```

**Real hardware — uses `geometry_msgs/PoseStamped` for EE and targets:**
```bash
python3 offboard_ctrl.py --real
```

Defaults to `--sim` if no flag is given.

---

## State Machine

```
IDLE  →  HOME  →  JACOBIAN_ENTRY  →  JACOBIAN
                                          ↑
                          OFFBOARD detected → IDLE ↩
```

| State | Joint targets | Move order |
|-------|--------------|------------|
| `IDLE` | [0, −90, 0, 0]° | Distal → proximal [2,3,1,0] |
| `HOME` | [90, −35, 100, −120]° | Proximal → distal [0,1,3,2] |
| `JACOBIAN_ENTRY` | [0, 0, 0, 0]° | Proximal → distal [0,1,3,2] |
| `JACOBIAN` | Tracks ROS topics live at 50 Hz | — |

---

## Offboard Controller — Control Phases

| Phase | Description |
|-------|-------------|
| 1 | Wait for drone pose, joint states, and EE pose |
| 2 | Wait for OFFBOARD mode — stream hold setpoints |
| 3 | Hold position for 10 s after OFFBOARD detected |
| 4 | Wait for `/targetN/pose` |
| 4a | Climb to safe height (max pole tip + arm reach + offset) |
| 4b | Jacobian approach — DLS pseudoinverse + null-space |

### Kinematics

```
EE_x = x_b + L1·sin(j1) + L2·sin(j1+j2)
EE_y = y_b
EE_z = z_b − GIMBAL_Z − L1·cos(j1) − L2·cos(j1+j2)

L1 = L2 = 0.16 m     GIMBAL_Z = 0.21 m
```

### Control law

```
dq = α · J⁺_dls · err  +  N · (−β · (q − q_pref))
     ↑ primary task          ↑ null-space (arm posture)
```

---

## Tuning Parameters (`offboard_ctrl.py`)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `ALPHA` | 0.3 | Jacobian step size — reduce to fix overshoot |
| `LAMBDA` | 0.05 | DLS damping — increase for singularity stability |
| `BETA` | 0.07 | Null-space gain — controls arm posture strength |
| `GOAL_THRESH` | 0.1 m | Target reached threshold |
| `CTRL_HZ` | 15 Hz | Control loop rate |
| `EE_OFFSET_Z` | 0.20 m | Goal height above target |
| `OFFBOARD_DELAY` | 10 s | Wait time after OFFBOARD before moving |
| `dq[:3]` clip | ±0.1 m | Max drone step per tick |
| `dq[3:]` clip | ±5° | Max joint step per tick |

---

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/mavros/imu/data` | `sensor_msgs/Imu` | MAVROS | `gimbal_node` |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | `offboard_ctrl`, `robot_state_machine` |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS | `offboard_ctrl` |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | `offboard_ctrl` | MAVROS |
| `/gimbal_controller/commands` | `std_msgs/Float64MultiArray` | `gimbal_node` | `robot_state_machine` |
| `/arm_controller/commands` | `std_msgs/Float64MultiArray` | `offboard_ctrl` | `robot_state_machine` |
| `/end_effector/pose` | `Odometry` (sim) / `PoseStamped` (real) | Hardware/sim | `offboard_ctrl` |
| `/targetN/pose` | `Odometry` (sim) / `PoseStamped` (real) | External | `offboard_ctrl` |
| `/joint_states` | `sensor_msgs/JointState` | Hardware | `offboard_ctrl` |

---

## POC Testing — Partial Hardware-in-the-Loop

The proof of concept was validated using a hybrid HIL approach:

**Simulated:** PX4 SITL + Gazebo (drone physics, IMU, odometry) + MAVROS + `offboard_ctrl`

**Real hardware:** Dynamixel motors + gimbal joints + `robot_state_machine`

MAVROS published all sensor topics identically to real flight, so every downstream node had no way to distinguish simulation from reality. The arm received genuine Jacobian-computed targets and responded with real motor torque and PID behaviour.

**Validated:** ROS2 interfaces, state machine transitions, motor limits, OFFBOARD detection, Jacobian tracking at 50 Hz.

**Not validated:** True EE accuracy with moving base, gimbal performance under flight vibration, drone-arm interaction dynamics.

---

## File Structure

```
.
├── gimbal_node.py          # 2-axis gimbal stabilisation
├── robot_state_machine.py  # Dynamixel state machine
├── offboard_ctrl.py        # Mobile Jacobian offboard controller
└── README.md
```