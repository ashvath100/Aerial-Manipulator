#!/usr/bin/env python3
"""
robot_state_machine.py
======================
State machine for a 4-DOF robot arm + gimbal using Dynamixel motors.

States
------
  IDLE     – joints move to idle_angles  in order [2,3,1,0] (distal→proximal)
  HOME     – joints move to home_angles  in order [0,1,3,2] (proximal→distal)
  JACOBIAN_ENTRY – joints move to [0,0,0,0] in order [0,1,3,2] (blocking)
  JACOBIAN – joints track ROS2 topics live (rad → deg, clamped):
               /gimbal_controller/commands  data=[pitch_rad, roll_rad] → J0, J1
               /arm_controller/commands     data=[j2_rad,   j3_rad  ] → J2, J3

Thread model
------------
  • rclpy.spin()  runs in a daemon background thread  (ROS callbacks)
  • sm.run()      runs in the main thread              (state loop @ 50 Hz)
  • sm.request()  is thread-safe – call from anywhere

Usage
-----
  python3 robot_state_machine.py
  # or:  ros2 run <pkg> robot_state_machine
"""

import math
import time
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import State as MavState

from dynamixel_easy_sdk import Connector, OperatingMode, ProfileConfiguration


# ─────────────────────────────────────────────────────────────────────────────
# Hardware configuration
# ─────────────────────────────────────────────────────────────────────────────

DEVICE_PORT          = "/dev/ttyUSB1"
BAUDRATE             = 57600
MOTOR_IDS            = [0, 1, 2, 3]

RAW_MAX              = 4095
DEG_PER_RAW          = 360.0 / RAW_MAX
RAW_PER_DEG          = RAW_MAX / 360.0

PROFILE_VELOCITY     = 1
PROFILE_ACCELERATION = 0.5
POSITION_P_GAIN      = 350
POSITION_I_GAIN      = 10
POSITION_D_GAIN      = 100

# Per-joint travel limits (degrees)
MIN_DEG  = [-180, -180, -180, -180]
MAX_DEG  = [ 180,  180,  180,  180]

# Named poses (degrees)
HOME_ANGLES          = [90.0, -35.0, 100.0, -120.0]
IDLE_ANGLES          = [ 0.0, -90.0,   0.0,    0.0]
JACOBIAN_ENTRY_ANGLES = [0.0,   0.0,   0.0,    0.0]

# Safe movement orders
IDLE_ORDER           = [2, 3, 1, 0]   # distal → proximal
HOME_ORDER           = [0, 1, 3, 2]   # proximal → distal
JACOBIAN_ENTRY_ORDER = [0, 1, 3, 2]   # proximal → distal

POSITION_TOLERANCE_DEG = 3.0
WAIT_TIMEOUT           = 4.0   # seconds per joint

# Topic → joint mapping
#   /gimbal_controller/commands : data[0]=pitch → J0,  data[1]=roll → J1
#   /arm_controller/commands    : data[0]       → J2,  data[1]      → J3
GIMBAL_JOINT_MAP = {0: 0, 1: 1}
ARM_JOINT_MAP    = {0: 2, 1: 3}


# ─────────────────────────────────────────────────────────────────────────────
# States
# ─────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE          = auto()
    HOME          = auto()
    JACOBIAN_ENTRY = auto()
    JACOBIAN      = auto()


# ─────────────────────────────────────────────────────────────────────────────
# Low-level angle helpers
# ─────────────────────────────────────────────────────────────────────────────

def raw_to_deg(raw: int) -> float:
    return raw * DEG_PER_RAW

def deg_to_raw(deg: float) -> int:
    return int(round(deg * RAW_PER_DEG))

def wrap_360(deg: float) -> float:
    return deg % 360.0

def clamp_deg(angle: float, joint_idx: int) -> float:
    return max(MIN_DEG[joint_idx], min(MAX_DEG[joint_idx], angle))

def shortest_goal_raw(current_raw: int, target_deg: float):
    """
    Compute the goal raw value that reaches target_deg via the
    shortest angular path (avoids unnecessary full rotations).
    Returns (goal_raw, current_mod_deg, error_deg, goal_deg).
    """
    current_mod = wrap_360(raw_to_deg(current_raw))
    error_deg   = (target_deg - current_mod + 180.0) % 360.0 - 180.0
    goal_deg    = raw_to_deg(current_raw) + error_deg
    return deg_to_raw(goal_deg), current_mod, error_deg, goal_deg


# ─────────────────────────────────────────────────────────────────────────────
# State machine node
# ─────────────────────────────────────────────────────────────────────────────

class RobotStateMachine(Node):
    """
    ROS2 node + Dynamixel controller with four operating states.

    Public methods (thread-safe)
    ----------------------------
    request(State.HOME)          – queue a transition
    request(State.IDLE)
    request(State.JACOBIAN_ENTRY)
    request(State.JACOBIAN)
    run(rate_hz=50.0)            – blocking main loop (call from main thread)
    shutdown()                   – disable torque, close port
    """

    def __init__(self):
        super().__init__('robot_state_machine')

        # ── State variables ───────────────────────────────────────────
        self._state         = None
        self._pending_state = None
        self._state_lock    = threading.Lock()

        # Live joint targets for JACOBIAN state (degrees, clamped)
        self._jac_targets_deg = [0.0] * len(MOTOR_IDS)
        self._jac_lock        = threading.Lock()

        # Flight mode tracking
        self.mode = None

        # ── Hardware ──────────────────────────────────────────────────
        self.get_logger().info(f'Opening {DEVICE_PORT} @ {BAUDRATE} baud')
        self._conn   = Connector(DEVICE_PORT, BAUDRATE)
        self._motors = [self._conn.createMotor(mid) for mid in MOTOR_IDS]
        self._init_motors()

        # ── ROS subscribers ───────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Float64MultiArray, '/gimbal_controller/commands',
            self._cb_gimbal, qos)
        self.create_subscription(
            Float64MultiArray, '/arm_controller/commands',
            self._cb_arm, qos)
        self.create_subscription(
            MavState, '/mavros/state',
            self._cb_mavros_state, qos)

        self.get_logger().info('RobotStateMachine initialised.')

    # ─────────────────────────────────────────────────────────────────
    # Hardware init
    # ─────────────────────────────────────────────────────────────────

    def _init_motors(self):
        for i, motor in enumerate(self._motors):
            motor.disableTorque()
            motor.setOperatingMode(OperatingMode.EXTENDED_POSITION)
            try:
                motor.setProfileConfiguration(
                    ProfileConfiguration(PROFILE_VELOCITY, PROFILE_ACCELERATION))
            except Exception:
                self.get_logger().warn(
                    f'Motor {MOTOR_IDS[i]}: profile config not supported')
            motor.setPositionPGain(POSITION_P_GAIN)
            motor.setPositionIGain(POSITION_I_GAIN)
            motor.setPositionDGain(POSITION_D_GAIN)
            motor.enableTorque()
        self.get_logger().info('All motors ready.')

    # ─────────────────────────────────────────────────────────────────
    # ROS topic callbacks
    # ─────────────────────────────────────────────────────────────────

    def _cb_gimbal(self, msg: Float64MultiArray):
        """
        /gimbal_controller/commands → J0 (pitch), J1 (roll).
        Values are in radians; convert and clamp before storing.
        """
        with self._jac_lock:
            if len(msg.data) >= 2:
                self._jac_targets_deg[0] = clamp_deg(math.degrees(msg.data[0]), 0)
                self._jac_targets_deg[1] = clamp_deg(math.degrees(msg.data[1]), 1)

    def _cb_arm(self, msg: Float64MultiArray):
        """
        /arm_controller/commands → J2, J3.
        Values are in radians; convert and clamp before storing.
        """
        with self._jac_lock:
            if len(msg.data) >= 2:
                self._jac_targets_deg[2] = clamp_deg(math.degrees(msg.data[0]), 2)
                self._jac_targets_deg[3] = clamp_deg(math.degrees(msg.data[1]), 3)

    def _cb_mavros_state(self, msg: MavState):
        if msg.mode != self.mode:
            self.get_logger().info(f'Flight mode → {msg.mode}')
        self.mode = msg.mode

        if msg.mode == 'OFFBOARD' and self._state != State.IDLE:
            self.get_logger().info(
                f'OFFBOARD detected: requesting IDLE '
                f'(was {self._state.name if self._state else "INIT"})')
            self.request(State.IDLE)

    # ─────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────

    def request(self, new_state: State):
        """Request a state transition (thread-safe, non-blocking)."""
        with self._state_lock:
            self._pending_state = new_state
        self.get_logger().info(f'→ State requested: {new_state.name}')

    @property
    def state(self) -> State:
        return self._state

    # ─────────────────────────────────────────────────────────────────
    # Main run loop
    # ─────────────────────────────────────────────────────────────────

    def run(self, rate_hz: float = 50.0):
        """
        Blocking main loop.
        - Processes pending state transitions.
        - In JACOBIAN state: applies ROS topic targets to motors at rate_hz.
        - In IDLE/HOME/JACOBIAN_ENTRY: motors driven by _enter_state().
        """
        dt = 1.0 / rate_hz

        while rclpy.ok():

            with self._state_lock:
                pending = self._pending_state
                self._pending_state = None

            if pending is not None and pending != self._state:
                self._enter_state(pending)

            if self._state == State.JACOBIAN:
                self._jacobian_tick()

            time.sleep(dt)

    # ─────────────────────────────────────────────────────────────────
    # State entry handlers
    # ─────────────────────────────────────────────────────────────────

    def _enter_state(self, new_state: State):
        self.get_logger().info(
            f'[{self._state.name if self._state else "INIT"}] → [{new_state.name}]')
        self._state = new_state

        if new_state == State.IDLE:
            self._run_sequence(IDLE_ANGLES, IDLE_ORDER)

        elif new_state == State.HOME:
            self._run_sequence(HOME_ANGLES, HOME_ORDER)

        elif new_state == State.JACOBIAN_ENTRY:
            self.get_logger().info('JACOBIAN_ENTRY: moving to entry pose...')
            self._run_sequence(JACOBIAN_ENTRY_ANGLES, JACOBIAN_ENTRY_ORDER)

        elif new_state == State.JACOBIAN:
            with self._jac_lock:
                for i, motor in enumerate(self._motors):
                    self._jac_targets_deg[i] = wrap_360(
                        raw_to_deg(motor.getPresentPosition()))
            self.get_logger().info(
                'JACOBIAN: tracking /gimbal_controller/commands '
                'and /arm_controller/commands')

    # ─────────────────────────────────────────────────────────────────
    # Sequenced motion helpers
    # ─────────────────────────────────────────────────────────────────

    def _run_sequence(self, target_angles: list, order: list):
        for joint_idx in order:
            self._move_joint(joint_idx, target_angles[joint_idx])
            time.sleep(0.2)

    def _move_joint(self, joint_idx: int, target_deg: float) -> bool:
        motor       = self._motors[joint_idx]
        target_deg  = clamp_deg(target_deg, joint_idx)
        current_raw = motor.getPresentPosition()
        goal_raw, current_mod, error_deg, _ = shortest_goal_raw(
            current_raw, target_deg)

        self.get_logger().info(
            f'  J{joint_idx} (ID {MOTOR_IDS[joint_idx]}) '
            f'{current_mod:.1f}° → {target_deg:.1f}°  Δ{error_deg:+.1f}°')

        motor.setGoalPosition(goal_raw)
        return self._wait_for_joint(joint_idx, target_deg)

    def _wait_for_joint(self, joint_idx: int, target_deg: float) -> bool:
        motor    = self._motors[joint_idx]
        deadline = time.time() + WAIT_TIMEOUT

        while time.time() < deadline:
            current_deg = wrap_360(raw_to_deg(motor.getPresentPosition()))
            error       = abs((target_deg - current_deg + 180.0) % 360.0 - 180.0)
            if error <= POSITION_TOLERANCE_DEG:
                self.get_logger().info(f'  ✓ J{joint_idx}: reached {current_deg:.1f}°')
                return True
            time.sleep(0.05)

        current_deg = wrap_360(raw_to_deg(motor.getPresentPosition()))
        self.get_logger().warn(
            f'  ✗ J{joint_idx}: timeout at {current_deg:.1f}° '
            f'(err {abs((target_deg - current_deg + 180)%360-180):.1f}°)')
        return False

    # ─────────────────────────────────────────────────────────────────
    # JACOBIAN tick  (called at run-loop rate, ~50 Hz)
    # ─────────────────────────────────────────────────────────────────

    def _jacobian_tick(self):
        with self._jac_lock:
            targets = list(self._jac_targets_deg)

        for joint_idx, target_deg in enumerate(targets):
            current_raw = self._motors[joint_idx].getPresentPosition()
            goal_raw, _, _, _ = shortest_goal_raw(current_raw, target_deg)
            self._motors[joint_idx].setGoalPosition(goal_raw)

    # ─────────────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────────────

    def shutdown(self):
        self.get_logger().info('Disabling motors...')
        for motor in self._motors:
            motor.disableTorque()
        self._conn.closePort()
        self.get_logger().info('Port closed.')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    sm = RobotStateMachine()

    spin_thread = threading.Thread(target=rclpy.spin, args=(sm,), daemon=True)
    spin_thread.start()

    try:
        def user_sequence():
            print("\n[USER] IDLE")
            sm.request(State.IDLE)
            time.sleep(1)

            print("\n[USER] HOME")
            sm.request(State.HOME)
            time.sleep(5)

            print("\n[USER] IDLE")
            sm.request(State.IDLE)
            time.sleep(2)

            print("\n[USER] JACOBIAN_ENTRY")
            sm.request(State.JACOBIAN_ENTRY)
            time.sleep(2)

            print("\n[USER] JACOBIAN")
            sm.request(State.JACOBIAN)

            # Watch for OFFBOARD mode and go IDLE
            while rclpy.ok():
                if sm.mode == 'OFFBOARD' and sm.state != State.IDLE:
                    print("\n[USER] OFFBOARD detected → IDLE")
                    sm.request(State.IDLE)
                time.sleep(0.1)

        seq_thread = threading.Thread(target=user_sequence, daemon=True)
        seq_thread.start()

        sm.run(rate_hz=50.0)

    except KeyboardInterrupt:
        print('\nStopped by user.')

    finally:
        sm.shutdown()
        rclpy.shutdown()
        print('Done.')


if __name__ == '__main__':
    main()