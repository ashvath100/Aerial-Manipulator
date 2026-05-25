#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py: Mobile Jacobian controller for SDU UAS drone + 2-DOF arm.

DOF layout (5 total):
  q = [x_b, y_b, z_b, j1, j2]

Usage
-----
  Simulation (Gazebo/SITL):
    python3 offboard_ctrl.py --sim

  Real hardware:
    python3 offboard_ctrl.py --real

Mode differences
----------------
  sim  — /end_effector/pose and /targetN/pose use nav_msgs/Odometry
  real — /end_effector/pose and /targetN/pose use geometry_msgs/PoseStamped
"""

import argparse
import math
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType

# ── Tuning ──────────────────────────────────────────────────────────────────
MAX_TARGETS  = 6
GOAL_THRESH  = 0.1     # metres per axis
CTRL_HZ      = 15.0    # Hz
ALPHA        = 0.3     # Jacobian step size
LAMBDA       = 0.05    # DLS damping
BETA         = 0.07    # null-space gain
EE_OFFSET_Z  = 0.20    # metres above target

OFFBOARD_DELAY  = 10.0  # seconds to wait after OFFBOARD detected
TARGET_HOVER_T  = 10.0  # seconds to hover at each target before moving on

L1           = 0.16
L2           = 0.16
GIMBAL_Z     = 0.21    # arm root offset below drone base

LIMITS = np.array([
    [-50.0, 50.0],   # x_b
    [-50.0, 50.0],   # y_b
    [  0.3, 10.0],   # z_b
    [ -3.14, 3.14],  # joint1
    [ -3.14, 3.14],  # joint2
])


# ── Kinematics ───────────────────────────────────────────────────────────────

def fk(q):
    x_b, y_b, z_b, j1, j2 = q
    a12 = j1 + j2
    return np.array([
        x_b + L1 * math.sin(j1) + L2 * math.sin(a12),
        y_b,
        z_b - GIMBAL_Z - L1 * math.cos(j1) - L2 * math.cos(a12),
    ])


def jacobian(q):
    _, _, _, j1, j2 = q
    a12 = j1 + j2
    c1, c12 = math.cos(j1), math.cos(a12)
    return np.array([
        [1, 0, 0,  L1*c1 + L2*c12,  L2*c12],
        [0, 1, 0,  0,               0     ],
        [0, 0, 1,  0,               0     ],
    ])


def dls_pinv(J):
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + LAMBDA**2 * np.eye(m))


# ── Pose extraction helpers ───────────────────────────────────────────────────

def pos_from_odometry(msg: Odometry) -> np.ndarray:
    return np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    ])

def pos_from_posestamped(msg: PoseStamped) -> np.ndarray:
    return np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ])


# ── Node ─────────────────────────────────────────────────────────────────────

class OffboardControl(Node):

    def __init__(self, mode: str = 'sim'):
        self._at_safe_height = False
        self._at_target_xy   = False

        super().__init__('offboard_ctrl')

        self._sim = (mode == 'sim')
        self.get_logger().info(
            f"Mode: {'SIMULATION (Odometry)' if self._sim else 'REAL (PoseStamped)'}")

        # ── QoS ───────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── State ─────────────────────────────────────────────────────
        self.q              = np.array([0.0, 0.0, 0.0, 0, 1.57])
        self.joint_names    = ['joint1', 'joint2']

        self.targets        = {}
        self.target_idx     = 1
        self.offboard       = False
        self._pose_received = False
        self._joints_init   = True   # no joint feedback — fixed at init values

        self.ee_pos         = np.array([0.0, 0.0, 0.0])
        self._ee_received   = False

        self._offboard_detected_time = None
        self._offboard_delay_done    = False
        self._hold_position          = None  # position snapshot at OFFBOARD entry

        # Hover state — set when target is first reached
        self._hover_start_time = None   # monotonic time when hover began
        self._hovering         = False  # True while waiting out hover period
        self._hover_position   = None   # position snapshot when target reached

        # Return home state — after all targets complete
        self._returning        = False  # True once all targets done
        self._return_phase     = None   # 'climb' or 'center'
        self._center           = np.array([0.0, 0.0])  # XY home position



        # ── Subscribers — always the same ─────────────────────────────
        self.create_subscription(State,    '/mavros/state',               self._state_cb, qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self._pose_cb,  qos)
        if self._sim:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, '/joint_states', self._joint_cb, qos)
        # ── Subscribers — mode-dependent ──────────────────────────────
        if self._sim:
            self.create_subscription(
                Odometry, '/end_effector/pose', self._ee_cb_odom, qos)
            for i in range(1, MAX_TARGETS + 1):
                self.create_subscription(
                    Odometry, f'/target{i}/pose',
                    lambda msg, idx=i: self._target_cb_odom(msg, idx), qos)
        else:
            # print("real mode ")
            self.create_subscription(
                PoseStamped, '/vrpn_mocap/end_effector_1/pose', self._ee_cb_pose, qos)
            for i in range(1, MAX_TARGETS + 1):
                self.create_subscription(
                    PoseStamped, f'/vrpn_mocap/target_{i}/pose',
                    lambda msg, idx=i: self._target_cb_pose(msg, idx), qos)

        # ── Publishers ────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos)
        self._pub_arm = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)

        self.timer = self.create_timer(1.0 / CTRL_HZ, self.timer_callback)

        # Publish initial joint angles once at startup
        init_j1 = 0.0
        init_j2 = 1.57
        self._pub_arm.publish(Float64MultiArray(data=[init_j1, init_j2]))
        self.q[3] = init_j1   # seed q with init angles so sim starts correct
        self.q[4] = init_j2
        self.get_logger().info(
            'Init joints published | j1=%.1f° j2=%.1f°' % (
                math.degrees(init_j1), math.degrees(init_j2)))

        self.get_logger().info(
            'OffboardControl ready | arm reach=%.2fm | EE offset=+%.2fm Z' % (
                GIMBAL_Z + L1 + L2, EE_OFFSET_Z))

        # Set PX4 velocity limits at startup
        self._set_px4_params()

    # ── PX4 param setter ──────────────────────────────────────────────────

    def _set_px4_params(self):
        params = {
            'MPC_XY_VEL_MAX':   0.5,   # m/s horizontal
            'MPC_Z_VEL_MAX_UP': 0.5,   # m/s climb
            'MPC_Z_VEL_MAX_DN': 0.3,   # m/s descend
        }
        cli = self.create_client(ParamSetV2, '/mavros/param/set_v2')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('PX4 param service not available — velocity limits not set')
            return

        for name, value in params.items():
            req = ParamSetV2.Request()
            req.param_id = name
            req.value    = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value))
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().success:
                self.get_logger().info('PX4 param set | %s = %.2f' % (name, value))
            else:
                self.get_logger().warn('PX4 param set FAILED | %s' % name)

    # ── MAVROS callbacks ──────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        now_offboard = (msg.mode == 'OFFBOARD')

        if now_offboard and not self.offboard:
            self._offboard_detected_time = time.monotonic()
            self._offboard_delay_done    = False
            self._hold_position          = self.q[:3].copy()  # snapshot position at entry
            self.get_logger().info(
                'OFFBOARD detected | holding at [%.2f,%.2f,%.2f] for %.1fs' % (
                    *self._hold_position, OFFBOARD_DELAY))

        if not now_offboard and self.offboard:
            self._offboard_detected_time = None
            self._offboard_delay_done    = False
            self.get_logger().warn('OFFBOARD lost — delay reset')

        self.offboard = now_offboard

    def _pose_cb(self, msg: Odometry):
        self.q[0] = msg.pose.pose.position.x
        self.q[1] = msg.pose.pose.position.y
        self.q[2] = msg.pose.pose.position.z
        self._pose_received = True

    # ── Joint callback (sim only) ─────────────────────────────────────────

    def _joint_cb(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                val = msg.position[msg.name.index(name)]
                self.q[3+i] = float(np.clip(val, LIMITS[3+i,0], LIMITS[3+i,1]))

    # ── EE callbacks ──────────────────────────────────────────────────────

    def _ee_cb_odom(self, msg: Odometry):
        self.ee_pos       = pos_from_odometry(msg)
        self._ee_received = True

    def _ee_cb_pose(self, msg: PoseStamped):
        self.ee_pos       = pos_from_posestamped(msg)
        self._ee_received = True

    # ── Target callbacks ───────────────────────────────────────────────────

    def _target_cb_odom(self, msg: Odometry, idx: int):
        self._store_target(pos_from_odometry(msg), idx)

    def _target_cb_pose(self, msg: PoseStamped, idx: int):
        self._store_target(pos_from_posestamped(msg), idx)

    def _store_target(self, pos: np.ndarray, idx: int):
        if idx not in self.targets:
            self.get_logger().info('Target %d: [%.2f, %.2f, %.2f]' % (idx, *pos))
        self.targets[idx] = pos

    # ── Publish helpers ───────────────────────────────────────────────────

    def _hold(self):
        self._goto(self.q[0], self.q[1], self.q[2])

    def _goto(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)

    # ── Control loop ──────────────────────────────────────────────────────

    def timer_callback(self):

        # Phase 1: wait for sensors
        # print(self._pose_received,self._joints_init ,self._ee_received)

        if not self._pose_received or not self._joints_init or not self._ee_received:
            self.get_logger().info('Waiting for sensors...', throttle_duration_sec=2.0)
            return

        # Phase 2: not in offboard — hold and stream setpoints
        if not self.offboard:
            self._hold()
            self.get_logger().info(
                'Waiting OFFBOARD | drone=[%.2f,%.2f,%.2f] | targets=%s' % (
                    self.q[0], self.q[1], self.q[2],
                    list(self.targets.keys()) or 'none'),
                throttle_duration_sec=2.0)
            return

        # Phase 3: OFFBOARD delay — hold snapshot position
        if not self._offboard_delay_done:
            self._goto(*self._hold_position)  # frozen position, not live q
            elapsed   = time.monotonic() - self._offboard_detected_time
            remaining = OFFBOARD_DELAY - elapsed
            if remaining > 0:
                self.get_logger().info(
                    'OFFBOARD delay: %.1fs remaining — holding [%.2f,%.2f,%.2f]' % (
                        remaining, *self._hold_position),
                    throttle_duration_sec=1.0)
                return
            else:
                self._offboard_delay_done = True
                self.get_logger().info('OFFBOARD delay complete — starting control')

        # Phase 4: wait for target
        if self.target_idx not in self.targets:
            self._hold()
            self.get_logger().info(
                'Waiting /target%d/pose | known=%s' % (
                    self.target_idx,
                    list(self.targets.keys()) or 'none'),
                throttle_duration_sec=2.0)
            return

        # Safe height = highest known pole tip + full arm + offset
        safe_z = max(t[2] for t in self.targets.values()) + GIMBAL_Z + L1 + L2 + EE_OFFSET_Z

        # Phase 4a-1: climb to safe height (Z only)
        if not self._at_safe_height:
            self._goto(self.q[0], self.q[1], safe_z)
            if abs(self.q[2] - safe_z) < 0.10:
                self._at_safe_height = True
                self.get_logger().info(
                    'Safe height reached (%.2fm) — moving above target %d' % (
                        safe_z, self.target_idx))
            else:
                self.get_logger().info(
                    'Climbing | z=%.2f → %.2f' % (self.q[2], safe_z),
                    throttle_duration_sec=1.0)
            return

        # Phase 4a-2: move above target XY at safe height
        if not self._at_target_xy:
            target_xy = self.targets[self.target_idx]
            self._goto(target_xy[0], target_xy[1], safe_z)
            xy_dist = math.hypot(self.q[0] - target_xy[0], self.q[1] - target_xy[1])
            if xy_dist < 1:
                self._at_target_xy = True
                self.get_logger().info(
                    'Within 0.5m of target %d (xy_dist=%.2fm) — starting Jacobian approach' % (
                        self.target_idx, xy_dist))
            else:
                self.get_logger().info(
                    'Moving above target %d | xy_dist=%.2fm' % (
                        self.target_idx, xy_dist),
                    throttle_duration_sec=1.0)
            return

        # Phase 4b: hover wait after target reached
        if self._hovering:
            self._goto(*self._hover_position)  # frozen snapshot, not live q
            elapsed   = time.monotonic() - self._hover_start_time
            remaining = TARGET_HOVER_T - elapsed
            if remaining > 0:
                self.get_logger().info(
                    'Hovering at target %d | %.1fs remaining' % (
                        self.target_idx, remaining),
                    throttle_duration_sec=1.0)
                return
            else:
                # Hover complete — advance to next target
                self._hovering = False
                nxt = self.target_idx + 1
                if nxt in self.targets:
                    self.target_idx      = nxt
                    self._at_safe_height = False
                    self._at_target_xy   = False
                    self.get_logger().info(
                        '>>> Hover complete | Next: target %d [%.2f,%.2f,%.2f]' % (
                            nxt, *self.targets[nxt]))
                else:
                    self.get_logger().info('>>> All targets complete — returning home')
                    self._returning    = True
                    self._return_phase = 'climb'
                return

        # Phase 5: return home after all targets complete
        if self._returning:
            if self._return_phase == 'climb':
                self._goto(self.q[0], self.q[1], safe_z)
                if abs(self.q[2] - safe_z) < 0.10:
                    self._return_phase = 'center'
                    self.get_logger().info(
                        'Return: safe height reached (%.2fm) — moving to center' % safe_z)
                else:
                    self.get_logger().info(
                        'Return: climbing | z=%.2f → %.2f' % (self.q[2], safe_z),
                        throttle_duration_sec=1.0)
            elif self._return_phase == 'center':
                self._goto(self._center[0], self._center[1], safe_z)
                xy_dist = math.hypot(
                    self.q[0] - self._center[0],
                    self.q[1] - self._center[1])
                if xy_dist < 0.15:
                    self._return_phase = 'done'
                    self.get_logger().info('Return: reached center — holding')
                else:
                    self.get_logger().info(
                        'Return: moving to center | xy_dist=%.2fm' % xy_dist,
                        throttle_duration_sec=1.0)
            elif self._return_phase == 'done':
                self._goto(self._center[0], self._center[1], safe_z)
            return

        # Phase 4c: Jacobian approach
        goal = self.targets[self.target_idx] + np.array([0.0, 0.0, EE_OFFSET_Z])
        ee   = self.ee_pos
        err  = goal - ee
        dist = np.linalg.norm(err)

        reached = (abs(err[0]) < GOAL_THRESH and
                   abs(err[1]) < GOAL_THRESH and
                   abs(err[2]) < GOAL_THRESH)
        if reached:
            self._hold()
            self.get_logger().info(
                '>>> Reached target %d (err=%.3fm) | EE=[%.2f,%.2f,%.2f] | hovering %.1fs' % (
                    self.target_idx, dist, *ee, TARGET_HOVER_T))
            # Start hover timer — next target advance happens in Phase 4b
            self._hovering         = True
            self._hover_start_time = time.monotonic()
            self._hover_position   = self.q[:3].copy()  # snapshot now
            return

        hover_z  = self.targets[self.target_idx][2] + GIMBAL_Z + L1 + L2 + EE_OFFSET_Z
        xy_err   = math.hypot(err[0], err[1])
        pref_z   = hover_z if xy_err < 0.3 else safe_z
        q_pref   = np.array([self.q[0], self.q[1], pref_z, math.radians(60), math.radians(0)])

        J   = jacobian(self.q)
        Jp  = dls_pinv(J)
        N   = np.eye(len(self.q)) - Jp @ J
        dq  = ALPHA * Jp @ err + N @ (-BETA * (self.q - q_pref))

        dq[:3] = np.clip(dq[:3], -0.1, 0.1)
        dq[3:] = np.clip(dq[3:], -math.radians(5), math.radians(5))

        sp = np.clip(self.q + dq, LIMITS[:,0], LIMITS[:,1])

        self._goto(sp[0], sp[1], sp[2])
        self._pub_arm.publish(Float64MultiArray(data=[float(sp[3]), float(sp[4])]))

        self.get_logger().info(
            'T%d/%d | goal[%.2f,%.2f,%.2f] | ee[%.2f,%.2f,%.2f] | '
            'err[%.2f,%.2f,%.2f] d=%.3f | '
            'meas[%.2f,%.2f,%.2f] sp[%.2f,%.2f,%.2f] | '
            'j1=%.1f° j2=%.1f° | hover_z=%.2f' % (
                self.target_idx, len(self.targets),
                *goal, *ee, *err, dist,
                self.q[0], self.q[1], self.q[2],
                sp[0],     sp[1],     sp[2],
                math.degrees(sp[3]), math.degrees(sp[4]),
                hover_z))


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Offboard Jacobian controller')
    parser.add_argument('--sim',  action='store_true', help='Simulation mode  (Odometry topics)')
    parser.add_argument('--real', action='store_true', help='Real hardware mode (PoseStamped topics)')
    parsed, remaining = parser.parse_known_args(sys.argv[1:])

    if parsed.real and parsed.sim:
        print('ERROR: cannot pass both --sim and --real')
        sys.exit(1)

    mode = 'real' if parsed.real else 'sim'

    rclpy.init(args=remaining)
    node = OffboardControl(mode=mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()