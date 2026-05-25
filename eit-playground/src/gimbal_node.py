#!/usr/bin/env python3
"""
gimbal_node.py  –  2-axis gimbal stabilisation node (ROS 2 / Python)

Subscriptions
    /mavros/imu/data              sensor_msgs/Imu

Publications
    /gimbal_controller/commands   std_msgs/Float64MultiArray
        data[0] = gimbal_pitch_joint  (rad, position)
        data[1] = gimbal_roll_joint   (rad, position)

        Controller YAML:
            joints: [gimbal_pitch_joint, gimbal_roll_joint]
            interface_name: position

Parameters (all declared under the node namespace)
    pitch_kp/ki/kd   PID gains for pitch axis        (1.5 / 0.05 / 0.20)
    roll_kp/ki/kd    PID gains for roll  axis        (1.5 / 0.05 / 0.20)
    pitch_limit_rad  max gimbal pitch travel          (0.5236 = ±30°)
    roll_limit_rad   max gimbal roll  travel          (0.5236 = ±30°)
    lp_alpha         IIR low-pass weight ∈ (0,1]      (0.35)
    setpoint_pitch   world-frame pitch target (rad)  (0.0)
    setpoint_roll    world-frame roll  target (rad)  (pi/2 = 90°)

Design notes
    • IMU quaternion → ZYX Euler roll + pitch via quat_to_roll_pitch()
    • First-order IIR filter removes high-freq vibration
    • PID with derivative-on-measurement (no kick on setpoint changes)
    • Integral anti-windup via symmetric clamp
    • Joint order in data[] matches the controller YAML declaration
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


# ─────────────────────────────────────────────────────────────────────────────
# Pure-maths helpers  (importable without ROS for unit testing)
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_roll_pitch(x: float, y: float, z: float, w: float):
    """
    Unit quaternion (ZYX / aerospace convention) → (roll_rad, pitch_rad).
    Yaw is intentionally ignored – a 2-axis gimbal has no yaw joint.
    Pitch is clamped to ±π/2 to stay numerically safe near gimbal lock.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


def wrap_angle(a: float) -> float:
    """Wrap any angle into (-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ─────────────────────────────────────────────────────────────────────────────
# PID controller
# ─────────────────────────────────────────────────────────────────────────────

class PID:
    """
    Discrete PID with:
      • Derivative on measurement  – eliminates setpoint-step spikes
      • Integral anti-windup clamp
      • Symmetric output saturation
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float, out_max: float,
                 i_limit: float = 0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_limit = i_limit

        self._integral   = 0.0
        self._prev_meas  = None      # used for derivative-on-measurement

    def reset(self):
        self._integral  = 0.0
        self._prev_meas = None

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """Return the PID output for one timestep."""
        if dt <= 0.0:
            return 0.0

        error = wrap_angle(setpoint - measurement)

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral = max(
            -self.i_limit,
            min(self.i_limit, self._integral + error * dt),
        )
        i = self.ki * self._integral

        # Derivative on measurement (no kick)
        if self._prev_meas is None:
            d = 0.0
        else:
            d = -self.kd * (measurement - self._prev_meas) / dt
        self._prev_meas = measurement

        return max(self.out_min, min(self.out_max, p + i + d))


# ─────────────────────────────────────────────────────────────────────────────
# Gimbal node
# ─────────────────────────────────────────────────────────────────────────────

class GimbalNode(Node):

    # Joint order must stay in sync with the controller YAML:
    #   joints: [gimbal_pitch_joint, gimbal_roll_joint]
    JOINT_ORDER = ['gimbal_pitch_joint', 'gimbal_roll_joint']

    def __init__(self):
        super().__init__('gimbal_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('pitch_kp',        3)
        self.declare_parameter('pitch_ki',        0.05)
        self.declare_parameter('pitch_kd',        0.20)

        self.declare_parameter('roll_kp',         3)
        self.declare_parameter('roll_ki',         0.05)
        self.declare_parameter('roll_kd',         0.20)

        self.declare_parameter('pitch_limit_rad', 0.5236)   # ±30°
        self.declare_parameter('roll_limit_rad',  0.5236)   # ±30°

        self.declare_parameter('lp_alpha',        0.35)
        self.declare_parameter('setpoint_pitch',  0.0)
        self.declare_parameter('setpoint_roll',   math.pi / 2)  # 90°

        p = self.get_parameter
        pitch_lim      = p('pitch_limit_rad').value
        roll_lim       = p('roll_limit_rad').value
        self.lp_alpha  = p('lp_alpha').value
        self.sp_pitch  = p('setpoint_pitch').value
        self.sp_roll   = p('setpoint_roll').value

        # ── PID controllers ───────────────────────────────────────────
        self.pid_pitch = PID(
            kp=p('pitch_kp').value, ki=p('pitch_ki').value,
            kd=p('pitch_kd').value, out_min=-pitch_lim, out_max=pitch_lim,
        )
        self.pid_roll = PID(
            kp=p('roll_kp').value,  ki=p('roll_ki').value,
            kd=p('roll_kd').value,  out_min=-roll_lim,  out_max=roll_lim,
        )

        # ── State ─────────────────────────────────────────────────────
        self._filt_pitch = 0.0
        self._filt_roll  = 0.0
        self._last_t     = None    # rclpy.Time of previous callback

        # ── QoS – BEST_EFFORT matches the default MAVROS IMU profile ──
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._sub = self.create_subscription(
            Imu, '/mavros/imu/data', self._cb_imu, imu_qos,
        )
        self._pub = self.create_publisher(
            Float64MultiArray, '/gimbal_controller/commands', 10,
        )

        self.get_logger().info(
            f'GimbalNode ready  joints={self.JOINT_ORDER}  '
            f'limits: pitch=±{math.degrees(pitch_lim):.1f}°  '
            f'roll=±{math.degrees(roll_lim):.1f}°'
        )

    # ── Private helpers ───────────────────────────────────────────────

    def _get_dt(self) -> float:
        """Return elapsed seconds since last call; update internal stamp."""
        now = self.get_clock().now()
        if self._last_t is None:
            self._last_t = now
            return 0.02          # assume 50 Hz on very first tick
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        return max(dt, 1e-6)     # guard against zero / clock jitter

    def _low_pass(self, raw_pitch: float, raw_roll: float):
        a = self.lp_alpha
        self._filt_pitch = a * raw_pitch + (1.0 - a) * self._filt_pitch
        self._filt_roll  = a * raw_roll  + (1.0 - a) * self._filt_roll

    @staticmethod
    def _make_msg(pitch_cmd: float, roll_cmd: float) -> Float64MultiArray:
        """Pack joint commands into a properly annotated Float64MultiArray."""
        msg = Float64MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label='joints', size=2, stride=2)],
            data_offset=0,
        )
        msg.data = [pitch_cmd, roll_cmd]   # index order == JOINT_ORDER
        return msg

    # ── IMU callback ──────────────────────────────────────────────────

    def _cb_imu(self, imu: Imu):
        dt = self._get_dt()

        q = imu.orientation
        raw_pitch, raw_roll = quat_to_roll_pitch(q.x, q.y, q.z, q.w)

        self._low_pass(raw_pitch, raw_roll)

        pitch_cmd = self.pid_pitch.compute(self.sp_pitch, self._filt_pitch, dt)
        roll_cmd  = self.pid_roll.compute(self.sp_roll,   self._filt_roll,  dt)

        self._pub.publish(self._make_msg(pitch_cmd, roll_cmd))

        self.get_logger().debug(
            f'IMU  roll={math.degrees(raw_roll):+.2f}°  '
            f'pitch={math.degrees(raw_pitch):+.2f}° | '
            f'cmd  pitch={math.degrees(pitch_cmd):+.3f}°  '
            f'roll={math.degrees(roll_cmd):+.3f}° | dt={dt*1e3:.1f}ms'
        )


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()