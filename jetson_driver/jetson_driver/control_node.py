"""Control node — converts planner output into (throttle, steer, brake).

Rewritten 2026-05-18 to address the no-motion bug seen on the prototype.

The previous version applied the throttle floor as:
    elif 0.0 < throttle < self.throttle_min_pwm:
        throttle = self.throttle_min_pwm
which is correct for the un-inverted case but is tested BEFORE the
sign-flip needed for the chassis with reversed motor leads.  In the
operating regime of this bench — target speed below the floor's
break-even ratio — the floor was silently skipped, producing PWM levels
below the motor's static-friction break-away.

This rewrite computes the floor on the ABSOLUTE VALUE of the final
throttle, AFTER the sign flip, so the motor always sees enough PWM.

Additional changes:
  * On any transition from drive_enabled=False -> True, the controller
    arms a "kick" for DRIVE_KICK_FRAMES that pushes throttle to
    DRIVE_KICK_THROTTLE (1.0) regardless of target speed.  This breaks
    static friction at start-up.
  * Steer authority is multiplied by max(LANE_CONF_DEFAULT_FLOOR,
    lane_confidence).  When confidence is low, steer commands are
    attenuated so a noisy fit can't yank the wheel.
  * /lane_confidence is now subscribed (additive; defaults to 1.0 if no
    publisher is attached).

External contract preserved byte-identical from approach 2.
"""

import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Float32MultiArray

from .common import (
    CAM_W, CAM_H,
    CONTROL_HZ,
    STEER_KP, STEER_KI, STEER_KD, STEER_CLAMP,
    INTEGRAL_CLAMP, D_INPUT_CLAMP, DEAD_ZONE_PX, MAX_STEER_DELTA,
    THROTTLE_FULL_KMH, THROTTLE_IDLE, THROTTLE_MIN_PWM, BRAKE_REVERSE,
    LANE_CHANGE_STEER, PULL_OVER_STEER,
    LATERAL_CONTROLLER_DEFAULT,
    LOOKAHEAD_PX, WHEELBASE_PX_EQUIV, STEER_GAIN_PP,
    DRIVE_KICK_THROTTLE, DRIVE_KICK_FRAMES,
    PULSE_PERIOD_FRAMES, PULSE_MIN_ON_FRAMES,
    LANE_CONF_DEFAULT_FLOOR,
    Behavior,
)


_LATERAL_STATES = (
    Behavior.LANE_FOLLOW.value,
    Behavior.SLOW_FOR_OBSTACLE.value,
    Behavior.SEARCH_PARK.value,
)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ===========================================================================
#  Lateral controllers
# ===========================================================================
class PIDController:

    def __init__(self, params):
        self.kp = params['kp']
        self.ki = params['ki']
        self.kd = params['kd']
        self.integral_clamp = params['integral_clamp']
        self.d_input_clamp = params['d_input_clamp']
        self.dead_zone_px = params['dead_zone_px']
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def compute(self, lane_err_px, lane_found):
        err = float(lane_err_px)
        effective_err = err if abs(err) > self.dead_zone_px else 0.0
        if lane_found and effective_err != 0.0:
            self._integral += effective_err
            self._integral = _clamp(self._integral,
                                     -self.integral_clamp,
                                     self.integral_clamp)
        self._integral *= 0.995
        d_input = _clamp(err - self._prev_err,
                          -self.d_input_clamp, self.d_input_clamp)
        self._prev_err = err
        steer = (self.kp * effective_err
                 + self.ki * self._integral
                 + self.kd * d_input)
        if not lane_found:
            steer *= 0.5
        return steer


class PurePursuitController:

    def __init__(self, params):
        self.lookahead_px = float(params['lookahead_px'])
        self.wheelbase = float(params['wheelbase_px_equiv'])
        self.gain = float(params['steer_gain_pp'])
        self.img_w = int(params.get('img_w', CAM_W))
        self.img_h = int(params.get('img_h', CAM_H))
        self._poly = None
        self._poly_t = 0.0

    def reset(self):
        pass

    def update_poly(self, poly):
        if poly is not None and len(poly) == 3:
            self._poly = (float(poly[0]), float(poly[1]), float(poly[2]))
            self._poly_t = time.time()

    def compute(self, lane_err_norm, lane_found):
        if self._poly is None or (time.time() - self._poly_t) > 0.5:
            return _clamp(self.gain * 0.4 * float(lane_err_norm), -1.0, 1.0)
        a, b, c = self._poly
        roi_h_est = max(40.0, 0.45 * self.img_h)
        y_eval = roi_h_est - 1.0 - self.lookahead_px
        y_eval = max(0.0, y_eval)
        x_ahead = a * y_eval * y_eval + b * y_eval + c
        x_offset = x_ahead - (self.img_w * 0.5)
        forward = (roi_h_est - y_eval)
        if forward <= 1.0:
            return 0.0
        alpha = math.atan2(x_offset, forward)
        Ld = max(1.0, math.hypot(forward, x_offset))
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        steer = _clamp(self.gain * delta, -1.0, 1.0)
        if not lane_found:
            steer *= 0.5
        return steer


# ===========================================================================
#  Control node
# ===========================================================================
class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('steer_kp', STEER_KP)
        self.declare_parameter('steer_ki', STEER_KI)
        self.declare_parameter('steer_kd', STEER_KD)
        self.declare_parameter('steer_clamp', STEER_CLAMP)
        self.declare_parameter('integral_clamp', INTEGRAL_CLAMP)
        self.declare_parameter('d_input_clamp', D_INPUT_CLAMP)
        self.declare_parameter('dead_zone_px', DEAD_ZONE_PX)
        self.declare_parameter('max_steer_delta', MAX_STEER_DELTA)
        self.declare_parameter('throttle_full_kmh', THROTTLE_FULL_KMH)
        self.declare_parameter('throttle_min_pwm', THROTTLE_MIN_PWM)
        self.declare_parameter('brake_reverse', BRAKE_REVERSE)
        self.declare_parameter('invert_steer', False)
        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('lane_change_steer', LANE_CHANGE_STEER)
        self.declare_parameter('pull_over_steer', PULL_OVER_STEER)
        self.declare_parameter('lateral_controller', LATERAL_CONTROLLER_DEFAULT)
        self.declare_parameter('lookahead_px', LOOKAHEAD_PX)
        self.declare_parameter('wheelbase_px_equiv', WHEELBASE_PX_EQUIV)
        self.declare_parameter('steer_gain_pp', STEER_GAIN_PP)
        self.declare_parameter('drive_kick_throttle', DRIVE_KICK_THROTTLE)
        self.declare_parameter('drive_kick_frames', DRIVE_KICK_FRAMES)
        self.declare_parameter('pulse_period_frames', PULSE_PERIOD_FRAMES)
        self.declare_parameter('pulse_min_on_frames', PULSE_MIN_ON_FRAMES)
        self.declare_parameter('enable_pulse_mode', True)

        self.steer_clamp = float(self.get_parameter('steer_clamp').value)
        self.max_steer_delta = float(self.get_parameter('max_steer_delta').value)
        self.throttle_full_kmh = float(self.get_parameter('throttle_full_kmh').value)
        self.throttle_min_pwm = float(self.get_parameter('throttle_min_pwm').value)
        self.brake_reverse = float(self.get_parameter('brake_reverse').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.invert_throttle = bool(self.get_parameter('invert_throttle').value)
        self.lane_change_steer = float(self.get_parameter('lane_change_steer').value)
        self.pull_over_steer = float(self.get_parameter('pull_over_steer').value)
        self.lateral_kind = str(self.get_parameter('lateral_controller').value).strip().lower()
        self.drive_kick_throttle = float(self.get_parameter('drive_kick_throttle').value)
        self.drive_kick_frames = int(self.get_parameter('drive_kick_frames').value)
        self.pulse_period_frames = max(1, int(self.get_parameter('pulse_period_frames').value))
        self.pulse_min_on_frames = max(1, int(self.get_parameter('pulse_min_on_frames').value))
        self.enable_pulse_mode   = bool(self.get_parameter('enable_pulse_mode').value)

        pid_params = dict(
            kp=float(self.get_parameter('steer_kp').value),
            ki=float(self.get_parameter('steer_ki').value),
            kd=float(self.get_parameter('steer_kd').value),
            integral_clamp=float(self.get_parameter('integral_clamp').value),
            d_input_clamp=float(self.get_parameter('d_input_clamp').value),
            dead_zone_px=float(self.get_parameter('dead_zone_px').value),
        )
        pp_params = dict(
            lookahead_px=float(self.get_parameter('lookahead_px').value),
            wheelbase_px_equiv=float(self.get_parameter('wheelbase_px_equiv').value),
            steer_gain_pp=float(self.get_parameter('steer_gain_pp').value),
        )
        self._pid = PIDController(pid_params)
        self._pp = PurePursuitController(pp_params)

        if self.lateral_kind not in ('pid', 'pure_pursuit'):
            self.get_logger().warn(
                f"unknown lateral_controller '{self.lateral_kind}' — using 'pid'")
            self.lateral_kind = 'pid'

        self.create_subscription(Float32, '/lane_error', self._lane_err_cb, 10)
        self.create_subscription(Float32, '/lane_error_norm', self._lane_err_norm_cb, 10)
        self.create_subscription(Float32MultiArray, '/lane_poly', self._lane_poly_cb, 10)
        self.create_subscription(Bool, '/lane_found', self._lane_found_cb, 10)
        self.create_subscription(Float32, '/lane_confidence', self._lane_conf_cb, 10)
        self.create_subscription(Float32, '/target_speed', self._target_cb, 10)
        self.create_subscription(String, '/behavior', self._behavior_cb, 10)
        self.create_subscription(Bool, '/drive_enabled', self._drive_cb, 10)

        self.throttle_pub = self.create_publisher(Float32, '/throttle_cmd', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer_cmd', 10)
        self.brake_pub = self.create_publisher(Float32, '/brake_cmd', 10)

        self._lane_err = 0.0
        self._lane_err_norm = 0.0
        self._lane_found = False
        self._lane_conf = 1.0   # additive topic — default to full authority
        self._target_kmh = 0.0
        self._behavior = Behavior.HOLD.value
        self._prev_behavior = Behavior.HOLD.value
        self._drive = False
        self._prev_target = 0.0
        self._last_tgt_t = 0.0
        self._prev_steer = 0.0
        self._kick_remaining = 0
        self._pulse_phase = 0   # 0 .. pulse_period_frames-1, advances each tick

        self.create_timer(1.0 / max(1, CONTROL_HZ), self._tick)

        self.get_logger().info(
            f"control_node up: lateral={self.lateral_kind} "
            f"clamp={self.steer_clamp:.2f} slew={self.max_steer_delta:.2f}/tick "
            f"full_kmh={self.throttle_full_kmh:.1f} "
            f"min_pwm={self.throttle_min_pwm:.2f} "
            f"invert_steer={self.invert_steer} "
            f"invert_throttle={self.invert_throttle} "
            f"kick={self.drive_kick_throttle:.2f}x{self.drive_kick_frames}f "
            f"pulse={'on' if self.enable_pulse_mode else 'off'}"
            f" period={self.pulse_period_frames}f min_on={self.pulse_min_on_frames}f")

    def _lane_err_cb(self, m):       self._lane_err = float(m.data)
    def _lane_err_norm_cb(self, m):  self._lane_err_norm = float(m.data)
    def _lane_found_cb(self, m):     self._lane_found = bool(m.data)
    def _lane_conf_cb(self, m):      self._lane_conf = float(m.data)
    def _lane_poly_cb(self, m):      self._pp.update_poly(list(m.data))

    def _target_cb(self, m):
        self._target_kmh = float(m.data)
        self._last_tgt_t = time.time()

    def _behavior_cb(self, m):
        new_beh = str(m.data)
        if new_beh in _LATERAL_STATES and self._behavior not in _LATERAL_STATES:
            self._pid.reset()
            self._pp.reset()
            # CRITICAL: wipe the slew-limit memory so the PID can jump
            # straight to its commanded value on the merge-exit edge.
            # Without this, _prev_steer retains the merge open-loop
            # +0.85 and the slew limiter clamps any leftward correction
            # to -0.15/tick -- meaning the car keeps steering right for
            # ~0.3-0.5 s after the merge officially ends, often long
            # enough to drive off-track.  Resetting to 0 lets the PID
            # take full authority immediately on re-entry.
            self._prev_steer = 0.0
        self._prev_behavior = self._behavior
        self._behavior = new_beh

    def _drive_cb(self, m):
        was_driving = self._drive
        self._drive = bool(m.data)
        if self._drive and not was_driving:
            self._pid.reset()
            self._pp.reset()
            self._kick_remaining = self.drive_kick_frames
            self.get_logger().info(
                f"drive armed — kicking throttle for {self._kick_remaining} frames")

    # ------------------------------------------------------------------
    def _tick(self):
        now = time.time()
        stale = (now - self._last_tgt_t) > 0.5 if self._last_tgt_t > 0 else True
        if not self._drive or stale:
            self._pid.reset()
            self._pp.reset()
            self._publish(0.0, 0.0, 1.0)
            return

        # ---- lateral ----
        if self._behavior == Behavior.CHANGE_LANE_RIGHT.value:
            self._pid.reset()
            self._pp.reset()
            steer_raw = self.lane_change_steer
        elif self._behavior == Behavior.PULL_OVER.value:
            self._pid.reset()
            self._pp.reset()
            steer_raw = self.pull_over_steer
        elif self._behavior in (Behavior.HOLD.value, Behavior.PARKED.value):
            self._pid.reset()
            self._pp.reset()
            steer_raw = 0.0
        else:
            if self.lateral_kind == 'pure_pursuit':
                steer_raw = self._pp.compute(self._lane_err_norm,
                                              self._lane_found)
            else:
                steer_raw = self._pid.compute(self._lane_err,
                                               self._lane_found)
            # Confidence-weighted attenuation only in lane-following
            # states; open-loop steers in CHANGE_LANE_RIGHT / PULL_OVER
            # bypass this.
            authority = max(LANE_CONF_DEFAULT_FLOOR, self._lane_conf)
            steer_raw *= authority

        if self.invert_steer:
            steer_raw = -steer_raw

        delta = _clamp(steer_raw - self._prev_steer,
                        -self.max_steer_delta, self.max_steer_delta)
        steer = _clamp(self._prev_steer + delta,
                        -self.steer_clamp, self.steer_clamp)
        self._prev_steer = steer

        # ---- throttle / brake ----
        brake = 0.0
        target = max(0.0, self._target_kmh)
        decel_rate = (self._prev_target - target) / max(1e-3, 1.0 / CONTROL_HZ)
        self._prev_target = target

        if (self._behavior in (Behavior.HOLD.value, Behavior.PARKED.value)
                or target < 0.1):
            throttle_mag = 0.0
            brake = 1.0
            # Reset pulse phase so the next start is clean.
            self._pulse_phase = 0
        else:
            # Desired duty (linear in target_kmh).  0.0..1.0.
            duty = target / max(1e-3, self.throttle_full_kmh)
            duty = max(0.0, min(1.0, duty))

            # Drive-arm kick — bypasses pulse mode, runs full PWM for
            # drive_kick_frames to guarantee static-friction break-away.
            if self._kick_remaining > 0:
                throttle_mag = max(self.drive_kick_throttle,
                                   self.throttle_min_pwm)
                self._kick_remaining -= 1
                # Keep phase aligned so the post-kick pulse train starts
                # at the beginning of a cycle.
                self._pulse_phase = 0
            elif decel_rate > 8.0:
                # Brief coast frame on big speed-down transitions.
                throttle_mag = 0.0
                brake = 0.0
                self._pulse_phase = 0
            elif self.enable_pulse_mode and duty < 1.0:
                # PULSE MODE: alternate ON (=throttle_min_pwm) and OFF (=0)
                # frames within a fixed-length cycle.  Effective speed
                # ≈ (on_frames / pulse_period_frames) × full speed.
                period = self.pulse_period_frames
                on_frames = int(round(duty * period))
                # Always at least pulse_min_on_frames so the motor breaks
                # static friction on every cycle (no stall at low duty).
                on_frames = max(self.pulse_min_on_frames, on_frames)
                on_frames = min(period, on_frames)
                if self._pulse_phase < on_frames:
                    throttle_mag = self.throttle_min_pwm
                else:
                    throttle_mag = 0.0
                self._pulse_phase = (self._pulse_phase + 1) % period
            else:
                # Pulse mode disabled or duty ≥ 1.0 — run at floor / full.
                throttle_mag = max(self.throttle_min_pwm, duty)
                throttle_mag = min(1.0, throttle_mag)
                self._pulse_phase = 0

        # Sign goes on AFTER the floor.
        throttle = -throttle_mag if self.invert_throttle else throttle_mag

        self._publish(throttle, steer, brake)

    def _publish(self, throttle, steer, brake):
        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steer_pub.publish(Float32(data=float(steer)))
        self.brake_pub.publish(Float32(data=float(brake)))


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0, 1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
