"""Planner node — single-path FSM with hard minimum-dwell-time gating.

Rewritten 2026-05-18 to kill the mode-jumping observed in approach 1.

KEY ANTI-CHATTER MECHANISMS:

  1.  Every transition out of a state is gated on TWO conditions in AND:
        * a debounced sensor condition (the sensor topics are now
          debounced upstream in perception_node), and
        * a minimum-dwell-frame counter in the source state.
      Both must hold before _set() is called.

  2.  After every _set(), a transition-cooldown counter is armed.  While
      the cooldown is non-zero, NO transition will fire — the planner
      ticks the current state but refuses to flip.  This kills the
      pathological "ping-pong across one frame" failure mode that
      approach 1 hit when the lane detector chattered.

  3.  Counter advancement is gated on /lane_found being True (i.e. the
      perception layer is producing a stable signal).  A perception
      dropout PAUSES the FSM instead of marching the dwell counters
      forward on a clock.

  4.  Exactly one merge per arm cycle.  After CHANGE_LANE_RIGHT
      completes, _merged_once latches True.  Future obstacle events are
      handled by SLOW_FOR_OBSTACLE, never another merge.  On a 2-lane
      track there is only ever one merge.

External contract preserved byte-identical from approach 2.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

from .common import (
    Behavior, PLANNER_HZ,
    SPEED_STRAIGHT_KMH, SPEED_OBSTACLE_SLOW_KMH,
    SPEED_PARK_APPROACH_KMH, SPEED_LANE_CHANGE_KMH,
    PARK_SPEED_DECAY, PARK_HOLD_SPEED_KMH,
    LC_OBSTACLE_CLEAR_FRAMES, LC_SAFE_BLINDSPOT_FRAMES,
    POST_CHANGE_DWELL_FRAMES, PARK_SAFE_BLINDSPOT_FRAMES,
    MIN_MERGE_FRAMES, MERGE_COMPLETE_ERR_PX, MERGE_SETTLED_FRAMES_REQ,
    MIN_DWELL_IN_STATE_FRAMES, TRANSITION_COOLDOWN_FRAMES,
    AUTO_LANECHANGE_TIMEOUT_S, ENABLE_AUTO_LANECHANGE,
    LANE_LOST_TIMEOUT_S, ENABLE_LANE_LOST_STOP,
    DISABLE_OBSTACLE_AFTER_MERGE,
)


# States in which the lane detector MUST be producing /lane_found=True.
# Out of these states the lane-lost watchdog is bypassed (CHANGE_LANE_RIGHT
# legitimately drops lane mid-merge; PULL_OVER is open-loop; HOLD / PARKED
# aren't moving).
_LANE_REQUIRED_STATES = (
    Behavior.LANE_FOLLOW,
    Behavior.SLOW_FOR_OBSTACLE,
    Behavior.SEARCH_PARK,
)


LINK_BOOT_GRACE_S = 10.0


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        # --- params -------------------------------------------------------
        self.declare_parameter('enable_lane_changes',      True)
        self.declare_parameter('enable_parking',           True)
        self.declare_parameter('require_trigger',          False)
        self.declare_parameter('require_blindspot_clear', False)
        self.declare_parameter('lc_safe_frames_to_merge',
                                LC_SAFE_BLINDSPOT_FRAMES)
        self.declare_parameter('lc_obstacle_clear_frames',
                                LC_OBSTACLE_CLEAR_FRAMES)
        self.declare_parameter('post_change_dwell_frames',
                                POST_CHANGE_DWELL_FRAMES)
        self.declare_parameter('park_safe_frames',
                                PARK_SAFE_BLINDSPOT_FRAMES)
        self.declare_parameter('min_merge_frames',
                                MIN_MERGE_FRAMES)
        self.declare_parameter('merge_complete_err_px',
                                MERGE_COMPLETE_ERR_PX)
        self.declare_parameter('merge_settled_frames_req',
                                MERGE_SETTLED_FRAMES_REQ)
        self.declare_parameter('min_dwell_in_state_frames',
                                MIN_DWELL_IN_STATE_FRAMES)
        self.declare_parameter('transition_cooldown_frames',
                                TRANSITION_COOLDOWN_FRAMES)
        # Fallback lane-change knobs (Option C: manual + auto):
        self.declare_parameter('enable_auto_lanechange',
                                ENABLE_AUTO_LANECHANGE)
        self.declare_parameter('auto_lanechange_timeout_s',
                                AUTO_LANECHANGE_TIMEOUT_S)
        self.declare_parameter('auto_lanechange_log_every_s', 2.0)
        # Lane-lost safety watchdog:
        self.declare_parameter('enable_lane_lost_stop',
                                ENABLE_LANE_LOST_STOP)
        self.declare_parameter('lane_lost_timeout_s',
                                LANE_LOST_TIMEOUT_S)
        self.declare_parameter('lane_lost_warn_every_s', 0.5)
        # Post-merge obstacle muting:
        self.declare_parameter('disable_obstacle_after_merge',
                                DISABLE_OBSTACLE_AFTER_MERGE)

        self.enable_lane_changes      = bool(self.get_parameter('enable_lane_changes').value)
        self.enable_parking           = bool(self.get_parameter('enable_parking').value)
        self.require_trigger          = bool(self.get_parameter('require_trigger').value)
        self.require_blindspot_clear  = bool(self.get_parameter('require_blindspot_clear').value)
        self.lc_safe_frames_to_merge  = int(self.get_parameter('lc_safe_frames_to_merge').value)
        self.lc_obstacle_clear_frames = int(self.get_parameter('lc_obstacle_clear_frames').value)
        self.post_change_dwell_frames = int(self.get_parameter('post_change_dwell_frames').value)
        self.park_safe_frames         = int(self.get_parameter('park_safe_frames').value)
        self.min_merge_frames         = int(self.get_parameter('min_merge_frames').value)
        self.merge_complete_err_px    = float(self.get_parameter('merge_complete_err_px').value)
        self.merge_settled_frames_req = int(self.get_parameter('merge_settled_frames_req').value)
        self.min_dwell_in_state_frames = int(self.get_parameter('min_dwell_in_state_frames').value)
        self.transition_cooldown_frames = int(self.get_parameter('transition_cooldown_frames').value)
        self.enable_auto_lanechange    = bool(self.get_parameter('enable_auto_lanechange').value)
        self.auto_lanechange_timeout_s = float(self.get_parameter('auto_lanechange_timeout_s').value)
        self.auto_lanechange_log_every_s = float(self.get_parameter('auto_lanechange_log_every_s').value)
        self.enable_lane_lost_stop     = bool(self.get_parameter('enable_lane_lost_stop').value)
        self.lane_lost_timeout_s       = float(self.get_parameter('lane_lost_timeout_s').value)
        self.lane_lost_warn_every_s    = float(self.get_parameter('lane_lost_warn_every_s').value)
        self.disable_obstacle_after_merge = bool(self.get_parameter('disable_obstacle_after_merge').value)

        # --- subscriptions ------------------------------------------------
        self.create_subscription(Float32, '/lane_error',           self._lane_err_cb,    10)
        self.create_subscription(Bool,    '/lane_found',           self._lane_found_cb,  10)
        self.create_subscription(Bool,    '/obstacle_ahead',       self._obst_cb,        10)
        self.create_subscription(Bool,    '/yolo_blindspot_right', self._blind_cb,       10)
        self.create_subscription(String,  '/start_parking',        self._trigger_cb,     10)
        self.create_subscription(Bool,    '/emergency_stop',       self._estop_cb,       10)
        self.create_subscription(Bool,    '/trigger_link_alive',   self._link_cb,        10)

        # --- publishers ---------------------------------------------------
        self.speed_pub    = self.create_publisher(Float32, '/target_speed',  10)
        self.behavior_pub = self.create_publisher(String,  '/behavior',      10)
        self.drive_pub    = self.create_publisher(Bool,    '/drive_enabled', 10)

        # --- state --------------------------------------------------------
        self.state = Behavior.HOLD
        self._lane_err          = 0.0
        self._lane_found        = False
        self._obstacle_ahead    = False
        self._blindspot_blocked = False
        self._drive_enabled     = not self.require_trigger

        self._obstacle_clear_frames = 0
        self._post_change_frames    = 0
        self._park_safe_frames      = 0
        self._state_frames          = 0
        self._merge_settled_frames  = 0
        self._cooldown_frames       = 0   # anti-chatter — no transitions while >0

        self._merged_once = False
        self._pullover_speed = 0.0

        # Auto-lanechange fallback timer.  _drive_armed_t is the moment GO
        # was accepted; we measure the timeout against this so the user's
        # mental model — "X seconds after I press GO" — matches reality.
        self._drive_armed_t        = 0.0
        self._last_lanechange_log_t = 0.0

        # Lane-lost safety watchdog.  Refreshed every time we receive
        # /lane_found=True and on every drive-arm rising edge.
        self._last_lane_found_t    = time.time()
        self._last_lane_warn_t     = 0.0

        self._estop_latched = False
        self._link_alive   = True
        self._link_ever_up = False
        self._boot_t       = time.time()

        period = 1.0 / max(1, PLANNER_HZ)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"planner_node up: lane_changes={self.enable_lane_changes}, "
            f"parking={self.enable_parking}, "
            f"require_trigger={self.require_trigger}, "
            f"require_blindspot_clear={self.require_blindspot_clear}, "
            f"min_dwell={self.min_dwell_in_state_frames}f, "
            f"cooldown={self.transition_cooldown_frames}f, "
            f"auto_lanechange="
            f"{'on @ ' + format(self.auto_lanechange_timeout_s, '.1f') + 's' if self.enable_auto_lanechange else 'off'}, "
            f"lane_lost_stop="
            f"{'on @ ' + format(self.lane_lost_timeout_s, '.1f') + 's' if self.enable_lane_lost_stop else 'off'}, "
            f"post_merge_obst_mute="
            f"{'on' if self.disable_obstacle_after_merge else 'off'}")

    # ==================================================================
    #  Topic callbacks
    # ==================================================================
    def _lane_err_cb(self, msg):   self._lane_err   = float(msg.data)
    def _lane_found_cb(self, msg):
        self._lane_found = bool(msg.data)
        if self._lane_found:
            self._last_lane_found_t = time.time()
    def _obst_cb(self, msg):       self._obstacle_ahead    = bool(msg.data)
    def _blind_cb(self, msg):      self._blindspot_blocked = bool(msg.data)

    def _trigger_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'go':
            if self._estop_latched:
                self.get_logger().warn(
                    "ignoring 'go' — emergency_stop latched")
                return
            if not self._link_alive_effective():
                self.get_logger().warn(
                    "ignoring 'go' — trigger ESP link is dead")
                return
            if not self._drive_enabled:
                self._drive_enabled = True
                self.get_logger().info("drive ENABLED")
                self._reset_counters()
                self._merged_once = False
                self._drive_armed_t = time.time()
                self._last_lanechange_log_t = self._drive_armed_t
                # Fresh lane-lost grace window starting from GO.
                self._last_lane_found_t = self._drive_armed_t
                self._last_lane_warn_t  = 0.0
                self._set(Behavior.LANE_FOLLOW, reason="GO trigger")
            else:
                if (self.enable_parking
                        and self.state not in (Behavior.SEARCH_PARK,
                                                Behavior.PULL_OVER,
                                                Behavior.PARKED,
                                                Behavior.HOLD)):
                    self._begin_park(reason="second GO trigger")

        elif cmd == 'park':
            if (self._drive_enabled and self.enable_parking
                    and self.state not in (Behavior.SEARCH_PARK,
                                            Behavior.PULL_OVER,
                                            Behavior.PARKED,
                                            Behavior.HOLD)):
                self._begin_park(reason="explicit park trigger")

        elif cmd in ('lanechange', 'merge', 'change', 'changelane',
                     'lane_change', 'change_lane'):
            self._force_lanechange(reason=f"manual '{cmd}' trigger")

        elif cmd in ('stop', 'hold'):
            self._set(Behavior.HOLD, reason="stop trigger")
            self._drive_enabled = False
            self._drive_armed_t = 0.0

    def _estop_cb(self, msg: Bool):
        latched = bool(msg.data)
        if latched and not self._estop_latched:
            self.get_logger().error(
                "EMERGENCY STOP — disarming drive, forcing HOLD")
            self._drive_enabled = False
            self._drive_armed_t = 0.0
            self._set(Behavior.HOLD, reason="estop latched")
        elif (not latched) and self._estop_latched:
            self.get_logger().info(
                "emergency_stop cleared — drive stays disarmed until next 'go'")
        self._estop_latched = latched

    def _link_cb(self, msg: Bool):
        alive = bool(msg.data)
        if alive:
            self._link_ever_up = True
        if (not alive) and self._link_alive:
            if self._in_boot_grace():
                return
            self.get_logger().error(
                "trigger ESP link dead — disarming drive, forcing HOLD")
            self._drive_enabled = False
            self._drive_armed_t = 0.0
            self._set(Behavior.HOLD, reason="trigger link dead")
        elif alive and not self._link_alive:
            self.get_logger().info(
                "trigger ESP link restored — drive stays disarmed until next 'go'")
        self._link_alive = alive

    # ==================================================================
    #  FSM tick
    # ==================================================================
    def _tick(self):
        # Always advertise drive_enabled so the control_node gate stays
        # current even for late subscribers.
        self.drive_pub.publish(Bool(data=self._drive_enabled))

        # Hard safety gates → HOLD regardless of state.
        link_ok = self._link_alive_effective()
        if self._estop_latched or not link_ok:
            self._drive_enabled = False
            self._publish(Behavior.HOLD, 0.0)
            return

        if not self._drive_enabled:
            self._publish(Behavior.HOLD, 0.0)
            return

        # ------------------------------------------------------------------
        # Lane-lost safety watchdog.  Only checks in states where /lane_found
        # *should* be True (LANE_FOLLOW / SLOW_FOR_OBSTACLE / SEARCH_PARK).
        # Bypassed in CHANGE_LANE_RIGHT (legitimate mid-merge dropout) and
        # PULL_OVER (open-loop steer, lane may be invisible).
        # ------------------------------------------------------------------
        if (self.enable_lane_lost_stop
                and self.state in _LANE_REQUIRED_STATES):
            since_lane = time.time() - self._last_lane_found_t
            if since_lane > self.lane_lost_timeout_s:
                self.get_logger().error(
                    f"LANE LOST for {since_lane:.2f}s "
                    f"(> {self.lane_lost_timeout_s:.1f}s) "
                    f"in state {self.state.value} -- "
                    f"forcing HOLD and disarming drive")
                self._drive_enabled = False
                self._drive_armed_t = 0.0
                self._set(Behavior.HOLD, reason="lane lost")
                self._publish(Behavior.HOLD, 0.0)
                return
            # Warn while the timer is more than halfway to the limit so
            # the operator gets advance notice.
            if (self.lane_lost_warn_every_s > 0.0
                    and since_lane > (0.5 * self.lane_lost_timeout_s)
                    and (time.time() - self._last_lane_warn_t)
                        >= self.lane_lost_warn_every_s):
                self.get_logger().warn(
                    f"lane lost {since_lane:.2f}s "
                    f"(stop in {self.lane_lost_timeout_s - since_lane:.2f}s "
                    f"if not recovered)")
                self._last_lane_warn_t = time.time()

        self._state_frames += 1
        if self._cooldown_frames > 0:
            self._cooldown_frames -= 1

        # Effective blindspot: ignored when require_blindspot_clear=False.
        blindspot_active = (self.require_blindspot_clear
                             and self._blindspot_blocked)

        if self.state == Behavior.LANE_FOLLOW:
            self._tick_lane_follow(blindspot_active)
        elif self.state == Behavior.SLOW_FOR_OBSTACLE:
            self._tick_slow(blindspot_active)
        elif self.state == Behavior.CHANGE_LANE_RIGHT:
            self._tick_change_lane_right(blindspot_active)
        elif self.state == Behavior.SEARCH_PARK:
            self._tick_search_park(blindspot_active)
        elif self.state == Behavior.PULL_OVER:
            self._tick_pull_over()
        elif self.state == Behavior.PARKED:
            self._publish(Behavior.PARKED, 0.0)
        else:
            self._publish(Behavior.HOLD, 0.0)

    # ------------------------------------------------------------------
    def _can_transition(self) -> bool:
        """A transition is allowed only after the minimum dwell AND when
        the cooldown is clear.  Returning False keeps the FSM in its
        current state for this tick — the planner falls through to its
        normal cruise speed publish."""
        return (self._state_frames >= self.min_dwell_in_state_frames
                and self._cooldown_frames == 0)

    # ------------------------------------------------------------------
    def _tick_lane_follow(self, blindspot_active: bool):
        # 0) Auto-lanechange fallback timer.  If we've been armed for
        #    auto_lanechange_timeout_s seconds without a merge having
        #    fired, force the merge.  Skipped if disabled, already
        #    merged, or drive wasn't armed yet.
        if (self.enable_auto_lanechange
                and self._drive_armed_t > 0.0
                and not self._merged_once):
            elapsed = time.time() - self._drive_armed_t
            remaining = self.auto_lanechange_timeout_s - elapsed
            # Periodic countdown log (default every 2 s).
            if (self.auto_lanechange_log_every_s > 0.0
                    and (time.time() - self._last_lanechange_log_t)
                        >= self.auto_lanechange_log_every_s
                    and remaining > 0):
                self.get_logger().info(
                    f"auto-lanechange countdown: "
                    f"{elapsed:.1f}s / {self.auto_lanechange_timeout_s:.1f}s "
                    f"(merge in {remaining:.1f}s if no obstacle)")
                self._last_lanechange_log_t = time.time()
            if elapsed >= self.auto_lanechange_timeout_s:
                if self._force_lanechange(
                        reason=f"auto-timeout {self.auto_lanechange_timeout_s:.1f}s"):
                    return

        # 1) Obstacle handling — debounced upstream.
        # Post-merge muting: once the (one allowed) lane change has
        # fired, ignore /obstacle_ahead so YOLO false-positives from
        # clutter near the parking strip or the decoy itself as we pass
        # it don't drag us into SLOW_FOR_OBSTACLE while we're heading
        # toward SEARCH_PARK.  Controlled by YAML param
        # planner_node.disable_obstacle_after_merge.
        obstacle_active = self._obstacle_ahead
        if (self.disable_obstacle_after_merge and self._merged_once):
            obstacle_active = False
        if obstacle_active and self._can_transition():
            can_merge_right = (self.enable_lane_changes
                                and not blindspot_active
                                and not self._merged_once)
            if can_merge_right:
                self._set(Behavior.CHANGE_LANE_RIGHT,
                          reason="obstacle ahead, blindspot clear")
                self._merged_once = True
                self._obstacle_clear_frames = 0
                self._publish(Behavior.CHANGE_LANE_RIGHT,
                              SPEED_LANE_CHANGE_KMH)
                return
            self._set(Behavior.SLOW_FOR_OBSTACLE,
                      reason="obstacle ahead, can't merge")
            self._publish(Behavior.SLOW_FOR_OBSTACLE, SPEED_OBSTACLE_SLOW_KMH)
            return

        # 2) Post-merge auto-park.
        if self.enable_parking and self._merged_once:
            if self._lane_found:
                self._post_change_frames += 1
            if (self._post_change_frames >= self.post_change_dwell_frames
                    and self._can_transition()):
                self._begin_park(
                    reason=f"post-merge dwell {self._post_change_frames}f done")
                return

        # 3) Default — cruise.
        self._publish(Behavior.LANE_FOLLOW, SPEED_STRAIGHT_KMH)

    # ------------------------------------------------------------------
    def _tick_slow(self, blindspot_active: bool):
        # If the obstacle clears (debounced upstream), return to cruise.
        if not self._obstacle_ahead and self._can_transition():
            self._set(Behavior.LANE_FOLLOW, reason="obstacle cleared")
            self._publish(Behavior.LANE_FOLLOW, SPEED_STRAIGHT_KMH)
            return
        self._publish(Behavior.SLOW_FOR_OBSTACLE, SPEED_OBSTACLE_SLOW_KMH)

    # ------------------------------------------------------------------
    def _tick_change_lane_right(self, blindspot_active: bool):
        # Abort merge if blindspot suddenly fires.
        if blindspot_active and self._can_transition():
            self._set(Behavior.SLOW_FOR_OBSTACLE,
                       reason="blindspot blocked mid-merge")
            self._publish(Behavior.SLOW_FOR_OBSTACLE,
                          SPEED_OBSTACLE_SLOW_KMH)
            return

        # Track "obstacle gone" and "lane settled" streaks.
        if not self._obstacle_ahead:
            self._obstacle_clear_frames += 1
        else:
            self._obstacle_clear_frames = 0

        settled_now = (self._lane_found and
                        abs(self._lane_err) <= self.merge_complete_err_px)
        if settled_now:
            self._merge_settled_frames += 1
        else:
            self._merge_settled_frames = 0

        # Minimum dwell of min_merge_frames is enforced absolutely.
        # The general min_dwell is a SHORTER guard; merge needs longer.
        long_enough = (self._state_frames >= self.min_merge_frames)
        reactive_exit = (self._obstacle_clear_frames
                          >= self.lc_obstacle_clear_frames)
        vision_exit = (long_enough
                       and self._merge_settled_frames
                              >= self.merge_settled_frames_req)

        if (reactive_exit or vision_exit) and long_enough and self._cooldown_frames == 0:
            reason = (f"lane settled {self._merge_settled_frames}f "
                       f"(|err|={abs(self._lane_err):.1f}px)"
                       if vision_exit else
                       f"obstacle clear {self._obstacle_clear_frames}f")
            self._set(Behavior.LANE_FOLLOW, reason=f"merge complete ({reason})")
            self._post_change_frames = 0
            self._publish(Behavior.LANE_FOLLOW, SPEED_STRAIGHT_KMH)
            return

        self._publish(Behavior.CHANGE_LANE_RIGHT, SPEED_LANE_CHANGE_KMH)

    # ------------------------------------------------------------------
    def _tick_search_park(self, blindspot_active: bool):
        if blindspot_active:
            self._park_safe_frames = 0
        else:
            self._park_safe_frames += 1
        if (self._park_safe_frames >= self.park_safe_frames
                and self._can_transition()):
            self._set(Behavior.PULL_OVER, reason="rear clear")
            self._pullover_speed = SPEED_PARK_APPROACH_KMH
            self._publish(Behavior.PULL_OVER, self._pullover_speed)
            return
        self._publish(Behavior.SEARCH_PARK, SPEED_PARK_APPROACH_KMH)

    # ------------------------------------------------------------------
    def _tick_pull_over(self):
        self._pullover_speed *= PARK_SPEED_DECAY
        if (self._pullover_speed < PARK_HOLD_SPEED_KMH
                and self._can_transition()):
            self._set(Behavior.PARKED, reason="speed ramp complete")
            self._publish(Behavior.PARKED, 0.0)
            return
        self._publish(Behavior.PULL_OVER, self._pullover_speed)

    # ==================================================================
    #  Helpers
    # ==================================================================
    def _begin_park(self, reason="park request"):
        self._set(Behavior.SEARCH_PARK, reason=reason)
        self._park_safe_frames = 0

    def _force_lanechange(self, reason: str = "manual") -> bool:
        """Fire CHANGE_LANE_RIGHT immediately, respecting hard safety
        gates (estop, link, drive armed, enable_lane_changes,
        _merged_once, blindspot if enabled) but bypassing the dwell /
        cooldown / obstacle-debounce gates.

        Used by:
          * manual operator trigger via /start_parking 'lanechange'
          * the auto-lanechange timeout fallback in _tick_lane_follow

        Returns True if the merge actually fired."""
        if self._estop_latched or not self._link_alive_effective():
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- safety gate "
                f"(estop={self._estop_latched}, link_ok="
                f"{self._link_alive_effective()})")
            return False
        if not self._drive_enabled:
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- drive not armed")
            return False
        if not self.enable_lane_changes:
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- lane changes disabled")
            return False
        if self._merged_once:
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- already merged once")
            return False
        if self.state in (Behavior.CHANGE_LANE_RIGHT,
                           Behavior.SEARCH_PARK,
                           Behavior.PULL_OVER,
                           Behavior.PARKED,
                           Behavior.HOLD):
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- bad state "
                f"{self.state.value}")
            return False
        if (self.require_blindspot_clear and self._blindspot_blocked):
            self.get_logger().warn(
                f"refusing lanechange ({reason}) -- blindspot blocked")
            return False
        self.get_logger().info(f"FORCED CHANGE_LANE_RIGHT ({reason})")
        self._set(Behavior.CHANGE_LANE_RIGHT, reason=reason)
        self._merged_once = True
        self._obstacle_clear_frames = 0
        self._publish(Behavior.CHANGE_LANE_RIGHT, SPEED_LANE_CHANGE_KMH)
        return True

    def _set(self, new: Behavior, reason: str = ""):
        if new != self.state:
            self.get_logger().info(
                f"state {self.state.value} -> {new.value}"
                + (f"  ({reason})" if reason else ""))
            self.state = new
            self._state_frames = 0
            self._merge_settled_frames = 0
            self._cooldown_frames = self.transition_cooldown_frames

    def _publish(self, state: Behavior, speed_kmh: float):
        self.speed_pub.publish(Float32(data=float(speed_kmh)))
        self.behavior_pub.publish(String(data=state.value))

    def _reset_counters(self):
        self._obstacle_clear_frames = 0
        self._post_change_frames    = 0
        self._park_safe_frames      = 0
        self._state_frames          = 0
        self._merge_settled_frames  = 0
        self._cooldown_frames       = 0

    def _in_boot_grace(self) -> bool:
        return (not self._link_ever_up and
                (time.time() - self._boot_t) < LINK_BOOT_GRACE_S)

    def _link_alive_effective(self) -> bool:
        if self._link_alive:
            return True
        return self._in_boot_grace()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
