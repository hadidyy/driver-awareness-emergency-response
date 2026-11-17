"""Real-scenario simulation for the unified rewritten jetson_driver stack.

Imports the REAL planner_node, control_node, perception_node, motor_bridge_node
modules via the ros_stub shim and drives them frame-by-frame.  Every
assertion exercises behaviour that has to hold on real hardware.

Run with:
    PYTHONPATH=sim:jetson_driver python3 sim/sim_runner.py
"""
import sys, io, time
# Force stdout to UTF-8 so the Windows console can print arrows etc.
try:
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass
import ros_stub  # patches sys.modules
from ros_stub import SimBus, Float32, Bool, String, Float32MultiArray

from jetson_driver.planner_node import PlannerNode
from jetson_driver.control_node import ControlNode
from jetson_driver.motor_bridge_node import MotorBridgeNode
from jetson_driver.common import (
    Behavior, MIN_DWELL_IN_STATE_FRAMES, TRANSITION_COOLDOWN_FRAMES,
    SPEED_STRAIGHT_KMH, SPEED_LANE_CHANGE_KMH, SPEED_OBSTACLE_SLOW_KMH,
    THROTTLE_FULL_KMH, THROTTLE_MIN_PWM, DRIVE_KICK_FRAMES,
    LANE_FOUND_TRUE_FRAMES, LANE_FOUND_FALSE_FRAMES,
    OBSTACLE_TRUE_FRAMES, OBSTACLE_FALSE_FRAMES,
)


PASS, FAIL = [], []
def check(name, cond, detail=""):
    if cond:
        print(f"  [PASS] {name}")
        PASS.append(name)
    else:
        print(f"  [FAIL] {name}  {detail}")
        FAIL.append((name, detail))


def banner(title):
    print()
    print("=" * 70)
    print(f"  {title}")
    print("=" * 70)


def feed(node, topic, msg):
    sub = node.subs_by_topic.get(topic)
    assert sub is not None, f"{node._name} has no subscription on {topic}"
    sub.cb(msg)


def step(p, c, *, lane_err=0.0, lane_found=True, obstacle=False, blindspot=False,
         lane_err_norm=None, lane_conf=1.0, link_alive=True, estop=False):
    """One synchronous tick of planner + control with shared percepts."""
    feed(p, '/lane_error', Float32(data=float(lane_err)))
    feed(c, '/lane_error', Float32(data=float(lane_err)))
    if lane_err_norm is None:
        lane_err_norm = max(-1.0, min(1.0, lane_err / 320.0))
    feed(c, '/lane_error_norm', Float32(data=float(lane_err_norm)))
    feed(p, '/lane_found', Bool(data=lane_found))
    feed(c, '/lane_found', Bool(data=lane_found))
    feed(c, '/lane_confidence', Float32(data=float(lane_conf)))
    feed(p, '/obstacle_ahead', Bool(data=obstacle))
    feed(p, '/yolo_blindspot_right', Bool(data=blindspot))
    feed(p, '/emergency_stop', Bool(data=estop))
    feed(p, '/trigger_link_alive', Bool(data=link_alive))
    p._tick()
    beh = SimBus.get('/behavior')
    tgt = SimBus.get('/target_speed')
    drv = SimBus.get('/drive_enabled')
    if beh is not None: feed(c, '/behavior', beh)
    if tgt is not None: feed(c, '/target_speed', tgt)
    if drv is not None: feed(c, '/drive_enabled', drv)
    c._tick()


def cruise_through_dwell(p, c, n=None):
    """Burn frames in the current state to satisfy min_dwell_in_state_frames
    + transition_cooldown_frames."""
    if n is None:
        n = max(MIN_DWELL_IN_STATE_FRAMES, TRANSITION_COOLDOWN_FRAMES) + 2
    for _ in range(n):
        step(p, c, lane_err=0.0)


def make_planner_with_trigger_gate():
    """PlannerNode whose drive_enabled defaults to False (i.e. require_trigger=True).
    The mock declare_parameter() uses the kwarg default; we'd otherwise auto-arm."""
    p = PlannerNode()
    p.require_trigger = True
    p._drive_enabled = False
    return p


# ===========================================================================
# SCENARIO 1 -- boot, GO, cruise with output-level throttle floor
# ===========================================================================
def scenario_1():
    banner("S1 -- boot + GO + cruise; verify throttle floor at output")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()

    step(p, c, link_alive=True)
    check("S1.boot: state == HOLD",
          p.state == Behavior.HOLD, f"got {p.state.value}")
    check("S1.boot: drive_enabled = False",
          SimBus.get_value('/drive_enabled') is False)
    check("S1.boot: target_speed = 0",
          SimBus.get_value('/target_speed') == 0.0)
    check("S1.boot: brake_cmd = 1",
          SimBus.get_value('/brake_cmd') == 1.0)
    check("S1.boot: throttle_cmd = 0",
          SimBus.get_value('/throttle_cmd') == 0.0)

    feed(p, '/start_parking', String(data='go'))
    step(p, c)
    check("S1.go: state LANE_FOLLOW",
          p.state == Behavior.LANE_FOLLOW, f"got {p.state.value}")
    check("S1.go: drive_enabled True",
          SimBus.get_value('/drive_enabled') is True)
    check("S1.go: target_speed == SPEED_STRAIGHT_KMH",
          SimBus.get_value('/target_speed') == SPEED_STRAIGHT_KMH)

    # First few frames: drive-arm kick is active (throttle should be 1.0).
    th_first = SimBus.get_value('/throttle_cmd')
    check("S1.kick: first tick after GO at full throttle (drive-arm kick)",
          abs(th_first - 1.0) < 1e-6,
          f"got {th_first:.3f}")

    # Burn through the kick frames AND let the min_dwell elapse.
    for _ in range(DRIVE_KICK_FRAMES + 2):
        step(p, c, lane_err=0.0)
    th_settled = SimBus.get_value('/throttle_cmd')
    # Post-kick: raw = 8/10 = 0.8, floor at 0.95 -> output 0.95
    # (invert_throttle defaults to False here, so positive).
    expected = max(SPEED_STRAIGHT_KMH / THROTTLE_FULL_KMH, THROTTLE_MIN_PWM)
    check("S1.cruise: settled throttle equals THROTTLE_MIN_PWM floor",
          abs(th_settled - expected) < 1e-6,
          f"got {th_settled:.4f} expected {expected:.4f}")
    check("S1.cruise: steer near 0 (centred lane)",
          abs(SimBus.get_value('/steer_cmd')) < 0.05,
          f"got {SimBus.get_value('/steer_cmd'):.3f}")
    check("S1.cruise: brake = 0",
          SimBus.get_value('/brake_cmd') == 0.0)


# ===========================================================================
# SCENARIO 2 -- obstacle -> CHANGE_LANE_RIGHT -> vision-aware exit
# ===========================================================================
def scenario_2():
    banner("S2 -- obstacle in ego lane -> CHANGE_LANE_RIGHT -> vision exit")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    # MUST cruise through min_dwell + cooldown before planner accepts merge.
    cruise_through_dwell(p, c)
    check("S2.setup: cruising in LANE_FOLLOW after dwell",
          p.state == Behavior.LANE_FOLLOW)

    step(p, c, lane_err=0.0, obstacle=True)
    check("S2.obstacle: planner -> CHANGE_LANE_RIGHT",
          p.state == Behavior.CHANGE_LANE_RIGHT,
          f"got {p.state.value}")
    check("S2.obstacle: target_speed = SPEED_LANE_CHANGE_KMH",
          SimBus.get_value('/target_speed') == SPEED_LANE_CHANGE_KMH)
    check("S2.obstacle: _merged_once latched",
          p._merged_once is True)

    last_steer = SimBus.get_value('/steer_cmd')
    check("S2.merge tick 1: steer is positive (ramping right)",
          last_steer > 0, f"got {last_steer:.3f}")
    # Slew limit: steer ramps to 0.85 over many ticks.
    for _ in range(10):
        step(p, c, lane_err=80.0, obstacle=False)
    steer_now = SimBus.get_value('/steer_cmd')
    check("S2.merge settled: steer == lane_change_steer (0.85)",
          abs(steer_now - 0.85) < 0.001, f"got {steer_now:.3f}")

    # Drive until we're past min_merge_frames + transition_cooldown, then
    # settle the lane to trigger vision-aware exit.
    while p._state_frames < p.min_merge_frames + 1:
        step(p, c, lane_err=80.0, obstacle=False)
    for _ in range(p.merge_settled_frames_req + 2):
        step(p, c, lane_err=15.0, obstacle=False)

    check("S2.exit: planner -> LANE_FOLLOW (vision exit)",
          p.state == Behavior.LANE_FOLLOW, f"got {p.state.value}")


# ===========================================================================
# SCENARIO 3 -- full park flow
# ===========================================================================
def scenario_3():
    banner("S3 -- post-merge dwell -> SEARCH_PARK -> PULL_OVER -> PARKED")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    cruise_through_dwell(p, c)

    step(p, c, obstacle=True)
    for _ in range(25): step(p, c, lane_err=80.0, obstacle=False)
    for _ in range(8): step(p, c, lane_err=10.0, obstacle=False)
    check("S3.post-merge: state LANE_FOLLOW",
          p.state == Behavior.LANE_FOLLOW)

    for _ in range(75):
        step(p, c, lane_err=5.0, obstacle=False, lane_found=True)
        if p.state == Behavior.SEARCH_PARK:
            break
    check("S3.dwell: planner -> SEARCH_PARK",
          p.state == Behavior.SEARCH_PARK, f"got {p.state.value}")

    for _ in range(40):
        step(p, c, lane_err=0.0, obstacle=False, blindspot=False)
        if p.state == Behavior.PULL_OVER:
            break
    check("S3.search_park: planner -> PULL_OVER",
          p.state == Behavior.PULL_OVER, f"got {p.state.value}")

    n = 0
    while p.state != Behavior.PARKED and n < 100:
        step(p, c, lane_err=0.0); n += 1
    check("S3.parked: reached PARKED",
          p.state == Behavior.PARKED, f"after {n} ticks")
    check("S3.parked: target_speed = 0, throttle = 0, brake = 1",
          SimBus.get_value('/target_speed') == 0.0
          and SimBus.get_value('/throttle_cmd') == 0.0
          and SimBus.get_value('/brake_cmd') == 1.0)


# ===========================================================================
# SCENARIO 4 -- emergency stop mid-drive
# ===========================================================================
def scenario_4():
    banner("S4 -- emergency stop mid-cruise")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    for _ in range(12): step(p, c, lane_err=10.0)
    check("S4.cruising: state LANE_FOLLOW", p.state == Behavior.LANE_FOLLOW)
    check("S4.cruising: throttle > 0",
          SimBus.get_value('/throttle_cmd') > 0)

    step(p, c, lane_err=10.0, estop=True)
    check("S4.estop: planner -> HOLD; drive_enabled False; throttle=0; brake=1",
          p.state == Behavior.HOLD
          and SimBus.get_value('/drive_enabled') is False
          and SimBus.get_value('/throttle_cmd') == 0.0
          and SimBus.get_value('/brake_cmd') == 1.0)
    step(p, c, lane_err=10.0, estop=False)
    check("S4.estop cleared: still HOLD, drive still disarmed",
          p.state == Behavior.HOLD
          and SimBus.get_value('/drive_enabled') is False)


# ===========================================================================
# SCENARIO 5 -- trigger link dies
# ===========================================================================
def scenario_5():
    banner("S5 -- trigger link dies after boot grace")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    p._boot_t = time.time() - 100.0
    p._link_ever_up = True
    feed(p, '/start_parking', String(data='go'))
    for _ in range(12):
        step(p, c, lane_err=0.0, link_alive=True)
    check("S5.cruising: state LANE_FOLLOW", p.state == Behavior.LANE_FOLLOW)
    step(p, c, link_alive=False)
    check("S5.link dead: HOLD + drive disarmed",
          p.state == Behavior.HOLD
          and SimBus.get_value('/drive_enabled') is False)


# ===========================================================================
# SCENARIO 6 -- lane lost
# ===========================================================================
def scenario_6():
    banner("S6 -- lane lost mid-cruise")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    for _ in range(20): step(p, c, lane_err=40.0)
    steer_before = SimBus.get_value('/steer_cmd')
    check("S6.setup: steer non-zero with err=40",
          abs(steer_before) > 0.1, f"got {steer_before:.3f}")
    for _ in range(5):
        step(p, c, lane_err=40.0, lane_found=False)
    steer_after = SimBus.get_value('/steer_cmd')
    check("S6.lane lost: steer magnitude shrinks",
          abs(steer_after) < abs(steer_before),
          f"before={steer_before:.3f} after={steer_after:.3f}")
    check("S6.lane lost: planner still LANE_FOLLOW",
          p.state == Behavior.LANE_FOLLOW)


# ===========================================================================
# SCENARIO 7 -- obstacle + blindspot blocked
# ===========================================================================
def scenario_7():
    banner("S7 -- obstacle + blindspot blocked (require_blindspot_clear=True)")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    p.require_blindspot_clear = True
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    cruise_through_dwell(p, c)

    step(p, c, obstacle=True, blindspot=True)
    check("S7.blocked: planner -> SLOW_FOR_OBSTACLE (no merge)",
          p.state == Behavior.SLOW_FOR_OBSTACLE,
          f"got {p.state.value}")
    check("S7.blocked: target_speed = SPEED_OBSTACLE_SLOW_KMH",
          SimBus.get_value('/target_speed') == SPEED_OBSTACLE_SLOW_KMH)
    check("S7.blocked: _merged_once still False",
          p._merged_once is False)
    # Settle both the state dwell and the transition cooldown before exit.
    for _ in range(max(p.min_dwell_in_state_frames,
                       p.transition_cooldown_frames) + 2):
        step(p, c, obstacle=False, blindspot=False)
    check("S7.clear: back to LANE_FOLLOW",
          p.state == Behavior.LANE_FOLLOW)


# ===========================================================================
# SCENARIO 8 -- post-merge obstacle muting
# ===========================================================================
def scenario_8():
    banner("S8 -- post-merge obstacle is muted (never re-merge)")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    cruise_through_dwell(p, c)
    step(p, c, obstacle=True)
    for _ in range(25): step(p, c, lane_err=80.0, obstacle=False)
    for _ in range(8): step(p, c, lane_err=10.0, obstacle=False)
    check("S8.first-merge: LANE_FOLLOW + _merged_once=True",
          p.state == Behavior.LANE_FOLLOW and p._merged_once is True)
    # Burn min_dwell again before the second obstacle.
    cruise_through_dwell(p, c)
    step(p, c, obstacle=True)
    check("S8.second-obstacle: remains LANE_FOLLOW with muting enabled",
          p.state == Behavior.LANE_FOLLOW,
          f"got {p.state.value}")


# ===========================================================================
# SCENARIO 9 -- perception fused-detector smoke test
# ===========================================================================
def scenario_9():
    banner("S9 -- fused HLS+Canny lane detector + confidence + debouncer")
    SimBus.reset()
    from jetson_driver.perception_node import (
        ClassicalLaneDetector, Debouncer,
    )
    import numpy as np
    import cv2

    det = ClassicalLaneDetector(publish_debug=False)

    def make_frame(left_x=180, right_x=460):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.line(img, (left_x, 280), (left_x,  470), (255, 255, 255), 6)
        cv2.line(img, (right_x, 280), (right_x, 470), (255, 255, 255), 6)
        return img

    img = make_frame(left_x=200, right_x=440)
    for _ in range(3):
        r = det.detect(img)
    check("S9.symmetric: found=True, |err| small",
          r['found'] and abs(r['error_px']) < 25.0,
          f"err={r['error_px']:.1f} conf={r['confidence']:.2f}")
    check("S9.symmetric: confidence > 0.5",
          r['confidence'] > 0.5,
          f"got conf={r['confidence']:.2f}")
    check("S9.symmetric: poly published",
          r['poly'] is not None)

    det = ClassicalLaneDetector(publish_debug=False)
    img = make_frame(left_x=280, right_x=520)
    for _ in range(3):
        r = det.detect(img)
    check("S9.shifted right: err > 0",
          r['error_px'] > 20, f"got err={r['error_px']:.1f}")

    det = ClassicalLaneDetector(publish_debug=False)
    img = make_frame(left_x=120, right_x=360)
    for _ in range(3):
        r = det.detect(img)
    check("S9.shifted left: err < 0",
          r['error_px'] < -20, f"got err={r['error_px']:.1f}")

    # Debouncer test (single-frame blips must NOT flip the latched state).
    db = Debouncer(true_frames=3, false_frames=5, initial=False)
    out1 = db.update(True)
    check("S9.debouncer: single True does NOT flip state",
          out1 is False)
    out2 = db.update(True); out3 = db.update(True)
    check("S9.debouncer: 3 consecutive Trues flips state True",
          out3 is True)
    out4 = db.update(False); out5 = db.update(False); out6 = db.update(False)
    check("S9.debouncer: 3 Falses NOT enough to flip back",
          out6 is True)
    db.update(False); db.update(False)
    check("S9.debouncer: 5 Falses flips back to False",
          db.state is False)


# ===========================================================================
# SCENARIO 10 -- pure-pursuit lateral controller
# ===========================================================================
def scenario_10():
    banner("S10 -- pure-pursuit direction sanity + fallback")
    SimBus.reset()
    c = ControlNode()
    c.lateral_kind = 'pure_pursuit'
    c._prev_steer = 0.0

    feed(c, '/lane_poly', Float32MultiArray(data=[0.0, 0.0, 400.0]))
    feed(c, '/lane_error', Float32(data=80.0))
    feed(c, '/lane_error_norm', Float32(data=0.25))
    feed(c, '/lane_confidence', Float32(data=1.0))
    feed(c, '/lane_found', Bool(data=True))
    feed(c, '/target_speed', Float32(data=8.0))
    feed(c, '/behavior', String(data='LANE_FOLLOW'))
    feed(c, '/drive_enabled', Bool(data=True))
    for _ in range(15):
        c._tick()
    st_right = SimBus.get_value('/steer_cmd')
    check("S10.lane-right: pure-pursuit steers right (+)",
          st_right > 0.05, f"got {st_right:.3f}")

    c._prev_steer = 0.0
    feed(c, '/lane_poly', Float32MultiArray(data=[0.0, 0.0, 240.0]))
    feed(c, '/lane_error', Float32(data=-80.0))
    feed(c, '/lane_error_norm', Float32(data=-0.25))
    for _ in range(15):
        c._tick()
    st_left = SimBus.get_value('/steer_cmd')
    check("S10.lane-left: pure-pursuit steers left (-)",
          st_left < -0.05, f"got {st_left:.3f}")

    c._pp._poly = None
    c._pp._poly_t = 0.0
    c._prev_steer = 0.0
    feed(c, '/lane_error_norm', Float32(data=0.5))
    for _ in range(10):
        c._tick()
    st_fb = SimBus.get_value('/steer_cmd')
    check("S10.fallback: no poly -> proportional fallback positive",
          st_fb > 0.05, f"got {st_fb:.3f}")


# ===========================================================================
# SCENARIO 11 (NEW) -- anti-mode-chatter
# Single-frame obstacle blips must NOT cause CHANGE_LANE_RIGHT.
# ===========================================================================
def scenario_11():
    banner("S11 -- anti-mode-chatter: obstacle blips must not transition")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    cruise_through_dwell(p, c)
    # Single-frame obstacle (then clear) -- planner uses already-debounced
    # /obstacle_ahead upstream, but the planner ALSO has min_dwell guard.
    # Here we exercise: even if upstream isn't debouncing, the planner
    # cooldown should prevent flip-flop.
    states_visited = []
    raw = [True, False, True, False, False, True, False]
    for r in raw:
        step(p, c, lane_err=0.0, obstacle=r)
        states_visited.append(p.state.value)
    transitions = sum(1 for a, b in zip(states_visited, states_visited[1:])
                      if a != b)
    check("S11.no-chatter: at most 1 state change across 7 obstacle blips",
          transitions <= 1,
          f"transitions={transitions}, path={states_visited}")


# ===========================================================================
# SCENARIO 12 (NEW) -- min_dwell before merge
# An obstacle on tick 2 (before min_dwell) must NOT transition.
# ===========================================================================
def scenario_12():
    banner("S12 -- obstacle BEFORE min_dwell elapses -> no transition")
    SimBus.reset()
    p = make_planner_with_trigger_gate()
    c = ControlNode()
    feed(p, '/start_parking', String(data='go'))
    step(p, c)
    step(p, c)
    check("S12.tick2: still LANE_FOLLOW",
          p.state == Behavior.LANE_FOLLOW)
    step(p, c, obstacle=True)
    check("S12.early-obstacle: still LANE_FOLLOW (min_dwell not elapsed)",
          p.state == Behavior.LANE_FOLLOW,
          f"state_frames={p._state_frames} min_dwell={p.min_dwell_in_state_frames}")
    # Now wait through min_dwell and re-assert obstacle.
    for _ in range(MIN_DWELL_IN_STATE_FRAMES + 1):
        step(p, c, obstacle=False)
    step(p, c, obstacle=True)
    check("S12.late-obstacle: NOW transitions to CHANGE_LANE_RIGHT",
          p.state == Behavior.CHANGE_LANE_RIGHT,
          f"got {p.state.value}")


# ===========================================================================
# SCENARIO 13 (NEW) -- motor_bridge no-motion fix
# Boot the bridge, publish first throttle/steer, verify TS frame is sent
# instead of being trapped on B\n.
# ===========================================================================
def scenario_13():
    banner("S13 -- motor_bridge boot watchdog fix (no more B\\n trap)")
    SimBus.reset()
    # Use simulate=True so the bridge logs frames instead of opening serial.
    mb = MotorBridgeNode()
    mb._params['simulate'].value = True
    mb.simulate = True
    # On boot, the bridge has t_last_* set to a stale-but-non-zero sentinel.
    # First TX tick BEFORE any messages: should send B\n (watchdog).
    mb._tx_tick()
    first_frame = mb._last_frame_logged
    check("S13.boot: first frame after boot is brake (watchdog stale)",
          first_frame == "B", f"got {first_frame!r}")

    # Publish throttle + steer + brake.
    feed(mb, '/throttle_cmd', Float32(data=-0.95))
    feed(mb, '/steer_cmd',    Float32(data=0.0))
    feed(mb, '/brake_cmd',    Float32(data=0.0))
    feed(mb, '/behavior',     String(data='LANE_FOLLOW'))
    # Next tick should now emit a TS frame, NOT B.
    mb._tx_tick()
    check("S13.first-cmds: bridge emits TS frame (no longer trapped)",
          mb._last_frame_logged == "TS",
          f"got {mb._last_frame_logged!r}")

    # PARKED override.
    feed(mb, '/behavior', String(data='PARKED'))
    mb._tx_tick()
    check("S13.parked: bridge emits P frame on PARKED entry",
          mb._last_frame_logged == "P",
          f"got {mb._last_frame_logged!r}")
    mb._tx_tick()
    check("S13.parked.hold: bridge then emits steady B frames",
          mb._last_frame_logged == "B",
          f"got {mb._last_frame_logged!r}")


# ===========================================================================
# SCENARIO 14 (NEW) -- throttle floor at output, with invert_throttle=True
# This is exactly the prototype's wiring: invert_throttle=True, min_pwm=0.95
# Final output should be a magnitude >= 0.95 negative.
# ===========================================================================
def scenario_14():
    banner("S14 -- throttle floor honoured at output with invert_throttle=True")
    SimBus.reset()
    c = ControlNode()
    c.invert_throttle = True
    c.throttle_min_pwm = 0.95
    c.throttle_full_kmh = 10.0
    c.drive_kick_frames = 0   # disable kick BEFORE drive_enabled fires
    feed(c, '/lane_error', Float32(data=0.0))
    feed(c, '/lane_error_norm', Float32(data=0.0))
    feed(c, '/lane_confidence', Float32(data=1.0))
    feed(c, '/lane_found', Bool(data=True))
    feed(c, '/target_speed', Float32(data=8.0))   # 8/10 = 0.8 raw
    feed(c, '/behavior', String(data='LANE_FOLLOW'))
    feed(c, '/drive_enabled', Bool(data=True))
    c._tick()
    th = SimBus.get_value('/throttle_cmd')
    # raw=0.8 -> floor 0.95 -> inverted -> -0.95
    check("S14.floor: output throttle is exactly -0.95 (floor applied)",
          abs(th - (-0.95)) < 1e-6, f"got {th:.4f}")

    # Lower target -- floor should still apply.  Need 2 ticks: first tick
    # hits the decel-rate branch which zeros throttle by design; second
    # tick settles to a steady raw=0.4, which the floor pushes to 0.95.
    feed(c, '/target_speed', Float32(data=4.0))
    c._tick()  # first tick: decel-rate branch, throttle = 0
    feed(c, '/target_speed', Float32(data=4.0))
    c._tick()  # second tick: steady state, floor applies
    th = SimBus.get_value('/throttle_cmd')
    check("S14.low-target: floor still honoured at -0.95 after settle",
          abs(th - (-0.95)) < 1e-6, f"got {th:.4f}")

    feed(c, '/target_speed', Float32(data=0.0))
    c._tick()
    check("S14.zero-target: throttle=0, brake=1",
          SimBus.get_value('/throttle_cmd') == 0.0
          and SimBus.get_value('/brake_cmd') == 1.0)


# ===========================================================================
# SCENARIO 15 (NEW) -- confidence-weighted steer attenuation
# At conf=0.5, steer authority is half of what it is at conf=1.0.
# ===========================================================================
def scenario_15():
    banner("S15 -- confidence-weighted steer attenuation")
    SimBus.reset()
    # High-confidence run.
    c1 = ControlNode()
    feed(c1, '/lane_error', Float32(data=40.0))
    feed(c1, '/lane_error_norm', Float32(data=0.125))
    feed(c1, '/lane_confidence', Float32(data=1.0))
    feed(c1, '/lane_found', Bool(data=True))
    feed(c1, '/target_speed', Float32(data=8.0))
    feed(c1, '/behavior', String(data='LANE_FOLLOW'))
    feed(c1, '/drive_enabled', Bool(data=True))
    for _ in range(20): c1._tick()
    steer_hi = SimBus.get_value('/steer_cmd')

    # Low-confidence run.
    SimBus.reset()
    c2 = ControlNode()
    feed(c2, '/lane_error', Float32(data=40.0))
    feed(c2, '/lane_error_norm', Float32(data=0.125))
    feed(c2, '/lane_confidence', Float32(data=0.4))  # at the floor
    feed(c2, '/lane_found', Bool(data=True))
    feed(c2, '/target_speed', Float32(data=8.0))
    feed(c2, '/behavior', String(data='LANE_FOLLOW'))
    feed(c2, '/drive_enabled', Bool(data=True))
    for _ in range(20): c2._tick()
    steer_lo = SimBus.get_value('/steer_cmd')

    check("S15: high-conf steer magnitude > low-conf steer magnitude",
          abs(steer_hi) > abs(steer_lo),
          f"hi={steer_hi:.3f}, lo={steer_lo:.3f}")


# ===========================================================================
# Run all
# ===========================================================================
def main():
    scenarios = [
        scenario_1, scenario_2, scenario_3, scenario_4, scenario_5,
        scenario_6, scenario_7, scenario_8, scenario_9, scenario_10,
        scenario_11, scenario_12, scenario_13, scenario_14, scenario_15,
    ]
    for s in scenarios:
        try:
            s()
        except Exception as e:
            import traceback
            print(f"  [CRASH] {s.__name__}: {e}")
            traceback.print_exc()
            FAIL.append((s.__name__, f"CRASH: {e}"))
    print()
    print("=" * 70)
    print(f"  TOTAL: {len(PASS)} PASS  /  {len(FAIL)} FAIL")
    print("=" * 70)
    if FAIL:
        for name, detail in FAIL:
            print(f"  FAIL: {name}  {detail}")
        sys.exit(1)
    sys.exit(0)


if __name__ == '__main__':
    main()
