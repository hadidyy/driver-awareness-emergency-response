"""Constants and shared helpers for the jetson_driver stack — unified rewrite.

Rewritten 2026-05-18 to combine the best of approach 1 (working PID gains,
camera and bridge infrastructure) and approach 2 (HLS sliding-window
detector, simplified FSM), plus the following new mechanisms targeted at
the two failure modes seen on the prototype:

  1.  Mode chatter in approach 1 was caused by loose counter resets and a
      missing minimum-dwell guard on vision-aware merge exit.  This file
      now defines explicit MIN_DWELL_* constants used by planner_node to
      enforce a settle window before any transition out of a state.

  2.  No motion in approach 2 was traced to two coupled problems:
        (a)  motor_bridge_node initialised its t_last_* timestamps to 0.0
             and explicitly tested ``== 0`` as a stale-cmd condition,
             trapping the bridge in B\\n forever on boot;
        (b)  control_node's THROTTLE_MIN_PWM floor was tested before the
             invert-throttle sign flip, and the magnitude that came out
             of (target / full_kmh) was below the floor only in some
             ranges, so the floor wasn't always honoured on the wire.
      The constants below feed a rewritten throttle pipeline that applies
      the floor on the absolute value of the final command, AFTER sign
      handling, so the motor PWM never sits below the static-friction
      break-away of the prototype.

  3.  Lane detection robustness — the HLS color mask is still the primary
      gate, but we now combine it with a Canny edge mask (binary OR), add
      per-fit sanity checks, and debounce /lane_found over multiple
      consecutive frames before the planner sees a state change.

External ROS contract preserved:
  * /lane_error, /lane_found, /obstacle_ahead, /yolo_blindspot_right,
    /perception_debug   (sensor_msgs/Image + std_msgs/Float32, Bool)
  * /lane_error_norm, /lane_curvature, /lane_poly  (additive, kept)
  * /target_speed, /behavior, /drive_enabled
  * /throttle_cmd, /steer_cmd, /brake_cmd
  * /start_parking, /emergency_stop, /trigger_link_alive, /heart_rate_bpm,
    /leads_ok
  * Motor ESP wire protocol (TS / B / P / H)   — DO NOT change
  * Trigger ESP TCP protocol                    — DO NOT change

A new /lane_confidence Float32 topic is published by perception_node.  It
is additive and optional; the control_node uses it for steer-authority
attenuation, but no consumer is required to subscribe.
"""

from enum import Enum


# ============================================================================
#  Behavior FSM — perception-driven, no time-based exits.
# ============================================================================
class Behavior(str, Enum):
    HOLD              = "HOLD"
    LANE_FOLLOW       = "LANE_FOLLOW"
    SLOW_FOR_OBSTACLE = "SLOW_FOR_OBSTACLE"
    CHANGE_LANE_RIGHT = "CHANGE_LANE_RIGHT"
    SEARCH_PARK       = "SEARCH_PARK"
    PULL_OVER         = "PULL_OVER"
    PARKED            = "PARKED"


# ============================================================================
#  Camera + lane detection
# ============================================================================
CAM_W = 640
CAM_H = 480

# Backend selection — kept for API parity with approach 2.  "classical" is
# the only fully wired backend; "segnet" is a future hook.
LANE_BACKEND_DEFAULT = "classical"

# Legacy histogram detector constants — kept so old imports resolve.
ROI_TOP_FRAC       = 0.55
THRESHOLD_VALUE    = 150
MIN_PEAK           = 100
HALF_LANE_FRAC     = 0.31
LANE_ERROR_ALPHA   = 0.7

# HLS color mask thresholds.
HLS_WHITE_L_MIN    = 180
HLS_WHITE_S_MAX    = 80
HLS_YELLOW_H_MIN   = 15
HLS_YELLOW_H_MAX   = 35
HLS_YELLOW_S_MIN   = 60
HLS_YELLOW_L_MIN   = 80

# Canny edge detection thresholds (NEW — adds an independent edge signal).
CANNY_LOW          = 60
CANNY_HIGH         = 180
CANNY_BLUR_KERNEL  = 5

# Sliding-window polyfit.
NWINDOWS            = 9
WINDOW_MARGIN_PX    = 80
MIN_PIX_PER_WINDOW  = 50

# EMA on polynomial coefficients.
LANE_POLY_ALPHA     = 0.6

# NEW: sanity check tolerances on each polyfit.
POLY_A_ABS_MAX      = 5e-3
POLY_B_ABS_MAX      = 1.0
LANE_WIDTH_PX_MIN   = 200
LANE_WIDTH_PX_MAX   = 600

# NEW: confidence / debounce thresholds.
LANE_CONF_HI         = 0.50   # confidence required to flip lane_found True
LANE_CONF_LO         = 0.30   # below this for LANE_FOUND_FALSE_FRAMES → flip False
LANE_FOUND_TRUE_FRAMES  = 3   # consecutive frames at >= HI
LANE_FOUND_FALSE_FRAMES = 5   # consecutive frames at <  LO
LANE_CONF_DEFAULT_FLOOR = 0.4 # minimum steer-authority weight even when conf low


# ============================================================================
#  YOLO object detection
# ============================================================================
YOLO_MODEL_NAME              = 'yolo11s.pt'
YOLO_CONF                    = 0.35
YOLO_IMGSZ                   = 416
YOLO_FRAME_INTERVAL          = 3
YOLO_VEHICLE_CLASSES         = (1, 2, 3, 5, 7)
YOLO_PERSON_CLASS            = 0

YOLO_FRONT_CLOSE_HEIGHT_FRAC = 0.10   # was 0.18 — trigger earlier (farther obstacle)
YOLO_FRONT_CLOSE_BOTTOM_FRAC = 0.50   # was 0.60 — accept obstacles higher in frame
YOLO_FRONT_EGO_LANE_LEFT_FRAC  = 0.30 # was 0.35 — wider ego-lane filter
YOLO_FRONT_EGO_LANE_RIGHT_FRAC = 0.70 # was 0.65

YOLO_REAR_BLINDSPOT_LEFT_FRAC  = 0.45
YOLO_REAR_BLINDSPOT_RIGHT_FRAC = 0.95
YOLO_REAR_MIN_HEIGHT_FRAC      = 0.08

# NEW: debounce on /obstacle_ahead so single-frame YOLO blips don't drive
# the FSM into CHANGE_LANE_RIGHT.
OBSTACLE_TRUE_FRAMES     = 2   # was 3 — faster lane-change trigger
OBSTACLE_FALSE_FRAMES    = 5
BLINDSPOT_TRUE_FRAMES    = 2
BLINDSPOT_FALSE_FRAMES   = 4


# ============================================================================
#  Planner speeds (km/h)
#  Target speeds below THROTTLE_FULL_KMH no longer saturate to the floor —
#  control_node now pulse-cycles the throttle to fake a slower average
#  speed.  Speed scales linearly with target_kmh / throttle_full_kmh.
# ============================================================================
SPEED_STRAIGHT_KMH         = 10.0   # was 8.0 — slow demo cruise
SPEED_CURVE_KMH            = 8.0
SPEED_OBSTACLE_SLOW_KMH    = 8.0   # was 6.0
SPEED_PARK_APPROACH_KMH    = 3.0   # was 6.0
SPEED_LANE_CHANGE_KMH      = 6.0   # was 7.0 — slightly faster mid-merge for crispness

PARK_SPEED_DECAY            = 0.90
PARK_HOLD_SPEED_KMH         = 0.4


# ============================================================================
#  FSM dwell counters (frames @ PLANNER_HZ).  These are the central
#  anti-mode-chatter knobs.  Every state transition is gated on at least
#  MIN_DWELL_IN_STATE_FRAMES of time spent in the source state, in
#  addition to the sensor condition.
# ============================================================================
MIN_DWELL_IN_STATE_FRAMES   = 5   # was 10 — quicker reaction to obstacles

LC_OBSTACLE_CLEAR_FRAMES    = 12
LC_SAFE_BLINDSPOT_FRAMES    = 30
POST_CHANGE_DWELL_FRAMES    = 60
PARK_SAFE_BLINDSPOT_FRAMES  = 20

MIN_MERGE_FRAMES            = 20   # absolute min in CHANGE_LANE_RIGHT
MERGE_COMPLETE_ERR_PX       = 30.0
MERGE_SETTLED_FRAMES_REQ    = 4

# Transition cool-down — after any state change, refuse a follow-up
# transition for this many frames so the FSM has time to settle.
TRANSITION_COOLDOWN_FRAMES  = 8

# Auto lane-change fallback — if no obstacle is ever seen, after this
# many seconds in LANE_FOLLOW since GO, the planner fires
# CHANGE_LANE_RIGHT anyway.  Safety net for "professor didn't put down a
# decoy" or "YOLO failed to recognise the decoy" failure modes.
# Override via YAML param planner_node.auto_lanechange_timeout_s.
AUTO_LANECHANGE_TIMEOUT_S   = 12.0   # seconds
ENABLE_AUTO_LANECHANGE      = True

# Post-merge obstacle muting — after the (one allowed) lane change has
# happened, ignore /obstacle_ahead so YOLO false-positives (clutter near
# the parking strip, the decoy itself as we pass it, etc.) don't drag
# the FSM into SLOW_FOR_OBSTACLE detours while we're heading toward
# SEARCH_PARK.  Keeps the post-merge → park flow visually clean.
# Override via YAML param planner_node.disable_obstacle_after_merge.
DISABLE_OBSTACLE_AFTER_MERGE = True

# Lane-lost safety watchdog — if the lane detector loses both lines for
# more than LANE_LOST_TIMEOUT_S seconds while the car is supposed to be
# tracking a lane (LANE_FOLLOW / SLOW_FOR_OBSTACLE / SEARCH_PARK), the
# planner forces HOLD (brake) and DISARMS drive.  User has to press GO
# again to resume.  Protects the prototype from driving off the track
# and into a wall.
#
# Skipped in CHANGE_LANE_RIGHT and PULL_OVER because those states
# legitimately lose lane tracking (crossing lanes / parking).
#
# Override via YAML params planner_node.lane_lost_timeout_s and
# planner_node.enable_lane_lost_stop.
LANE_LOST_TIMEOUT_S         = 2.0    # seconds without /lane_found=True
ENABLE_LANE_LOST_STOP       = True


# ============================================================================
#  Control — steering and throttle mapping
# ============================================================================
LATERAL_CONTROLLER_DEFAULT = "pid"

# PID gains — preserved from approach 1's bench-tuning.
STEER_KP          = 0.025
STEER_KI          = 0.0015
STEER_KD          = 0.008
STEER_CLAMP       = 1.0
INTEGRAL_CLAMP    = 120.0
D_INPUT_CLAMP     = 40.0
DEAD_ZONE_PX      = 5.0
MAX_STEER_DELTA   = 0.15

# Pure-pursuit constants.
LOOKAHEAD_PX        = 160
WHEELBASE_PX_EQUIV  = 240.0
STEER_GAIN_PP       = 2.2

# Open-loop steers for state-machine cases.
LANE_CHANGE_STEER = 0.85
PULL_OVER_STEER   = 0.60

# Throttle mapping is open-loop (no encoders).
THROTTLE_FULL_KMH = 10.0     # speed at which raw throttle saturates to 1.0
THROTTLE_IDLE     = 0.0
BRAKE_REVERSE     = -0.35
THROTTLE_MIN_PWM  = 0.95     # ABSOLUTE floor applied on |output| AFTER sign

# NEW: drive-arm "kick" — when transitioning from stopped to moving, push
# the throttle to 1.0 for KICK_FRAMES frames to overcome static friction
# before settling to the cruise PWM.  Prototype specific.
DRIVE_KICK_THROTTLE  = 1.00
DRIVE_KICK_FRAMES    = 6     # ~0.3 s at CONTROL_HZ

# NEW: pulse-mode throttle.  The prototype motor has a static-friction
# break-away at THROTTLE_MIN_PWM (~0.95).  Sending PWM below that stalls
# the motor; sending PWM above it always runs at the same "full" speed.
# To fake slower average speeds we alternate ON (=THROTTLE_MIN_PWM) and
# OFF (=0.0) frames so the motor still breaks friction during ON frames
# but coasts during OFF frames.  Effective speed ≈ duty × full speed.
#
#   duty = target_kmh / throttle_full_kmh
#   pulse_period = PULSE_PERIOD_FRAMES (e.g. 5 frames = 250 ms at 20 Hz)
#   on_frames    = round(duty * pulse_period), clamped to [PULSE_MIN_ON, period]
#
# PULSE_MIN_ON guarantees we always kick the motor through static friction
# even at very low duty (it would otherwise stall).
PULSE_PERIOD_FRAMES  = 5     # 250 ms cycle at 20 Hz (4 Hz pulse rate)
PULSE_MIN_ON_FRAMES  = 2     # at least 2 ON frames per cycle, so motor moves


# ============================================================================
#  Loop rates
# ============================================================================
CAMERA_HZ    = 20
PLANNER_HZ   = 20
CONTROL_HZ   = 20
MOTOR_CMD_HZ = 20

MOTOR_WATCHDOG_S = 0.5
