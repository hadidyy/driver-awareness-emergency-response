"""
Parametric 3D-printable Ackermann-steering car chassis
=======================================================

Designed to carry:
  - Jetson Nano (B01 carrier, 100 x 80 mm)
  - ESP32 DevKit (30-pin, 55 x 28 mm)
  - 2x USB webcams (26 x 26 mm ELP-style, front + rear)
  - NEO-6M GPS module (36 x 24 mm) with external antenna tower
  - SIM800L GSM module (40 x 28 mm)
  - 2S LiPo battery pack or 2x 18650 holder (~75 x 40 x 20 mm)
  - 2x TT plastic gearmotors (70 x 37 x 22 mm) at rear wheels
  - SG90 servo for front Ackermann steering

Drivetrain:
  Rear = 2x TT motors driving rear wheels (one per wheel, skid-steer compatible).
  Front = 2x steering knuckles pivoting on M3 king-pins, linked by a tie rod,
          actuated by an SG90 servo via a pushrod.

Author: Designed for Seif's driver-awareness project.
License: CC0 / public domain — tweak freely.

Tuning:
  Every dimension below is in millimeters and parametric. Change the values in
  the ### PARAMETERS ### block and re-run to regenerate STL + STEP files.
"""

import cadquery as cq
from pathlib import Path

# =============================================================================
#                             ### PARAMETERS ###
# =============================================================================

# --- Print / tolerance settings -------------------------------------------
WALL           = 2.4   # nominal wall thickness
PLATE          = 3.0   # chassis / deck plate thickness
CLEARANCE      = 0.25  # slip-fit clearance between mating printed parts
M3_CLEAR       = 3.4   # through-hole for M3 screw
M3_TAP         = 2.8   # self-tap hole for M3 into plastic
M2_CLEAR       = 2.3   # through-hole for M2 screw (servo horn, SIM800L)
M25_CLEAR      = 2.8   # through-hole for M2.5 screw (ESP32 mounts)

# --- Overall vehicle size -------------------------------------------------
WHEELBASE      = 170.0   # front axle to rear axle
TRACK          = 140.0   # wheel-to-wheel, outside of hubs
CHASSIS_LEN    = 210.0   # total chassis length
CHASSIS_W      = 130.0   # chassis width (narrower than track)
DECK_HEIGHT    = 75.0    # top deck above lower chassis (clears battery + servo)

# --- TT gearmotor dimensions (standard yellow motor) ---------------------
TT_BODY_L      = 70.0    # length from gearbox front face to motor back
TT_BODY_W      = 22.5    # motor body width
TT_BODY_H      = 37.0    # motor body height (incl. gearbox)
TT_SHAFT_D     = 5.5     # D-shaft outer diameter (nominal)
TT_MOUNT_DX    = 9.5     # mount hole offset from motor face along X
TT_MOUNT_DY    = 17.5    # mount hole offset from motor centerline along Y

# --- Wheel (printed) ------------------------------------------------------
WHEEL_OD       = 65.0
WHEEL_WIDTH    = 26.0
WHEEL_HUB_D    = 18.0    # hub boss outer diameter
TIRE_GROOVE_D  = 2.5     # cross-section depth for rubber O-ring tire groove

# --- Steering geometry ----------------------------------------------------
KINGPIN_D      = 3.0          # M3 king-pin bolt
KNUCKLE_H      = 22.0         # height of the knuckle body
STEER_ARM_LEN  = 22.0         # lever arm from king-pin to tie-rod pin
TIE_ROD_LEN    = TRACK - 2 * STEER_ARM_LEN - 10  # rod between knuckles

# --- Jetson Nano B01 ------------------------------------------------------
JETSON_L       = 100.0
JETSON_W       = 80.0
JETSON_H_STACK = 32.0
JETSON_HOLE_DX = 86.0    # hole center-to-center spacing along long axis
JETSON_HOLE_DY = 58.0    # hole center-to-center spacing along short axis

# --- ESP32 DevKit (30-pin, common layout) ---------------------------------
ESP_L          = 55.0
ESP_W          = 28.0
ESP_HOLE_DX    = 48.0
ESP_HOLE_DY    = 22.0

# --- NEO-6M GPS module ----------------------------------------------------
GPS_L          = 36.0
GPS_W          = 24.0
GPS_HOLE_DX    = 30.0
GPS_HOLE_DY    = 17.5

# --- SIM800L GSM module ---------------------------------------------------
GSM_L          = 40.0
GSM_W          = 28.0
GSM_HOLE_DX    = 36.0
GSM_HOLE_DY    = 24.0

# --- Battery tray (2S/3S pack or 2x 18650 holder) -------------------------
BATT_L         = 80.0
BATT_W         = 45.0
BATT_H         = 22.0

# --- USB webcam (small ELP-style square module) ---------------------------
CAM_L          = 28.0
CAM_W          = 28.0
CAM_HOLE_DX    = 22.0
CAM_HOLE_DY    = 22.0

# --- SG90 servo -----------------------------------------------------------
SERVO_BODY_L   = 23.0
SERVO_BODY_W   = 12.5
SERVO_BODY_H   = 22.7
SERVO_FLANGE_L = 32.5          # distance between flange hole centers
SERVO_HOLE_D   = 2.0           # screw hole on flange

# =============================================================================
#                             Helper primitives
# =============================================================================

def standoff(height, hole=M3_TAP, od=7.0):
    """A simple cylindrical standoff with a tapping hole down the center."""
    return (
        cq.Workplane("XY")
        .circle(od / 2).extrude(height)
        .faces(">Z").workplane()
        .hole(hole, depth=height - 1.5)
    )


def countersunk_hole(wp, d=M3_CLEAR, cs_d=6.0, cs_depth=1.6):
    """Simple countersink for M3 on a workplane face."""
    return wp.cskHole(d, cs_d, 82, depth=None)


def _rect_hole_grid(wp, dx, dy, d):
    """Drill 4 holes forming a rectangle centered at the current WP origin."""
    pts = [(dx / 2, dy / 2), (-dx / 2, dy / 2),
           (dx / 2, -dy / 2), (-dx / 2, -dy / 2)]
    return wp.pushPoints(pts).hole(d)


def _rect_slot_grid(wp, dx, dy, d, depth):
    pts = [(dx / 2, dy / 2), (-dx / 2, dy / 2),
           (dx / 2, -dy / 2), (-dx / 2, -dy / 2)]
    return wp.pushPoints(pts).circle(d / 2).cutBlind(-depth)


def _mounting_posts(dx, dy, height, d_tap=M3_TAP, od=6.5):
    """Four cylindrical posts at the corners of a dx-by-dy rectangle."""
    base = cq.Workplane("XY")
    for (px, py) in [(dx / 2, dy / 2), (-dx / 2, dy / 2),
                     (dx / 2, -dy / 2), (-dx / 2, -dy / 2)]:
        post = (cq.Workplane("XY").center(px, py)
                .circle(od / 2).extrude(height)
                .faces(">Z").workplane().hole(d_tap, depth=height - 1.5))
        base = base.union(post)
    return base


# =============================================================================
#                             Parts
# =============================================================================

def chassis_base():
    """
    Lower chassis plate — the main structural floor.
    X axis: vehicle length (positive = forward).
    Y axis: vehicle width.
    Z axis: up.  Origin at center of wheelbase, top face of plate at Z=0.

    Built as separate solids then unioned for numerical robustness.
    """
    # ---- 1. Flat plate with all cutouts ---------------------------------
    plate_raw = (
        cq.Workplane("XY")
        .rect(CHASSIS_LEN, CHASSIS_W).extrude(-PLATE)
        .edges("|Z").fillet(8.0)
    )
    # All through-cutouts at once via pushPoints
    slot_pts = [(-50, 0), (0, 0), (50, 0)]
    plate_raw = (plate_raw.faces(">Z").workplane()
                          .pushPoints(slot_pts)
                          .slot2D(40, 20, 0).cutThruAll())

    # ---- 2. Rear motor bulkhead -----------------------------------------
    rear_x = -WHEELBASE / 2
    bulkhead_h = TT_BODY_H + 4
    bulkhead_t = 3.0
    rear_bulk = (
        cq.Workplane("XY").center(rear_x, 0)
        .rect(bulkhead_t, CHASSIS_W - 10).extrude(bulkhead_h)
    )
    # Motor shaft + screw holes (drilled normal to YZ plane, running along X)
    for sy_sign in (-1, 1):
        yc = sy_sign * (TRACK / 2 - 18)
        shaft = (cq.Workplane("YZ").workplane(offset=rear_x)
                 .center(yc, TT_BODY_H / 2)
                 .circle(TT_SHAFT_D / 2 + 1.5)
                 .extrude(bulkhead_t + 4, both=True))
        rear_bulk = rear_bulk.cut(shaft)
        for dz in (-TT_MOUNT_DY / 2, TT_MOUNT_DY / 2):
            sc = (cq.Workplane("YZ").workplane(offset=rear_x)
                  .center(yc + TT_BODY_W / 2 - TT_MOUNT_DX, TT_BODY_H / 2 + dz)
                  .circle(M3_CLEAR / 2)
                  .extrude(bulkhead_t + 4, both=True))
            rear_bulk = rear_bulk.cut(sc)

    # ---- 3. Front steering bulkhead + king-pin bosses -------------------
    front_x = WHEELBASE / 2
    sb_h = KNUCKLE_H + 8
    sb_t = 4.0
    front_bulk = (
        cq.Workplane("XY").center(front_x, 0)
        .rect(sb_t, CHASSIS_W - 10).extrude(sb_h)
    )
    for sy_sign in (-1, 1):
        yc = sy_sign * (TRACK / 2 - 10)
        boss = (cq.Workplane("XY").center(front_x, yc)
                .circle(6.0).extrude(sb_h))
        kingpin = (cq.Workplane("XY").center(front_x, yc)
                   .circle(M3_TAP / 2).extrude(sb_h - 2)
                   .translate((0, 0, 2)))
        front_bulk = front_bulk.union(boss).cut(kingpin)

    # ---- 4. Servo pocket block ------------------------------------------
    servo_x = front_x - 30
    servo_block = (
        cq.Workplane("XY").center(servo_x, 0)
        .rect(SERVO_BODY_L + 2 * WALL, SERVO_BODY_W + 2 * WALL)
        .extrude(SERVO_BODY_H)
    )
    servo_hollow = (
        cq.Workplane("XY").center(servo_x, 0)
        .rect(SERVO_BODY_L, SERVO_BODY_W).extrude(SERVO_BODY_H + 1)
    )
    servo_block = servo_block.cut(servo_hollow)
    for dx in (-SERVO_FLANGE_L / 2, SERVO_FLANGE_L / 2):
        h = (cq.Workplane("XY").center(servo_x + dx, 0)
             .circle(SERVO_HOLE_D / 2).extrude(SERVO_BODY_H + PLATE + 4)
             .translate((0, 0, -PLATE - 2)))
        servo_block = servo_block.cut(h)

    # ---- 5. Standoff bosses for upper deck (4 corners) ------------------
    deck_dx = CHASSIS_LEN - 40
    deck_dy = CHASSIS_W - 20
    standoffs = cq.Workplane("XY")
    for (sx, sy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        b = (cq.Workplane("XY").center(sx * deck_dx / 2, sy * deck_dy / 2)
             .circle(5.0).extrude(8.0))
        hole = (cq.Workplane("XY").center(sx * deck_dx / 2, sy * deck_dy / 2)
                .circle(M3_TAP / 2).extrude(6.0).translate((0, 0, 1)))
        b = b.cut(hole)
        if len(standoffs.vals()) == 0:
            standoffs = b
        else:
            standoffs = standoffs.union(b)

    # ---- 6. Battery tray side walls -------------------------------------
    batt_x = -20
    batt_walls = cq.Workplane("XY")
    for sy_sign in (-1, 1):
        w = (cq.Workplane("XY")
             .center(batt_x, sy_sign * (BATT_W / 2 + 1.2))
             .rect(BATT_L, 2.4).extrude(BATT_H))
        if len(batt_walls.vals()) == 0:
            batt_walls = w
        else:
            batt_walls = batt_walls.union(w)

    # ---- 7. Union everything into final chassis -------------------------
    chassis = (plate_raw
               .union(rear_bulk)
               .union(front_bulk)
               .union(servo_block)
               .union(standoffs)
               .union(batt_walls))
    return chassis


def knuckle(side="left"):
    """
    Front steering knuckle. Pivots on an M3 king-pin inserted from below.
    The stub axle for the wheel is an M3 bolt through the horizontal boss.
    A steering arm sticks inward with a pin hole for the tie rod.
    `side` is 'left' (y>0) or 'right' (y<0) and controls the arm direction.
    """
    sign = 1 if side == "left" else -1

    # Main vertical body that wraps the king-pin
    body = (
        cq.Workplane("XY")
        .circle(6.0).extrude(KNUCKLE_H)
        .faces(">Z").workplane()
        .hole(KINGPIN_D + 0.3)   # loose slip-fit on the kingpin bolt
    )

    # Horizontal stub-axle arm sticking OUTWARD (toward the wheel)
    axle_arm = (
        cq.Workplane("XY")
        .center(0, sign * 8)
        .box(14, 16, 10, centered=(True, True, False))
        .translate((0, 0, KNUCKLE_H / 2 - 5))
    )
    # Bore the stub-axle hole horizontally through the arm
    axle_hole = (
        cq.Workplane("YZ")
        .center(sign * 8, KNUCKLE_H / 2)
        .circle(M3_CLEAR / 2 + 0.1)
        .extrude(30, both=True)
    )
    body = body.union(axle_arm).cut(axle_hole)

    # Steering arm sticking INWARD (toward the tie rod)
    steer_arm = (
        cq.Workplane("XY")
        .center(0, -sign * STEER_ARM_LEN / 2)
        .box(8, STEER_ARM_LEN, 4, centered=(True, True, False))
        .translate((0, 0, KNUCKLE_H - 4))
    )
    # Tie-rod pin hole (M3 through)
    pin_hole = (
        cq.Workplane("XY")
        .center(0, -sign * STEER_ARM_LEN)
        .circle(M3_CLEAR / 2)
        .extrude(10)
        .translate((0, 0, KNUCKLE_H - 6))
    )
    body = body.union(steer_arm).cut(pin_hole)
    return body


def tie_rod():
    """Flat rod that couples both steering knuckles."""
    rod = (
        cq.Workplane("XY")
        .box(TIE_ROD_LEN, 6.0, 3.0)
        .faces(">Z").workplane()
        .pushPoints([(-TIE_ROD_LEN / 2 + 3, 0), (TIE_ROD_LEN / 2 - 3, 0)])
        .hole(M3_CLEAR)
    )
    return rod


def wheel():
    """Printable wheel with a rubber O-ring tire groove on the outer face."""
    w = (
        cq.Workplane("XY")
        .circle(WHEEL_OD / 2).extrude(WHEEL_WIDTH)
    )
    # Spoke pockets (visual + weight reduction)
    for ang in range(0, 360, 60):
        pocket = (
            cq.Workplane("XY")
            .transformed(rotate=(0, 0, ang))
            .center(WHEEL_OD / 4, 0)
            .circle(8).extrude(WHEEL_WIDTH - 4)
            .translate((0, 0, 2))
        )
        w = w.cut(pocket)
    # Tire groove (O-ring profile)
    groove = (
        cq.Workplane("XZ")
        .center(0, WHEEL_WIDTH / 2)
        .moveTo(WHEEL_OD / 2, -TIRE_GROOVE_D)
        .threePointArc((WHEEL_OD / 2 + TIRE_GROOVE_D * 1.2, 0),
                       (WHEEL_OD / 2, TIRE_GROOVE_D))
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    w = w.cut(groove)
    # Center hub: D-shaft hole for TT motor (or M3 for front wheels)
    w = (w.faces("<Z").workplane()
           .circle(TT_SHAFT_D / 2 + 0.15).cutThruAll())
    # Flat on D-shaft (cut the side of the hub hole flat)
    flat = (
        cq.Workplane("XY")
        .center(TT_SHAFT_D / 2 - 0.7, 0)
        .rect(2.0, TT_SHAFT_D + 1)
        .extrude(WHEEL_WIDTH)
    )
    w = w.cut(flat)
    return w


def upper_deck():
    """Upper electronics deck, bolts onto the 4 standoff bosses on chassis.

    All holes collected into one pushPoints call for numerical robustness.
    Layout (looking down from above, +X = forward):
       +X  forward
       [ front cams ] [ GPS ]         [ GSM ]
       [ Jetson Nano centered               ]
       [ ESP32 rear                         ]
       -X  rear
    """
    deck_len = CHASSIS_LEN - 30
    deck_w   = CHASSIS_W - 5
    deck_dx  = CHASSIS_LEN - 40
    deck_dy  = CHASSIS_W - 20

    # Assemble absolute-coord hole lists by group, each with its own diameter
    corner_pts = [( deck_dx / 2,  deck_dy / 2),
                  (-deck_dx / 2,  deck_dy / 2),
                  ( deck_dx / 2, -deck_dy / 2),
                  (-deck_dx / 2, -deck_dy / 2)]

    jetson_pts = [( JETSON_HOLE_DX / 2,  JETSON_HOLE_DY / 2),
                  (-JETSON_HOLE_DX / 2,  JETSON_HOLE_DY / 2),
                  ( JETSON_HOLE_DX / 2, -JETSON_HOLE_DY / 2),
                  (-JETSON_HOLE_DX / 2, -JETSON_HOLE_DY / 2)]

    ex = -deck_len / 2 + ESP_L / 2 + 8
    esp_pts = [(ex + ESP_HOLE_DX / 2,  ESP_HOLE_DY / 2),
               (ex - ESP_HOLE_DX / 2,  ESP_HOLE_DY / 2),
               (ex + ESP_HOLE_DX / 2, -ESP_HOLE_DY / 2),
               (ex - ESP_HOLE_DX / 2, -ESP_HOLE_DY / 2)]

    gx = deck_len / 2 - GPS_L / 2 - 8
    gy =  deck_w / 2 - GPS_W / 2 - 6
    gps_pts = [(gx + GPS_HOLE_DX / 2, gy + GPS_HOLE_DY / 2),
               (gx - GPS_HOLE_DX / 2, gy + GPS_HOLE_DY / 2),
               (gx + GPS_HOLE_DX / 2, gy - GPS_HOLE_DY / 2),
               (gx - GPS_HOLE_DX / 2, gy - GPS_HOLE_DY / 2)]

    sxp = deck_len / 2 - GSM_L / 2 - 8
    syp = -(deck_w / 2 - GSM_W / 2 - 6)
    gsm_pts = [(sxp + GSM_HOLE_DX / 2, syp + GSM_HOLE_DY / 2),
               (sxp - GSM_HOLE_DX / 2, syp + GSM_HOLE_DY / 2),
               (sxp + GSM_HOLE_DX / 2, syp - GSM_HOLE_DY / 2),
               (sxp - GSM_HOLE_DX / 2, syp - GSM_HOLE_DY / 2)]

    cam_x_front = deck_len / 2 - 20
    cam_front_pts = [(cam_x_front + CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                     (cam_x_front - CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                     (cam_x_front + CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2),
                     (cam_x_front - CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2)]

    cam_x_rear = -deck_len / 2 + 20
    cam_rear_pts = [(cam_x_rear + CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                    (cam_x_rear - CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                    (cam_x_rear + CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2),
                    (cam_x_rear - CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2)]

    # Build the deck as a plate + separate cut cylinders (one per hole group)
    d = (cq.Workplane("XY")
         .rect(deck_len, deck_w).extrude(PLATE)
         .edges("|Z").fillet(6))

    def cut_holes(solid, points, diam):
        cutter = cq.Workplane("XY")
        first = True
        for (px, py) in points:
            c = (cq.Workplane("XY").center(px, py)
                 .circle(diam / 2).extrude(PLATE + 2)
                 .translate((0, 0, -1)))
            cutter = c if first else cutter.union(c)
            first = False
        return solid.cut(cutter)

    d = cut_holes(d, corner_pts,    M3_CLEAR)
    d = cut_holes(d, jetson_pts,    M3_CLEAR)
    d = cut_holes(d, esp_pts,       M25_CLEAR)
    d = cut_holes(d, gps_pts,       M3_CLEAR)
    d = cut_holes(d, gsm_pts,       M2_CLEAR)
    d = cut_holes(d, cam_front_pts, M3_CLEAR)
    d = cut_holes(d, cam_rear_pts,  M3_CLEAR)

    # Wire-pass slots (rectangles)
    def cut_slot(solid, cx, cy, lx, ly):
        s = (cq.Workplane("XY").center(cx, cy)
             .rect(lx, ly).extrude(PLATE + 2).translate((0, 0, -1)))
        return solid.cut(s)

    d = cut_slot(d, cam_x_front, 0, 14, 6)
    d = cut_slot(d, cam_x_rear,  0, 14, 6)
    d = cut_slot(d, -35, 0, 6, 18)
    d = cut_slot(d,  35, 0, 6, 18)
    return d


def camera_mount():
    """
    L-bracket camera mount. One set of holes bolts to the upper deck,
    the other set holds a 26x26 mm USB webcam module. A 20-deg tilt in
    the vertical leg gives a forward + slightly-down look angle.
    """
    tilt_deg = 20.0
    base_len = 40.0
    base_w   = 32.0
    up_h     = 48.0
    t        = 3.0

    base = (cq.Workplane("XY")
              .box(base_len, base_w, t, centered=(True, True, False)))
    # Base screws (match deck's 22 mm hole pattern on the front mount site)
    base = (base.faces(">Z").workplane()
                .pushPoints([( CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                             (-CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                             ( CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2),
                             (-CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2)])
                .hole(M3_CLEAR))

    # Upright with a tilt (rotate a box about its bottom edge)
    import math
    up = (
        cq.Workplane("XY")
        .transformed(offset=(base_len / 2 - t / 2, 0, 0),
                     rotate=(0, -tilt_deg, 0))
        .box(t, base_w, up_h, centered=(True, True, False))
    )
    # Camera mount holes in the upright face (22 x 22 pattern)
    up = (up.faces(">X").workplane()
             .center(0, up_h / 2 + 3)
             .pushPoints([( CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                          (-CAM_HOLE_DX / 2,  CAM_HOLE_DY / 2),
                          ( CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2),
                          (-CAM_HOLE_DX / 2, -CAM_HOLE_DY / 2)])
             .hole(M3_CLEAR))
    # Cable pass-through hole
    up = (up.faces(">X").workplane()
             .center(0, up_h / 2 + 3)
             .circle(5).cutThruAll())

    gusset_pts = [(base_len / 2 - t, -base_w / 2),
                  (base_len / 2 - t, base_w / 2),
                  (base_len / 2 - t - up_h * math.sin(math.radians(tilt_deg)),
                   0)]
    # Skipping gusset for geometry simplicity — upright is thick enough.
    return base.union(up)


def gps_antenna_tower():
    """Small pillar to elevate the GPS ceramic antenna above metal shields."""
    h = 55.0
    post = (cq.Workplane("XY")
              .rect(30, 30).extrude(PLATE)
              .faces(">Z").workplane()
              .rect(10, 10).extrude(h - PLATE))
    # Bolt holes (M3) in base to attach to top deck
    post = (post.faces("<Z").workplane()
                .pushPoints([(11, 11), (-11, 11), (11, -11), (-11, -11)])
                .hole(M3_CLEAR))
    # Flat top with M3 tap hole for the 25x25 antenna puck (double-sided
    # tape is usually enough, but the hole is there for a nylon bolt).
    top_plate = (cq.Workplane("XY").center(0, 0).rect(28, 28).extrude(2.4)
                   .translate((0, 0, h)))
    post = post.union(top_plate)
    post = (post.faces(">Z").workplane().hole(M3_TAP, depth=6))
    return post


def battery_strap_clip():
    """Small clip-on piece that closes the battery tray over the pack."""
    c = (cq.Workplane("XY").box(30, 10, 3)
            .faces(">Z").workplane()
            .pushPoints([(-10, 0), (10, 0)])
            .hole(M3_CLEAR))
    return c


# =============================================================================
#                             Export
# =============================================================================

OUT_STL  = Path("/sessions/dazzling-loving-goodall/mnt/outputs/car_chassis/stl")
OUT_STEP = Path("/sessions/dazzling-loving-goodall/mnt/outputs/car_chassis/step")
OUT_STL.mkdir(parents=True, exist_ok=True)
OUT_STEP.mkdir(parents=True, exist_ok=True)


PARTS = {
    "chassis_base":      chassis_base,
    "knuckle_left":      lambda: knuckle("left"),
    "knuckle_right":     lambda: knuckle("right"),
    "tie_rod":           tie_rod,
    "wheel":             wheel,
    "upper_deck":        upper_deck,
    "camera_mount":      camera_mount,
    "gps_antenna_tower": gps_antenna_tower,
    "battery_clip":      battery_strap_clip,
}


def export_all():
    for name, fn in PARTS.items():
        print(f"[build] {name}")
        obj = fn()
        cq.exporters.export(obj, str(OUT_STL / f"{name}.stl"))
        cq.exporters.export(obj, str(OUT_STEP / f"{name}.step"))
    print("Done.")


if __name__ == "__main__":
    export_all()
