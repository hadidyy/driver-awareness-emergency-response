"""Render labeled assembly previews of the car from multiple angles."""
import numpy as np
import trimesh
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa
from pathlib import Path

STL_DIR = Path("/sessions/dazzling-loving-goodall/mnt/outputs/car_chassis/stl")
OUT_DIR = Path("/sessions/dazzling-loving-goodall/mnt/outputs/car_chassis/preview")
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Same constants as the CAD file (kept in sync by hand)
WHEELBASE = 170.0
TRACK     = 140.0
WHEEL_OD  = 65.0
WHEEL_W   = 26.0
DECK_HEIGHT = 75.0
KNUCKLE_H = 22.0

def load(name):
    return trimesh.load(str(STL_DIR / f"{name}.stl"))

def place(m, translation=(0, 0, 0), rotation=None):
    m = m.copy()
    if rotation is not None:
        axis, angle_deg = rotation
        R = trimesh.transformations.rotation_matrix(
            np.radians(angle_deg), axis)
        m.apply_transform(R)
    m.apply_translation(translation)
    return m

# Build the assembly
chassis      = load("chassis_base")
upper        = place(load("upper_deck"),         (0, 0, DECK_HEIGHT))
knuckle_L    = place(load("knuckle_left"),       (WHEELBASE / 2, TRACK / 2 - 10, 0))
knuckle_R    = place(load("knuckle_right"),      (WHEELBASE / 2, -(TRACK / 2 - 10), 0))
# Wheels: rear wheels sit on TT motor shafts at -WHEELBASE/2, rotate 90deg about X
def wheel_at(y, side_sign=1, x=0):
    # wheel model has its axis along Z; rotate so axis aligns with Y
    w = load("wheel")
    R = trimesh.transformations.rotation_matrix(np.radians(90), [1, 0, 0])
    w.apply_transform(R)
    w.apply_translation((x, y, WHEEL_OD / 2))
    return w

wheels = [
    wheel_at(-(TRACK / 2),          x=-WHEELBASE / 2),  # rear-right
    wheel_at( (TRACK / 2),          x=-WHEELBASE / 2),  # rear-left
    wheel_at(-(TRACK / 2 - 2),      x= WHEELBASE / 2),  # front-right
    wheel_at( (TRACK / 2 - 2),      x= WHEELBASE / 2),  # front-left
]

# Representative electronic modules (drawn as simple boxes)
def box(size, center, color):
    m = trimesh.creation.box(extents=size)
    m.apply_translation(center)
    m.visual.face_colors = color
    return m

jetson_module = box((100, 80, 29), (0, 0, DECK_HEIGHT + 3 + 29/2), (30, 130, 220, 255))
esp32_module  = box((55, 28, 13),  (-80, 0, DECK_HEIGHT + 3 + 13/2), (200, 40, 40, 255))
gps_module    = box((36, 24, 8),   ( 60,  40, DECK_HEIGHT + 3 + 4),  (50, 180, 80, 255))
gsm_module    = box((40, 28, 10),  ( 60, -40, DECK_HEIGHT + 3 + 5),  (220, 150, 30, 255))
battery_pack  = box((80, 45, 20),  (-20, 0, 10),                      (90, 90, 90, 255))
cam_front     = box((26, 26, 28),  ( WHEELBASE/2 - 10, 0, DECK_HEIGHT + 45),
                    (180, 50, 180, 255))
cam_rear      = box((26, 26, 28),  (-WHEELBASE/2 + 10, 0, DECK_HEIGHT + 45),
                    (180, 50, 180, 255))
tt_motor_L    = box((70, 22, 37),  (-WHEELBASE/2 - 35, 35, 37/2 + 3),
                    (60, 60, 60, 255))
tt_motor_R    = box((70, 22, 37),  (-WHEELBASE/2 - 35, -35, 37/2 + 3),
                    (60, 60, 60, 255))
servo_body    = box((23, 12.5, 22.7), (WHEELBASE/2 - 30, 0, 22.7/2),
                    (240, 240, 240, 255))

# Combine all into one scene
scene_meshes = {
    "chassis": chassis,
    "upper_deck": upper,
    "knuckle_L": knuckle_L,
    "knuckle_R": knuckle_R,
}
for i, w in enumerate(wheels):
    scene_meshes[f"wheel_{i}"] = w

# Render with matplotlib (wire-free, fast)
def render_view(elev, azim, outfile, title):
    fig = plt.figure(figsize=(12, 9), dpi=140)
    ax = fig.add_subplot(111, projection='3d')

    def add_mesh(m, facecolor=(0.78, 0.78, 0.82, 0.95), edgecolor=(0.3, 0.3, 0.3, 0.6), lw=0.1):
        verts = m.vertices[m.faces]
        coll = Poly3DCollection(verts, linewidths=lw)
        coll.set_facecolor(facecolor)
        coll.set_edgecolor(edgecolor)
        ax.add_collection3d(coll)

    # Printed parts: light plastic color
    for m in scene_meshes.values():
        add_mesh(m)

    # Electronics boxes with their own colors
    def add_box(m, color):
        verts = m.vertices[m.faces]
        coll = Poly3DCollection(verts, linewidths=0.1)
        rgba = tuple(c / 255 for c in color[:3]) + (0.85,)
        coll.set_facecolor(rgba)
        coll.set_edgecolor((0, 0, 0, 0.6))
        ax.add_collection3d(coll)

    add_box(jetson_module, (30, 130, 220))
    add_box(esp32_module,  (200, 40, 40))
    add_box(gps_module,    (50, 180, 80))
    add_box(gsm_module,    (220, 150, 30))
    add_box(battery_pack,  (90, 90, 90))
    add_box(cam_front,     (180, 50, 180))
    add_box(cam_rear,      (180, 50, 180))
    add_box(tt_motor_L,    (60, 60, 60))
    add_box(tt_motor_R,    (60, 60, 60))
    add_box(servo_body,    (240, 240, 240))

    # Compute overall bounds
    all_pts = np.vstack([m.vertices for m in list(scene_meshes.values()) +
                          [jetson_module, esp32_module, gps_module, gsm_module,
                           battery_pack, cam_front, cam_rear,
                           tt_motor_L, tt_motor_R, servo_body]])
    mn = all_pts.min(axis=0)
    mx = all_pts.max(axis=0)
    ctr = (mn + mx) / 2
    span = (mx - mn).max() * 0.55

    ax.set_xlim(ctr[0] - span, ctr[0] + span)
    ax.set_ylim(ctr[1] - span, ctr[1] + span)
    ax.set_zlim(ctr[2] - span, ctr[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X (fwd) mm")
    ax.set_ylabel("Y mm")
    ax.set_zlabel("Z mm")
    ax.set_title(title, fontsize=14, pad=10)
    ax.grid(True, alpha=0.3)

    # Legend for colored boxes
    handles = [
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(30/255,130/255,220/255), markersize=12, label='Jetson Nano'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(200/255,40/255,40/255), markersize=12, label='ESP32'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(50/255,180/255,80/255), markersize=12, label='GPS'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(220/255,150/255,30/255), markersize=12, label='SIM800L GSM'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(90/255,90/255,90/255), markersize=12, label='Battery'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(180/255,50/255,180/255), markersize=12, label='USB webcams'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(0.78,0.78,0.82), markersize=12, label='Printed parts'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=(240/255,240/255,240/255), markersize=12, label='SG90 servo'),
    ]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9)

    plt.tight_layout()
    plt.savefig(outfile, bbox_inches='tight', dpi=140)
    plt.close()
    print(f"wrote {outfile}")

render_view(25, -55, str(OUT_DIR / "iso_front.png"),
            "3D-printable Ackermann Car — Isometric (Front)")
render_view(25, 125, str(OUT_DIR / "iso_rear.png"),
            "3D-printable Ackermann Car — Isometric (Rear)")
render_view(90, -90, str(OUT_DIR / "top.png"),
            "Top View — Component layout")
render_view(0,  -90, str(OUT_DIR / "side.png"),
            "Side View — Height profile")
