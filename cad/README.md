# Driver-Awareness Car — 3D Printable CAD Bundle

A parametric, Ackermann-steering car chassis designed to carry the full
driver-awareness stack (Jetson Nano, ESP32, two USB webcams, GPS, GSM, battery,
2× TT motors, SG90 servo) with room to spare.

## What's in this bundle

```
car_chassis/
├── README.md                  ← this file
├── src/
│   ├── car_chassis.py         ← parametric CadQuery source (edit + re-run)
│   └── render_preview.py      ← script that produced the preview PNGs
├── step/                      ← open these directly in SolidWorks
│   ├── chassis_base.step
│   ├── upper_deck.step
│   ├── knuckle_left.step
│   ├── knuckle_right.step
│   ├── tie_rod.step
│   ├── wheel.step             ← print 4×
│   ├── camera_mount.step      ← print 2× (front + rear)
│   ├── gps_antenna_tower.step
│   └── battery_clip.step
├── stl/                       ← slice these with Cura / PrusaSlicer / Bambu Studio
│   └── (same part list as step/)
└── preview/
    ├── iso_front.png
    ├── iso_rear.png
    └── top.png
```

SolidWorks users: open the STEP files. They import cleanly as solids and you
can edit, add constraints, or rebuild features as native SolidWorks bodies.

## Dimensions at a glance

| Dimension                | Value     |
|--------------------------|-----------|
| Wheelbase (F-R axle)     | 170 mm    |
| Track (wheel-to-wheel)   | 140 mm    |
| Chassis plate            | 210 × 130 × 3 mm |
| Upper deck               | 180 × 125 × 3 mm |
| Deck height above floor  | 75 mm     |
| Wheel OD × width         | 65 × 26 mm |
| Overall footprint        | ~240 × 200 mm (incl. wheels) |

Every chassis part is ≤ 210 mm long — fits on any 220 × 220 mm bed
(Ender 3, Prusa Mini, Bambu A1 Mini, etc.).

## Bill of Materials — printed parts

| Part                  | Qty | Notes                                           |
|-----------------------|-----|-------------------------------------------------|
| `chassis_base`        | 1   | Flip in slicer: print upside-down for flat top face |
| `upper_deck`          | 1   | Lay flat                                        |
| `knuckle_left`        | 1   | Supports from build plate only                  |
| `knuckle_right`       | 1   | Supports from build plate only                  |
| `tie_rod`             | 1   | Lay flat, 100 % infill                          |
| `wheel`               | 4   | 20 % infill is fine; print rim-up               |
| `camera_mount`        | 2   | One forward-looking, one rearward               |
| `gps_antenna_tower`   | 1   | Print with the flat base on the build plate     |
| `battery_clip`        | 1–2 | Small, print several spares                     |

Print settings I recommend:
- Material: PLA or PETG, 0.2 mm layer
- Walls: 3 perimeters, 25 % infill for structure / 15 % for wheels
- Chassis + deck + knuckles: print solid tops of 4+ layers so the M3
  screws have fresh plastic to bite into (self-tapping into the bosses)

## Bill of Materials — hardware

| Item                              | Qty | Where it goes                        |
|-----------------------------------|-----|--------------------------------------|
| M3 × 10 mm pan-head screw         | 16  | Deck-to-standoff, module mounts      |
| M3 × 25 mm socket-head screw      | 2   | King-pins (one per front knuckle)    |
| M3 × 20 mm socket-head screw      | 2   | Front stub axles                     |
| M3 × 8 mm + nut (motor mounts)    | 4   | TT motor → rear bulkhead             |
| M3 nylock nut                     | 6   | King-pins + stub axles               |
| M2.5 × 6 mm screw + nut           | 4   | ESP32 mount                          |
| M2 × 6 mm screw + nut             | 4   | SIM800L mount                        |
| M2 × 10 mm self-tapping           | 4   | SG90 servo flange                    |
| Brass M3 female standoff (70 mm)  | 4   | *(optional)* sturdier deck risers — the printed bosses work too |
| 16 – 20 mm brass standoff         | 4   | Jetson Nano feet (so USB ports clear the deck) |
| Rubber O-ring (~65 mm ID × 3 mm)  | 4   | Snaps into each wheel's tire groove  |

## Electronics the chassis is sized around

| Module            | Assumed footprint | Hole pattern |
|-------------------|-------------------|--------------|
| Jetson Nano B01   | 100 × 80 mm       | M3, 86 × 58 mm |
| ESP32 DevKit v1   | 55 × 28 mm        | M2.5, 48 × 22 mm |
| NEO-6M GPS        | 36 × 24 mm        | M3, 30 × 17.5 mm |
| SIM800L GSM       | 40 × 28 mm        | M2, 36 × 24 mm |
| SG90 servo        | 23 × 12.5 × 22.7  | M2, 32.5 mm flange span |
| TT yellow motor   | 70 × 22 × 37 mm   | M3, 17.5 mm span |
| USB webcam (ELP)  | 28 × 28 × 28 mm   | M3, 22 × 22 mm |
| 2S LiPo / 2×18650 | ~80 × 45 × 22 mm  | strap / velcro |

If any of these are different for *your* modules, open
`src/car_chassis.py`, edit the numbers in the `### PARAMETERS ###` block,
and re-run `python3 car_chassis.py` to regenerate every STL and STEP.

## Assembly order

1. **Rear drivetrain.** Bolt both TT motors to the rear bulkhead (M3 × 8, nut
   behind). Shafts point outward through the shaft-clearance holes.
2. **Rear wheels.** Push a wheel onto each TT motor's D-shaft. The wheel's
   hub is already D-keyed, so it can't slip. Secure with a drop of hot glue
   or a 2 mm set-screw through the hub.
3. **Front steering.** Slide each knuckle onto its king-pin boss with an
   M3 × 25 socket-head from below, nylock on top. The knuckle should rotate
   freely. Bolt the tie rod to the inner arm of each knuckle (M3 × 16 with
   nylocks, not fully tightened — they need to pivot).
4. **Front wheels.** Each front wheel rides on an M3 × 20 stub axle passing
   through the horizontal arm of the knuckle. Thread the M3 into a printed
   collar or use a nylock so the wheel can spin.
5. **Servo.** Drop the SG90 into the servo pocket on the chassis. Secure
   via its two flange screws into the pocket. Use a linkage (an unbent
   paper-clip works for prototyping) from the servo horn to one steering
   arm — the other knuckle follows via the tie rod.
6. **Battery.** Lay the battery pack between the side walls, secure with a
   velcro strap through the cutouts. Wire a 5 V 3–5 A UBEC for the Jetson
   and a separate 6 V BEC / 5 V buck for the servo.
7. **Upper deck.** Set it on the four standoff bosses, bolt through with
   M3 × 10s. Feed camera / motor / GPS antenna wires up through the spine
   slots before fully tightening.
8. **Electronics.** Mount Jetson Nano in the center of the deck (use the 4 M3
   screws with 16 mm brass standoffs so the USB-A ports clear the deck).
   Mount ESP32 at the rear, GPS front-left, SIM800L front-right. GPS
   antenna puck goes on the GPS tower on top of the deck.
9. **Cameras.** Bolt one camera mount to the front cam pattern on the deck
   and one to the rear pattern. Feed USB cables down through the cable
   slot next to each mount.

## Wiring at a glance

```
Battery (2S LiPo, 7.4V)
  ├── UBEC 5V/5A ──► Jetson Nano barrel jack (J25, set jumper for barrel)
  ├── BEC 6V/2A  ──► SG90 servo V+
  └── Direct 7.4V ──► ESP32 VIN (via onboard reg) and L298N / TB6612 motor driver

Jetson Nano
  ├── USB ──► Webcam front (/dev/video0)
  ├── USB ──► Webcam rear  (/dev/video1)
  ├── USB ──► GPS (NEO-6M via USB-TTL or UART pins)
  ├── UART ──► ESP32 (low-level motor/servo control)
  └── UART/USB ──► SIM800L (AT commands)

ESP32
  ├── PWM × 2 ──► Motor driver IN1/IN2/IN3/IN4
  ├── PWM × 1 ──► Servo signal
  └── UART ──► Jetson Nano (bidirectional)
```

## Tuning the design

Open `src/car_chassis.py`. Every dimension is near the top under
`### PARAMETERS ###`. Typical tweaks:

- **Bigger wheelbase:** increase `WHEELBASE` (the rear bulkhead and front
  bulkhead follow automatically).
- **Different motor:** change `TT_BODY_*` and `TT_SHAFT_D` — the rear
  bulkhead holes regenerate around the new motor size.
- **Bigger battery:** increase `BATT_L`, `BATT_W`, `BATT_H`. The side
  walls move out to match.
- **Different Jetson carrier:** change `JETSON_HOLE_DX` / `JETSON_HOLE_DY`.
  The hole pattern on the upper deck regenerates. (Jetson Orin Nano Dev
  Kit has the same 86 × 58 pattern as the B01, so no change needed there.)

Regenerate with:

```bash
pip install cadquery
cd src
python3 car_chassis.py
```

That rewrites every `stl/*.stl` and `step/*.step` from the current parameters.

## Known compromises

- The front wheels ride on plain M3 stub axles with no bearing — fine for
  rolling at robot-car speeds, replace with 623ZZ bearings (10 × 3 × 4 mm)
  pressed into the knuckle for a much smoother feel if you care.
- The design assumes a small square USB camera (ELP-style, 22 × 22 mount
  pattern). If you're using a Logitech C270 or similar clip-base webcam,
  print the base and epoxy the camera's clip onto it — the bolt holes
  won't line up with a C270.
- The GPS antenna tower has no RF shielding. For best results, keep the
  antenna ≥ 25 mm away from the Jetson Nano module (the default layout
  already does this).

## License

CC0 / public domain. Build it, modify it, ship it, sell it, no attribution
required. If you publish improvements, a mention would be appreciated.
