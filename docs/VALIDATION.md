# Validation Summary

The simulation harness imports the real ROS 2 package modules through a small local ROS-message shim. It drives their callbacks and timers synchronously and checks the published outputs.

## Automated scenarios

| # | Scenario | Main behavior checked |
| ---: | --- | --- |
| 1 | Boot, trigger, and cruise | Safe boot state, explicit arming, throttle floor, centered steering |
| 2 | Obstacle and lane change | Dwell-gated transition, one-shot merge, vision-based exit |
| 3 | Autonomous pull-over | Post-merge dwell, parking search, speed decay, parked brake |
| 4 | Emergency stop | Immediate hold, zero throttle, brake, explicit re-arm requirement |
| 5 | Trigger-link loss | Communication watchdog forces hold |
| 6 | Lane loss | Steering authority decays and state counters pause |
| 7 | Blocked blind spot | Slow-down behavior prevents unsafe merge |
| 8 | Post-merge obstacle muting | False positives cannot trigger a second lane change |
| 9 | Synthetic lane frames | Lane position sign, confidence, polynomial output, debouncing |
| 10 | Pure pursuit | Correct steering direction and proportional fallback |
| 11 | Obstacle blips | Anti-chatter behavior |
| 12 | Minimum dwell | Early transition is rejected |
| 13 | Motor bridge boot | First fresh command leaves boot brake; parked frame is emitted |
| 14 | Throttle inversion | Minimum output magnitude survives sign inversion |
| 15 | Confidence weighting | Low-confidence perception reduces steering authority |

Result for this snapshot: **63 passed, 0 failed**.

The shim instantiates nodes with their compiled parameter defaults. Hardware calibration overrides in `jetson_driver/config/driver_params.yaml` are reviewed separately and still require bench validation.

## Run it

```bash
PYTHONPATH=sim:jetson_driver python3 sim/sim_runner.py
```

On PowerShell:

```powershell
$env:PYTHONPATH = "sim;jetson_driver"
python sim\sim_runner.py
```

## Hardware validation still required

Simulation does not validate:

- real USB-camera frame rate, exposure, dropped frames, or track lighting;
- YOLO inference latency and temperature on the Jetson;
- motor breakaway PWM and current draw on the real surface;
- servo end stops, steering geometry, and mechanical backlash;
- ESP32 USB/Wi-Fi latency under electrical noise;
- GPS fix time, GSM registration, or alert delivery;
- vehicle stopping distance or road-level functional safety.

Bench testing should start wheels-up, then move to a controlled low-speed track with a physical emergency stop and a spotter.
