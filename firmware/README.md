# ESP32 Firmware

## Motor controller

`motor_esp/motor_esp.ino` receives line-delimited USB-serial commands from the ROS 2 motor bridge:

- `TS <drive> <steer>` — combined drive and steering command, each in `[-255, 255]`
- `B` — hard brake and center steering
- `P` — coast to park, center steering, and emit `EVT:PARKED`
- `H` — heartbeat
- `STATUS` — report the current drive, steering, and servo-angle state

The final hardware arrangement is one H-bridge-controlled DC drive motor plus a 50 Hz steering servo. A command watchdog brakes the drive if the serial stream stalls.

## Driver-awareness controller

`trigger_esp/trigger_esp.ino` integrates:

- AD8232 ECG input and lead-off detection;
- a normally-closed latching emergency-stop input;
- a Wi-Fi TCP heartbeat and trigger link to the Jetson;
- optional GPS position and SIM800L SMS/voice alert handling.

The public file contains non-working placeholders:

```cpp
WIFI_SSID  = "YOUR_WIFI_SSID";
WIFI_PASS  = "YOUR_WIFI_PASSWORD";
JETSON_IP  = "192.168.1.100";
ALERT_PHONE = "+000000000000";
```

Replace them only in a local working copy before flashing. Do not commit real credentials, private addresses, or phone numbers.

## Important hardware notes

- Power the servo and SIM800L from supplies sized for their current peaks; do not use the ESP32 3.3 V rail.
- Connect all grounds.
- Level-shift the ESP32 TX line if required by the modem board.
- Verify motor direction and servo end stops with the wheels off the ground.
- This firmware is for a controlled academic prototype, not a road vehicle.
