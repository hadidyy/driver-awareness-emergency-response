"""Motor-bridge node — forwards control commands to the motor ESP over USB serial.

Rewritten 2026-05-18 to fix the boot-brake trap that caused the
"car doesn't move when launched" bug.

Bug in approach 2:
    self._t_last_th = 0.0
    self._t_last_st = 0.0
    ...
    stale = ((now - self._t_last_th) > MOTOR_WATCHDOG_S or
             (now - self._t_last_st) > MOTOR_WATCHDOG_S or
             self._t_last_th == 0 or self._t_last_st == 0)
                                    ^^^^^^^^^^^^^^^^^^^^^^^
The explicit equality test against 0 trapped the bridge in
"send B\\n forever" until BOTH callbacks fired.  In the worst case (DDS
late-joiner discovery, a node-launch order quirk), the bridge could sit
in brake for many seconds.  demo_show_run.py worked because it
explicitly published brake=0.0 + behavior=LANE_FOLLOW in __init__ before
the first timer tick, racing the bridge's discovery.

Fix:
  * Initialise t_last_* to a sentinel that is older than the watchdog
    but is not 0.0 (so the explicit ==0 trap can simply be deleted).
  * Drop the ==0 check from the stale predicate.
  * On the FIRST inbound throttle+steer callback after boot, log a
    "first commands received — leaving boot brake" line so the bug is
    obvious on the bench if it ever recurs.

Wire protocol unchanged:
    TS <drive> <steer>\\n  drive + steer ints in -255..255
    B\\n                    hard brake
    P\\n                    park
    H\\n                    heartbeat

Safety layers preserved (all independent):
  1.  Stale-cmd watchdog (MOTOR_WATCHDOG_S timeout)
  2.  /emergency_stop override
  3.  /behavior == PARKED override
  4.  /brake_cmd >= 0.5 override
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

from .common import MOTOR_CMD_HZ, MOTOR_WATCHDOG_S, Behavior


class MotorBridgeNode(Node):

    def __init__(self):
        super().__init__('motor_bridge_node')

        # --- params -------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB_MOTOR')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('simulate',     False)
        self.port     = str(self.get_parameter('serial_port').value)
        self.baud     = int(self.get_parameter('baud').value)
        self.simulate = bool(self.get_parameter('simulate').value)

        # --- serial -------------------------------------------------------
        self.ser  = None
        self._ser_lock = threading.Lock()
        if not self.simulate:
            try:
                import serial   # type: ignore
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(
                    f"motor_bridge opened {self.port} @ {self.baud}")
            except Exception as e:
                self.get_logger().error(
                    f"could not open {self.port}: {e} — running in "
                    f"log-only mode")
                self.ser = None

        # --- state --------------------------------------------------------
        self._throttle  = 0.0
        self._steer     = 0.0
        self._brake     = 1.0   # start braked

        # Init t_last_* to a value that is *older* than MOTOR_WATCHDOG_S
        # but is NOT 0.0.  The stale-cmd check below uses only the
        # timeout comparison, so the explicit `== 0` trap from the prior
        # implementation is gone.
        boot_sentinel = time.time() - (MOTOR_WATCHDOG_S + 1.0)
        self._t_last_th = boot_sentinel
        self._t_last_st = boot_sentinel
        self._got_first_th = False
        self._got_first_st = False
        self._estop     = False
        self._behavior  = Behavior.HOLD.value
        self._parked_sent = False
        self._last_frame_logged = None

        # --- subscriptions ------------------------------------------------
        self.create_subscription(Float32, '/throttle_cmd',   self._th_cb,    10)
        self.create_subscription(Float32, '/steer_cmd',      self._st_cb,    10)
        self.create_subscription(Float32, '/brake_cmd',      self._br_cb,    10)
        self.create_subscription(Bool,    '/emergency_stop', self._estop_cb, 10)
        self.create_subscription(String,  '/behavior',       self._beh_cb,   10)

        # --- TX loop ------------------------------------------------------
        self.create_timer(1.0 / max(1, MOTOR_CMD_HZ), self._tx_tick)

        self.get_logger().info("motor_bridge ready — awaiting control commands")

    # ==================================================================
    def _th_cb(self, m):
        self._throttle = max(-1.0, min(1.0, float(m.data)))
        self._t_last_th = time.time()
        if not self._got_first_th:
            self._got_first_th = True
            self._log_first_if_ready()

    def _st_cb(self, m):
        self._steer = max(-1.0, min(1.0, float(m.data)))
        self._t_last_st = time.time()
        if not self._got_first_st:
            self._got_first_st = True
            self._log_first_if_ready()

    def _br_cb(self, m):
        self._brake = max(0.0, min(1.0, float(m.data)))

    def _estop_cb(self, m):
        latched = bool(m.data)
        if latched != self._estop:
            if latched:
                self.get_logger().error(
                    "EMERGENCY STOP — forcing brake frames at the bridge")
            else:
                self.get_logger().info("emergency_stop cleared at bridge")
        self._estop = latched

    def _beh_cb(self, m):
        new_beh = str(m.data)
        if new_beh != Behavior.PARKED.value:
            self._parked_sent = False
        self._behavior = new_beh

    def _log_first_if_ready(self):
        if self._got_first_th and self._got_first_st:
            self.get_logger().info(
                "first throttle+steer commands received — leaving boot brake")

    # ==================================================================
    def _tx_tick(self):
        now = time.time()
        stale = ((now - self._t_last_th) > MOTOR_WATCHDOG_S or
                 (now - self._t_last_st) > MOTOR_WATCHDOG_S)

        # PARKED override.
        if self._behavior == Behavior.PARKED.value:
            if not self._parked_sent:
                self._send("P\n")
                self._parked_sent = True
                return
            self._send("B\n")
            return

        if self._estop or stale or self._brake >= 0.5:
            self._send("B\n")
            return

        drive_int = int(round(self._throttle * 255.0))
        steer_int = int(round(self._steer    * 255.0))
        drive_int = max(-255, min(255, drive_int))
        steer_int = max(-255, min(255, steer_int))
        self._send(f"TS {drive_int} {steer_int}\n")

    # ==================================================================
    def _send(self, frame: str):
        # Cheap log throttle — only emit when the frame *category*
        # changes so the bench operator can see what's going on without
        # being flooded.
        cat = frame.split(' ', 1)[0].strip()
        if cat != self._last_frame_logged:
            self.get_logger().debug(f"-> {frame.strip()}")
            self._last_frame_logged = cat
        if self.ser is not None:
            try:
                with self._ser_lock:
                    self.ser.write(frame.encode('ascii'))
            except Exception as e:
                self.get_logger().warn(f"serial write failed: {e}")
        elif self.simulate:
            self.get_logger().info(f"SIM -> {frame.strip()}")

    # ==================================================================
    def destroy_node(self):
        try:
            if self.ser is not None:
                with self._ser_lock:
                    self.ser.write(b"B\n")
                    self.ser.flush()
                    self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
