"""Trigger-bridge node — WiFi TCP bridge for the driver-awareness ESP1.

ESP1 owns: AD8232 ECG (display only), response-test button + LED + buzzer,
16x2 LCD, SIM800L GSM.  This node owns the TCP link and translates ESP1's
line-based "EVT:..." protocol onto ROS2 topics, and forwards a handful of
Jetson-side commands (PARKED, RESET, PROMPT) back to ESP1.

Transport: ESP1 is a TCP CLIENT; the Jetson runs a TCP SERVER on
wifi_port (default 5005).

Wire protocol (matches firmware/esp1_driver_awareness.ino):

    ESP1 -> Jetson:
        EVT:READY                 boot / TCP (re)connect handshake
        EVT:BPM <n>               heart-rate telemetry (1 Hz) + link-alive HB
        EVT:LEADS_OFF/LEADS_OK    ECG electrodes detached / attached
        EVT:RESPONSE_FAIL         driver pressed the trigger button OR
                                  missed the response test
        EVT:RESPONSE_OK <ms>      driver passed the response test in time
        EVT:GO                    operator pressed the GO button
        EVT:ALERT_START / EVT:DONE
        EVT:SMS_OK / EVT:SMS_FAIL
        EVT:CALL_OK / EVT:CALL_ANSWERED / EVT:CALL_NOANSWER / EVT:CALL_FAIL
        EVT:RESET_OK
        EVT:STATUS BPM=<v> LEADS=<ok|off>

    Jetson -> ESP1:
        PARKED   car has come to rest after the PULL_OVER ramp; ESP1
                 runs the SMS + voice-call sequence.
        RESET    clear ESP1's RESPONSE_FAIL cooldown latch (re-arm).
        STATUS   request a status snapshot (text echoed back via EVT:STATUS).
        PROMPT   force a trigger (debug/demo — bypasses cooldown).

Mapping to /start_parking semantics:
    EVT:RESPONSE_FAIL → "go"   (autonomous park — driver unresponsive)
    EVT:GO            → "go"   (operator arm / pull-over now)
    EVT:RESPONSE_OK   → "go"   (passing the response test arms drive;
                                 the planner's autonomous demo handles
                                 the rest from a single 'go')

Link-liveness: any non-empty line refreshes the watchdog.  ESP1 emits
BPM at 1 Hz unconditionally so the link is always fed.
"""

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32


# ESP1 emits BPM at 1 Hz plus a flood of other lines on state changes;
# 600 s is generous enough to absorb WiFi jitter without false positives.
LINK_TIMEOUT_S = 600.0


class TriggerBridgeNode(Node):

    def __init__(self):
        super().__init__('trigger_bridge_node')

        # --- params -------------------------------------------------------
        self.declare_parameter('wifi_port', 5005)
        self.declare_parameter('simulate',  False)
        self.wifi_port = int(self.get_parameter('wifi_port').value)
        self.simulate  = bool(self.get_parameter('simulate').value)

        # --- publishers ---------------------------------------------------
        self.start_pub  = self.create_publisher(String, '/start_parking',      10)
        self.estop_pub  = self.create_publisher(Bool,   '/emergency_stop',     10)
        self.alive_pub  = self.create_publisher(Bool,   '/trigger_link_alive', 10)
        self.bpm_pub    = self.create_publisher(Int32,  '/heart_rate_bpm',     10)
        self.leads_pub  = self.create_publisher(Bool,   '/leads_ok',           10)

        # --- subscriptions ------------------------------------------------
        self.create_subscription(String, '/trigger_ack',  self._ack_cb,      10)
        self.create_subscription(String, '/behavior',     self._behavior_cb, 10)
        self.create_subscription(String, '/rearm',        self._rearm_cb,    10)
        self.create_subscription(String, '/force_prompt', self._prompt_cb,   10)

        # --- TCP server ---------------------------------------------------
        self._conn        = None
        self._conn_lock   = threading.Lock()
        self._server_sock = None

        if not self.simulate:
            try:
                self._server_sock = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
                self._server_sock.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server_sock.bind(('0.0.0.0', self.wifi_port))
                self._server_sock.listen(1)
                self.get_logger().info(
                    f"trigger_bridge TCP server listening on port {self.wifi_port}")
            except Exception as e:
                self.get_logger().error(
                    f"could not start TCP server on port {self.wifi_port}: {e}")
                self._server_sock = None

        # --- state --------------------------------------------------------
        self._estop_latched   = False
        self._last_rx_t       = 0.0
        self._last_link_alive = False
        self._last_behavior   = None
        self._leads_ok        = False

        # --- threads ------------------------------------------------------
        self._running = True
        if not self.simulate and self._server_sock:
            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True)
            self._accept_thread.start()

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f"trigger_bridge_node up; ESP1 EVT: protocol; "
            f"link_timeout={LINK_TIMEOUT_S:.1f}s")

    # ==================================================================
    #  TCP accept loop — waits for ESP1 (re)connect
    # ==================================================================
    def _accept_loop(self):
        while self._running:
            try:
                self.get_logger().info(
                    f"Waiting for trigger ESP to connect on port {self.wifi_port}...")
                conn, addr = self._server_sock.accept()
                self.get_logger().info(f"Trigger ESP connected from {addr}")
                with self._conn_lock:
                    if self._conn is not None:
                        try:
                            self._conn.close()
                        except Exception:
                            pass
                    self._conn = conn
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f"TCP accept error: {e}")
                time.sleep(1.0)

    # ==================================================================
    #  TCP RX loop
    # ==================================================================
    def _rx_loop(self):
        buf = b''
        while self._running and rclpy.ok():
            with self._conn_lock:
                conn = self._conn
            if conn is None:
                time.sleep(0.2)
                continue
            try:
                chunk = conn.recv(128)
            except Exception as e:
                self.get_logger().warn(f"socket read error: {e}")
                with self._conn_lock:
                    self._conn = None
                buf = b''
                time.sleep(0.2)
                continue
            if not chunk:
                self.get_logger().warn("trigger ESP disconnected")
                with self._conn_lock:
                    self._conn = None
                buf = b''
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle_line(line.decode('ascii',
                                              errors='replace').strip())

    # ==================================================================
    def _handle_line(self, line: str):
        if not line:
            return
        # Any non-empty line refreshes the watchdog.
        self._last_rx_t = time.time()

        if line.startswith('EVT:'):
            evt = line[4:].strip()
        else:
            self.get_logger().info(f"[esp-trigger] (raw) {line}")
            return

        # BPM stream — quiet (1 Hz).
        if evt.startswith('BPM '):
            try:
                bpm = int(evt.split()[1])
                self.bpm_pub.publish(Int32(data=bpm))
            except (ValueError, IndexError):
                pass
            return

        self.get_logger().info(f"[esp-trigger] EVT:{evt}")

        if evt == 'LEADS_OK':
            self._leads_ok = True
            self.leads_pub.publish(Bool(data=True))
            return
        if evt == 'LEADS_OFF':
            self._leads_ok = False
            self.leads_pub.publish(Bool(data=False))
            return

        if evt == 'GO':
            if self._estop_latched:
                self.get_logger().warn(
                    "ignoring GO press while emergency_stop is latched")
                return
            self.start_pub.publish(String(data='go'))
            self.get_logger().info(
                "published /start_parking 'go' (operator pressed GO)")
            return

        if evt == 'RESPONSE_OK' or evt.startswith('RESPONSE_OK '):
            if self._estop_latched:
                self.get_logger().warn(
                    "ignoring RESPONSE_OK while emergency_stop is latched")
                return
            self.start_pub.publish(String(data='go'))
            self.get_logger().info(
                "published /start_parking 'go' (driver passed response test)")
            return

        if evt == 'RESPONSE_FAIL':
            # Driver unresponsive — kick the planner into the autonomous
            # park flow.  The planner's autonomous demo walks the full
            # LANE_FOLLOW → ... → PULL_OVER → PARKED chain on a single
            # 'go', no follow-up 'park' needed.
            self.get_logger().error(
                "RESPONSE_FAIL — driver unresponsive, starting autonomous park")
            self.start_pub.publish(String(data='go'))
            return

        if evt == 'READY':
            # ESP1 just (re)connected.  We do not auto-clear estop here —
            # operator must explicitly /rearm.
            return

        # Everything else (ALERT_START, SMS_*, CALL_*, DONE, RESET_OK,
        # STATUS) is informational — already logged above.

    # ==================================================================
    def _set_estop(self, latched: bool, reason: str = ''):
        if latched == self._estop_latched:
            self.estop_pub.publish(Bool(data=latched))
            return
        self._estop_latched = latched
        self.estop_pub.publish(Bool(data=latched))
        if latched:
            self.get_logger().error(f"EMERGENCY STOP latched ({reason})")
        else:
            self.get_logger().info(f"emergency_stop cleared ({reason})")

    def _write_line(self, text: str):
        with self._conn_lock:
            conn = self._conn
        if conn is None:
            return
        try:
            conn.sendall((text + '\n').encode('ascii', errors='replace'))
        except Exception as e:
            self.get_logger().warn(f"socket write failed: {e}")
            with self._conn_lock:
                self._conn = None

    # ==================================================================
    def _watchdog_tick(self):
        now = time.time()
        alive = (self._last_rx_t > 0.0 and
                 (now - self._last_rx_t) < LINK_TIMEOUT_S)
        self.alive_pub.publish(Bool(data=alive))

        if alive != self._last_link_alive:
            if alive:
                self.get_logger().info("trigger ESP heartbeat OK")
            else:
                if self._last_rx_t > 0.0:
                    self.get_logger().error(
                        f"trigger ESP link lost "
                        f"(no traffic for >= {LINK_TIMEOUT_S:.1f}s) — "
                        f"latching emergency_stop")
                    self._set_estop(True, reason='link lost')
            self._last_link_alive = alive

    # ==================================================================
    def _ack_cb(self, msg: String):
        self._write_line(f"ACK:{msg.data}")

    def _behavior_cb(self, msg: String):
        """Forward PARKED to ESP1 on the PULL_OVER → PARKED transition.
        That is the moment the car has finished its pull-over ramp and is
        ESP1's trigger for the SMS + voice-call sequence."""
        new_state = msg.data.strip().upper()
        if (self._last_behavior in ('PULL_OVER', 'SEARCH_PARK')
                and new_state == 'PARKED'):
            self.get_logger().info(
                f"{self._last_behavior} → PARKED: forwarding PARKED to ESP1")
            self._write_line("PARKED")
        self._last_behavior = new_state

    def _prompt_cb(self, _msg: String):
        self.get_logger().info("forwarding PROMPT to trigger ESP (forced response test)")
        self._write_line("PROMPT")

    def _rearm_cb(self, _msg: String):
        if not self._estop_latched:
            self.get_logger().info("/rearm received but no estop is latched")
            return
        if not self._last_link_alive:
            self.get_logger().error(
                "/rearm refused -- trigger ESP link is dead")
            return
        self.get_logger().info("forwarding RESET to trigger ESP, clearing estop latch")
        self._write_line("RESET")
        self._set_estop(False, reason='operator rearm')

    # ==================================================================
    def destroy_node(self):
        self._running = False
        try:
            with self._conn_lock:
                if self._conn is not None:
                    self._conn.close()
        except Exception:
            pass
        try:
            if self._server_sock is not None:
                self._server_sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriggerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
