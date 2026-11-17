// =====================================================================
// trigger_esp.ino  --  driver-awareness "trigger" ESP32  (WiFi TCP + GSM)
//
// Transport: TCP CLIENT to <JETSON_IP>:<JETSON_PORT>.  The Jetson runs
// the server -- see jetson_driver/trigger_bridge_node.py.  Both the ESP
// and the Jetson must be on the same WiFi network (iPhone hotspot in
// the bench setup).
//
// Owns:
//   * AD8232 ECG front-end with leads-off detection ("driver present?")
//   * Latching mushroom emergency-stop button (NC, fail-safe wiring)
//   * GO button (manual fallback)
//   * 5 Hz heartbeat to the Jetson (link watchdog)
//   * Auto "go" on first stable heartbeat
//   * SIM800L cellular modem (SMS + voice call)        <-- added
//   * Neo-6M / generic GPS via TinyGPS++                <-- added
//   * On reception of "PARKED" from the Jetson, runs the full
//     emergency-alert sequence: GPS fix -> SMS with lat/lng -> voice
//     call -> beeps if answered -> hang up.
//
// Wire protocol with trigger_bridge_node:
//
//   ESP -> Jetson (text, '\n' terminated):
//     "HB"                heartbeat, every HB_PERIOD_MS
//     "go"                first stable heartbeat OR manual GO press
//     "ESTOP"             mushroom button is latched open
//     "ABORT:HR_LOST"     leads detached OR no R-peak inside HR_LOST_MS
//                         while the Jetson reported STATE:DRIVING
//     "READY"             sent on boot / TCP (re)connect / successful re-arm
//     "BPM:<n>"           heart-rate telemetry, ~1 Hz once leads good
//     "REARM_REJECTED:*"  ARM was sent but conditions not safe
//     -- alert sequence progress (in order, after Jetson "PARKED"): --
//     "ALERT_START"           emergency sequence beginning
//     "GPS:<lat>,<lng>"       GPS fix obtained
//     "GPS:NO_FIX"            GPS timed out
//     "SMS_OK" / "SMS_FAIL"
//     "CALL_OK" / "CALL_FAIL"
//     "CALL_ANSWERED" / "CALL_NOANSWER"
//     "ALERT_DONE"            sequence complete; ESP idles
//
//   Jetson -> ESP:
//     "STATE:DRIVING"     arm the HR-loss watchdog
//     "STATE:HOLD"        disarm the HR-loss watchdog
//     "ARM"               clear ESP-side abort + alert latches if safe
//     "ACK:<msg>"         echoed to onboard LED
//     "PARKED"            <-- fires the SMS + GSM call sequence ONCE
//
// Wiring (ESP32 DevKit V1 / WROOM-32):
//   AD8232 OUTPUT       -> GPIO34   (ADC1_CH6, input-only -- fine)
//   AD8232 LO+          -> GPIO25
//   AD8232 LO-          -> GPIO26
//   AD8232 3.3V, GND    -> 3V3, GND
//   Mushroom E-stop NC  -> GPIO27 to GND  (internal pull-up, fail-safe)
//   GO button (NO)      -> GPIO14 to GND  (internal pull-up)
//   Status LED          -> GPIO2   (built-in on most dev boards)
//   SIM800L modem TX    -> GPIO32  (ESP RX)
//   SIM800L modem RX    -> GPIO4   (ESP TX)        (level-shifted!)
//   SIM800L VCC         -> 4.0-4.2 V (NOT 5 V, NOT 3.3 V) with >= 2 A burst
//   SIM800L GND         -> common GND with ESP
//   GPS module TX       -> GPIO16  (ESP RX)
//   GPS module RX       -> GPIO17  (ESP TX)
//   GPS VCC, GND        -> 3V3, GND
//
// Build: Arduino IDE 2.x + ESP32 board package by Espressif, target
//        "ESP32 Dev Module".  Library: TinyGPS++ by Mikal Hart
//        (install from the IDE Library Manager).
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// ---------- WiFi / Jetson endpoint -----------------------------------
// Change these for your network.  iPhone hotspot defaults shown.
// Public-repository placeholders. Set these for your own bench network
// before flashing; never commit real credentials or private addresses.
static const char*    WIFI_SSID    = "YOUR_WIFI_SSID";
static const char*    WIFI_PASS    = "YOUR_WIFI_PASSWORD";
static const char*    JETSON_IP    = "192.168.1.100";
static const uint16_t JETSON_PORT  = 5005;          // matches wifi_port in
                                                    //   trigger_bridge_node

// ---------- pins ------------------------------------------------------
static const int PIN_ECG_OUT  = 34;
static const int PIN_LO_PLUS  = 25;
static const int PIN_LO_MINUS = 26;
static const int PIN_ESTOP    = 27;
static const int PIN_GO       = 14;
static const int PIN_LED      = 2;

// SIM800 cellular modem (UART1) -- pin numbers picked so they DON'T
// collide with AD8232 / estop / GO from the existing wiring.
static const int PIN_SIM800_RX = 32;   // ESP RX <- SIM800 TX
static const int PIN_SIM800_TX = 4;    // ESP TX -> SIM800 RX (3.3 V logic; ok)
static const uint32_t SIM800_BAUD = 9600;

// GPS module (UART2) -- TinyGPS++ on UART2.  Standard ESP32 UART2 pins.
static const int PIN_GPS_RX    = 16;   // ESP RX <- GPS TX
static const int PIN_GPS_TX    = 17;   // ESP TX -> GPS RX (most modules ignore)
static const uint32_t GPS_BAUD = 9600;

// Phone number that receives the alert SMS + voice call.
// E.164 destination for alerts. Replace locally before flashing.
static const char* ALERT_PHONE = "+000000000000";

// ---------- timing constants -----------------------------------------
static const uint32_t HB_PERIOD_MS      = 200;   // 5 Hz heartbeat
static const uint32_t ECG_SAMPLE_MS     = 4;     // 250 Hz sampling
static const uint32_t HR_LOST_MS        = 1500;  // no R-peak this long while
                                                 //   DRIVING -> abort
static const uint32_t LEADS_OFF_MS      = 200;   // leads detached this long
                                                 //   while DRIVING -> abort
static const uint32_t REFRACTORY_MS     = 250;   // min gap between peaks
                                                 //   (240 BPM hard ceiling)
static const uint32_t DEBOUNCE_MS       = 30;    // for E-stop and GO
static const uint32_t LED_ACK_MS        = 120;   // LED blink on ACK
static const uint32_t WIFI_RETRY_MS     = 500;
static const uint32_t TCP_RETRY_MS      = 2000;

// ---------- ECG threshold tuning -------------------------------------
//   AD8232 OUTPUT idles around mid-rail (~1.65 V on a 3.3 V supply ->
//   ~2048 on the 12-bit ADC).  R-peaks swing well above baseline.  We
//   maintain a slow EMA as the dynamic baseline and call a peak
//   whenever the signal exceeds baseline + THRESH_DELTA, gated by the
//   refractory period.
static const int   ADC_RES_BITS    = 12;
static const int   ADC_MAX         = (1 << ADC_RES_BITS) - 1;
static const float BASELINE_ALPHA  = 0.005f;
static const int   THRESH_DELTA    = 350;       // counts above baseline
                                                //   tune on the bench

// ---------- auto-go on stable heartbeat ------------------------------
// Number of valid R-peaks (gated by REFRACTORY_MS) before we send "go".
// 3 is enough to reject single-sample noise spikes without forcing the
// driver to wait several seconds.  Set to 0 to disable auto-go and
// require the manual GO button.
static const int   BEATS_TO_AUTO_GO = 3;

// ---------- abort reasons --------------------------------------------
enum AbortReason : uint8_t {
  ABORT_NONE = 0,
  ABORT_ESTOP,
  ABORT_HR_LOST,
};

// ---------- state ----------------------------------------------------
static WiFiClient tcp;
static uint32_t   t_last_tcp_attempt = 0;

static bool         jetson_driving = false;   // last STATE:* from Jetson
static bool         abort_latched  = false;
static AbortReason  abort_reason   = ABORT_NONE;

static uint32_t  t_last_hb_ms       = 0;
static uint32_t  t_last_ecg_ms      = 0;
static uint32_t  t_last_peak_ms     = 0;
static uint32_t  t_last_leads_ok_ms = 0;
static uint32_t  t_last_bpm_ms      = 0;
static uint32_t  t_led_off_ms       = 0;

static float     ecg_baseline      = 2048.0f;
static int       beat_count_window = 0;
static uint32_t  beat_window_t0    = 0;
static int       confirmed_beats   = 0;     // counts toward auto-go
static bool      auto_go_sent      = false; // one-shot per arm cycle

// E-stop / GO debounce
static int       estop_state_raw    = HIGH;
static int       estop_state_stable = HIGH;
static uint32_t  estop_t_change     = 0;
static int       go_state_raw       = HIGH;
static int       go_state_stable    = HIGH;
static uint32_t  go_t_change        = 0;
static bool      go_already_sent    = false;

// TCP RX line buffer
static char      rx_buf[64];
static size_t    rx_len = 0;

// ---------- SIM800 + GPS state ---------------------------------------
// UART1 -> SIM800, UART2 -> GPS.  Both spun up in setup().  We never
// share Serial0 (USB-debug) with either device, so the debug print
// stream is always available regardless of what the modem is doing.
static HardwareSerial sim800(1);
static HardwareSerial gpsSerial(2);
static TinyGPSPlus    gps;

// One-shot latch.  Set to true after the first PARKED message fires
// the alert sequence; reset on ARM / TCP reconnect so the next demo
// run can fire its own alert.
static bool   alert_done           = false;
static String alert_gps_location   = "GPS not fixed";
// Timing-out PARKED messages received while a previous sequence is
// still running is the safer policy than re-entering the function.
static bool   alert_in_progress    = false;

// ---------- helpers --------------------------------------------------
static inline void led_blink(uint32_t now_ms) {
  digitalWrite(PIN_LED, HIGH);
  t_led_off_ms = now_ms + LED_ACK_MS;
}

static inline void led_tick(uint32_t now_ms) {
  if (t_led_off_ms != 0 && (int32_t)(now_ms - t_led_off_ms) >= 0) {
    digitalWrite(PIN_LED, LOW);
    t_led_off_ms = 0;
  }
}

static void send_line(const char* s) {
  // Always echo to USB serial for debugging, even if TCP is down.
  Serial.print("[tx] "); Serial.println(s);
  if (tcp.connected()) {
    tcp.print(s);
    tcp.print('\n');
  }
}

static bool leads_off() {
  // AD8232 drives LO+/LO- HIGH when the corresponding electrode is not
  // making good contact.  Either one being high means we cannot trust
  // the ECG signal -> treat as "no driver".
  return (digitalRead(PIN_LO_PLUS)  == HIGH ||
          digitalRead(PIN_LO_MINUS) == HIGH);
}

// =====================================================================
//  SIM800 / GPS GSM helpers
//  Ported from the standalone reference sketch with two changes:
//    1.  Every progress / failure event is also forwarded to the Jetson
//        via send_line() so the launch logs reflect what's happening.
//    2.  The phone number is the module-level constant ALERT_PHONE
//        instead of a local in sendSMS().
// =====================================================================

// Drain SIM800 RX for up to timeoutMs, returning whatever it sent.
// Also echoes every byte to USB Serial so the bench operator can watch
// the modem in real-time.  Blocking, but only used inside the alert
// sequence where we're committed.
static String sim_read_resp(unsigned long timeoutMs) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (sim800.available()) {
      char c = (char)sim800.read();
      resp += c;
      Serial.write(c);
    }
    delay(1);
  }
  return resp;
}

// Send an AT command, log it to USB, wait waitMs for a reply, return
// the response text.
static String sim_send_cmd_get_resp(const char* cmd,
                                    unsigned long waitMs = 3000) {
  while (sim800.available()) sim800.read();
  Serial.print("\n[sim] CMD: "); Serial.println(cmd);
  sim800.println(cmd);
  return sim_read_resp(waitMs);
}

// Send + check that the response contains `mustContain` (default "OK").
// Pass `mustContain=nullptr` to ignore the response and always return
// true.
static bool sim_send_cmd(const char* cmd,
                         unsigned long waitMs = 3000,
                         const char* mustContain = "OK") {
  String resp = sim_send_cmd_get_resp(cmd, waitMs);
  if (mustContain == nullptr) return true;
  return resp.indexOf(mustContain) != -1;
}

// Bounce a couple of ATs to see if the modem is alive at all.
static bool sim_modem_alive() {
  for (int i = 0; i < 3; i++) {
    if (sim_send_cmd("AT", 2000, "OK")) return true;
    delay(1000);
  }
  return false;
}

// Pre-flight: modem alive, SIM unlocked, network registered.
static bool sim_modem_ready() {
  if (!sim_modem_alive()) {
    Serial.println("[sim] modem not responding to AT");
    return false;
  }
  String simResp = sim_send_cmd_get_resp("AT+CPIN?", 3000);
  if (simResp.indexOf("READY") == -1) {
    Serial.println("[sim] SIM not ready");
    return false;
  }
  String regResp = sim_send_cmd_get_resp("AT+CREG?", 3000);
  if (regResp.indexOf("+CREG: 0,1") == -1 &&
      regResp.indexOf("+CREG: 0,5") == -1) {
    Serial.println("[sim] network not registered");
    return false;
  }
  return true;
}

static bool sim_send_sms(const String& message) {
  while (sim800.available()) sim800.read();
  if (!sim_modem_ready())                          return false;
  if (!sim_send_cmd("AT+CMEE=2", 2000, "OK"))      return false;
  if (!sim_send_cmd("AT+CMGF=1", 2000, "OK"))      return false;

  Serial.println("\n[sim] CMD: AT+CMGS");
  sim800.print("AT+CMGS=\"");
  sim800.print(ALERT_PHONE);
  sim800.print("\"\r");

  String promptResp = sim_read_resp(5000);
  if (promptResp.indexOf('>') == -1) {
    Serial.println("\n[sim] no > prompt for SMS");
    return false;
  }
  sim800.print(message);
  delay(300);
  sim800.write(26);   // Ctrl-Z to commit the message

  String finalResp = sim_read_resp(20000);
  Serial.println("\n[sim] final SMS response:");
  Serial.println(finalResp);

  if (finalResp.indexOf("+CMGS:") != -1 &&
      finalResp.indexOf("OK")     != -1) {
    Serial.println("[sim] SMS sent");
    return true;
  }
  Serial.println("[sim] SMS failed");
  return false;
}

static bool sim_make_call() {
  while (sim800.available()) sim800.read();
  if (!sim_modem_ready()) return false;

  Serial.println("\n[sim] CMD: ATD");
  sim800.print("ATD");
  sim800.print(ALERT_PHONE);
  sim800.println(";");

  String resp = sim_read_resp(10000);
  Serial.println("\n[sim] call response:");
  Serial.println(resp);

  if (resp.indexOf("OK")      != -1 ||
      resp.indexOf("CONNECT") != -1) {
    Serial.println("[sim] call command accepted");
    return true;
  }
  Serial.println("[sim] call failed");
  return false;
}

// Poll AT+CLCC; "+CLCC: ...,0,0,0,..." means call state 0 (active).
static bool sim_call_answered() {
  String resp = sim_send_cmd_get_resp("AT+CLCC", 3000);
  return (resp.indexOf("+CLCC:")    != -1 &&
          resp.indexOf(",0,0,0,0") != -1);
}

static bool sim_wait_for_answer(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (sim_call_answered()) {
      Serial.println("\n[sim] call answered");
      return true;
    }
    delay(500);
  }
  Serial.println("\n[sim] no answer detected");
  return false;
}

static void sim_play_beeps() {
  sim_send_cmd("AT+DTAM=1", 3000, nullptr);
  sim_send_cmd("AT+CLDTMF=5,\"1,1,1,1,1,1,1,1\",100", 12000, nullptr);
}

static void sim_hang_up() {
  while (sim800.available()) sim800.read();
  Serial.println("\n[sim] CMD: ATH");
  sim800.println("ATH");
  sim_read_resp(3000);
}

// =====================================================================
//  Emergency sequence (fired once per arm-cycle on Jetson "PARKED").
//  Sends progress events back to the Jetson so the launch log shows
//  the GPS / SMS / call status without watching the SIM800 USB trace.
// =====================================================================
static void run_emergency_sequence() {
  if (alert_done) {
    Serial.println("[alert] PARKED ignored -- alert already completed "
                   "this arm-cycle");
    return;
  }
  if (alert_in_progress) {
    Serial.println("[alert] PARKED ignored -- sequence already running");
    return;
  }
  alert_in_progress = true;

  Serial.println("\n[alert] PARKED received -- starting SMS + call");
  send_line("ALERT_START");

  // ----- 1. GPS fix (up to 20 s) ------------------------------------
  unsigned long gpsStart = millis();
  bool gotFix = false;
  while (millis() - gpsStart < 20000) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      String lat = String(gps.location.lat(), 6);
      String lng = String(gps.location.lng(), 6);
      alert_gps_location = lat + "," + lng;
      gotFix = true;
      break;
    }
  }
  if (gotFix) {
    Serial.print("[alert] location: "); Serial.println(alert_gps_location);
    String evt = "GPS:" + alert_gps_location;
    send_line(evt.c_str());
  } else {
    alert_gps_location = "GPS not fixed";
    Serial.println("[alert] GPS fix not acquired");
    send_line("GPS:NO_FIX");
  }

  // ----- 2. SMS -----------------------------------------------------
  String smsText = "HELP Location: " + alert_gps_location;
  bool smsOk = sim_send_sms(smsText);
  send_line(smsOk ? "SMS_OK" : "SMS_FAIL");
  if (smsOk) delay(3000);

  // ----- 3. Voice call ---------------------------------------------
  bool callOk = sim_make_call();
  send_line(callOk ? "CALL_OK" : "CALL_FAIL");
  if (callOk) {
    bool answered = sim_wait_for_answer(30000);
    send_line(answered ? "CALL_ANSWERED" : "CALL_NOANSWER");
    if (answered) {
      sim_play_beeps();
      delay(5000);
    }
    sim_hang_up();
  }

  send_line("ALERT_DONE");
  Serial.println("[alert] sequence finished");
  alert_done        = true;
  alert_in_progress = false;
}

// ---------- WiFi / TCP plumbing --------------------------------------
static void wifi_maintain() {
  // Begin once; let the WiFi library handle reconnects in the background.
  static bool wifi_begun = false;
  if (!wifi_begun) {
    Serial.print("WiFi connecting to "); Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifi_begun = true;
  }
  // No blocking wait -- the rest of loop() still runs and serial debug
  // keeps flowing while WiFi negotiates in the background.
}

static bool tcp_connected() {
  return tcp.connected();
}

static void tcp_maintain(uint32_t now) {
  if (tcp.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if ((now - t_last_tcp_attempt) < TCP_RETRY_MS) return;
  t_last_tcp_attempt = now;

  Serial.print("TCP connect "); Serial.print(JETSON_IP);
  Serial.print(":"); Serial.print(JETSON_PORT); Serial.print(" ... ");
  if (tcp.connect(JETSON_IP, JETSON_PORT)) {
    Serial.println("ok");
    // Reset per-arm-cycle one-shots so a fresh stable heartbeat will
    // re-fire "go" after the bridge restarts.  Also wipe the alert
    // latch so the next PARKED fires a fresh SMS/call sequence.
    auto_go_sent    = false;
    go_already_sent = false;
    confirmed_beats = 0;
    alert_done      = false;
    rx_len = 0;
    send_line("READY");
  } else {
    Serial.println("failed");
  }
}

// ---------- abort logic ----------------------------------------------
static void trip_abort(AbortReason r) {
  if (abort_latched) return;
  abort_latched = true;
  abort_reason  = r;
  switch (r) {
    case ABORT_ESTOP:    send_line("ESTOP"); break;
    case ABORT_HR_LOST:  send_line("ABORT:HR_LOST"); break;
    default:             send_line("ABORT:UNKNOWN"); break;
  }
  digitalWrite(PIN_LED, HIGH);   // solid LED while latched
}

static void clear_abort_if_safe() {
  // Only clear if the operator has actually released the mushroom AND
  // the leads are attached.  This is the one place the latch is dropped;
  // the Jetson asks for it via "ARM\n".
  if (estop_state_stable == HIGH) {
    send_line("REARM_REJECTED:ESTOP_HELD");
    return;
  }
  if (leads_off()) {
    send_line("REARM_REJECTED:LEADS_OFF");
    return;
  }
  abort_latched      = false;
  abort_reason       = ABORT_NONE;
  t_last_peak_ms     = millis();
  t_last_leads_ok_ms = millis();
  // re-arm cycle: allow auto-go to fire again on next stable heartbeat,
  // and let a future PARKED fire its own alert sequence.
  auto_go_sent       = false;
  confirmed_beats    = 0;
  alert_done         = false;
  digitalWrite(PIN_LED, LOW);
  send_line("READY");
}

// ---------- TCP RX ---------------------------------------------------
static void handle_rx_line(const char* line) {
  Serial.print("[rx] "); Serial.println(line);
  if (strcmp(line, "STATE:DRIVING") == 0) {
    jetson_driving = true;
    uint32_t now = millis();
    t_last_peak_ms     = now;   // grace window so the watchdog doesn't
    t_last_leads_ok_ms = now;   //   trip on stale timestamps
  } else if (strcmp(line, "STATE:HOLD") == 0) {
    jetson_driving = false;
  } else if (strcmp(line, "ARM") == 0) {
    clear_abort_if_safe();
  } else if (strcmp(line, "PARKED") == 0) {
    // Fires the SMS + GSM call sequence ONCE per arm-cycle.  See
    // run_emergency_sequence() above.  Blocks the rest of loop() for
    // ~60-90 s in the worst case (GPS 20 s + SMS 20 s + call answer
    // wait 30 s).  Jetson bridge link timeout is 600 s so we're safe;
    // also the sequence itself emits "ALERT_START" / "SMS_OK" / etc.
    // which refresh the bridge's link watchdog.
    run_emergency_sequence();
  } else if (strncmp(line, "ACK:", 4) == 0) {
    led_blink(millis());
  }
  // anything else is silently ignored
}

static void poll_tcp_rx() {
  if (!tcp.connected()) return;
  while (tcp.available()) {
    char c = (char)tcp.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rx_buf[rx_len] = '\0';
      if (rx_len > 0) handle_rx_line(rx_buf);
      rx_len = 0;
    } else if (rx_len < sizeof(rx_buf) - 1) {
      rx_buf[rx_len++] = c;
    } else {
      rx_len = 0;   // overflow, resync
    }
  }
}

// ---------- button polling -------------------------------------------
static void poll_buttons(uint32_t now) {
  // E-stop (NC, pulled up) -- LOW = armed, HIGH = pressed/broken
  int estop_raw = digitalRead(PIN_ESTOP);
  if (estop_raw != estop_state_raw) {
    estop_state_raw = estop_raw;
    estop_t_change  = now;
  }
  if ((now - estop_t_change) > DEBOUNCE_MS &&
      estop_state_stable != estop_state_raw) {
    estop_state_stable = estop_state_raw;
    if (estop_state_stable == HIGH) {
      // mushroom pressed (or wire cut) -- fail-safe trips here
      trip_abort(ABORT_ESTOP);
    }
    // we do NOT auto-clear on release; Jetson must send ARM
  }

  // GO button (momentary, NO, pulled up) -- LOW when pressed.  Manual
  // fallback if you want to fire "go" without waiting for a heartbeat.
  int go_raw = digitalRead(PIN_GO);
  if (go_raw != go_state_raw) {
    go_state_raw = go_raw;
    go_t_change  = now;
  }
  if ((now - go_t_change) > DEBOUNCE_MS &&
      go_state_stable != go_state_raw) {
    go_state_stable = go_state_raw;
    if (go_state_stable == LOW && !go_already_sent && !abort_latched) {
      send_line("go");
      go_already_sent = true;
      auto_go_sent    = true;   // suppress redundant auto-go
    } else if (go_state_stable == HIGH) {
      go_already_sent = false;
    }
  }
}

// ---------- ECG sampling ---------------------------------------------
static void poll_ecg(uint32_t now) {
  if ((now - t_last_ecg_ms) < ECG_SAMPLE_MS) return;
  t_last_ecg_ms = now;

  if (leads_off()) {
    // hold baseline frozen and let the watchdog handle it
    if (jetson_driving && !abort_latched &&
        (now - t_last_leads_ok_ms) > LEADS_OFF_MS) {
      trip_abort(ABORT_HR_LOST);
    }
    return;
  }
  t_last_leads_ok_ms = now;

  int sample = analogRead(PIN_ECG_OUT);
  ecg_baseline += BASELINE_ALPHA * (sample - ecg_baseline);

  // peak detect with refractory period
  if (sample > ecg_baseline + THRESH_DELTA &&
      (now - t_last_peak_ms) > REFRACTORY_MS) {
    t_last_peak_ms = now;
    beat_count_window++;
    if (confirmed_beats < BEATS_TO_AUTO_GO) confirmed_beats++;

    // Auto-go: first time we have a stable rhythm AND not aborted AND
    // we haven't already fired "go" this arm-cycle, send it.
    if (BEATS_TO_AUTO_GO > 0 &&
        !auto_go_sent && !abort_latched &&
        confirmed_beats >= BEATS_TO_AUTO_GO) {
      send_line("go");
      auto_go_sent = true;
      Serial.println("--> auto go (heartbeat established)");
    }
  }

  // BPM telemetry once per second
  if ((now - t_last_bpm_ms) > 1000) {
    t_last_bpm_ms = now;
    uint32_t window = now - beat_window_t0;
    if (window >= 4000) {                       // need >= 4 s for stable BPM
      int bpm = (int)(beat_count_window * 60000UL / window);
      char buf[24];
      snprintf(buf, sizeof(buf), "BPM:%d", bpm);
      send_line(buf);
      beat_window_t0    = now;
      beat_count_window = 0;
    } else if (beat_window_t0 == 0) {
      beat_window_t0 = now;
    }
  }

  // HR-lost watchdog -- only enforced while Jetson reports DRIVING
  if (jetson_driving && !abort_latched &&
      (now - t_last_peak_ms) > HR_LOST_MS) {
    trip_abort(ABORT_HR_LOST);
  }
}

// ---------- heartbeat -------------------------------------------------
static void send_heartbeat(uint32_t now) {
  if ((now - t_last_hb_ms) < HB_PERIOD_MS) return;
  t_last_hb_ms = now;
  send_line("HB");
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  pinMode(PIN_LO_PLUS,  INPUT);
  pinMode(PIN_LO_MINUS, INPUT);
  pinMode(PIN_ESTOP,    INPUT_PULLUP);
  pinMode(PIN_GO,       INPUT_PULLUP);
  pinMode(PIN_LED,      OUTPUT);
  digitalWrite(PIN_LED, LOW);

  analogReadResolution(ADC_RES_BITS);
  // 11 dB attenuation gives ~0-3.3 V full-scale on the ADC, matching the
  // AD8232 rail.  Without this the ADC clips around 1.1 V.
  analogSetPinAttenuation(PIN_ECG_OUT, ADC_11db);

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("trigger_esp boot");

  // Initial state: if mushroom is already pressed at boot, latch
  // immediately so we don't pretend to be ready.
  delay(20);
  if (digitalRead(PIN_ESTOP) == HIGH) {
    estop_state_raw = estop_state_stable = HIGH;
    abort_latched = true;
    abort_reason  = ABORT_ESTOP;
    digitalWrite(PIN_LED, HIGH);
    Serial.println("WARN: mushroom pressed at boot -- starting latched");
  }

  uint32_t now = millis();
  t_last_peak_ms     = now;
  t_last_leads_ok_ms = now;
  t_last_bpm_ms      = now;
  beat_window_t0     = now;

  // SIM800 + GPS UART bring-up.  The 5 s delay lets SIM800 finish its
  // boot-up babble before we start firing AT commands at it.  GPS is
  // free-running and just needs us to drain its NMEA stream in loop().
  sim800.begin(SIM800_BAUD, SERIAL_8N1, PIN_SIM800_RX, PIN_SIM800_TX);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("SIM800 + GPS UARTs up, waiting 5 s for modem boot...");
  delay(5000);
  // Best-effort prime: kick the modem with a few inert ATs so it's
  // awake and registered by the time the demo actually runs.  We don't
  // gate on the responses here -- the alert sequence does its own
  // pre-flight via sim_modem_ready().
  sim_send_cmd("AT",       2000, nullptr);
  sim_send_cmd("AT+CMEE=2",2000, nullptr);
  sim_send_cmd("AT+CPIN?", 3000, nullptr);
  sim_send_cmd("AT+CREG?", 3000, nullptr);
  sim_send_cmd("AT+CSQ",   3000, nullptr);
  Serial.println("SIM800 prime complete.");

  wifi_maintain();
  // READY is sent by tcp_maintain on first successful TCP connect, not
  // here -- bridge would never see it before the socket exists.
}

void loop() {
  uint32_t now = millis();
  wifi_maintain();
  tcp_maintain(now);
  poll_tcp_rx();
  poll_buttons(now);
  poll_ecg(now);
  // Continuously drain the GPS NMEA stream so the module has a current
  // fix the instant we need one.  This is cheap and non-blocking; the
  // emergency sequence will still wait up to 20 s for a first fix on
  // a cold start.
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (tcp.connected()) {
    send_heartbeat(now);
  }
  led_tick(now);
}
