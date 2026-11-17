// =====================================================================
// motor_esp.ino  --  drive ESP32  (DC drive H-bridge + SERVO steering)
//
// Mirror of esp2_motors_only/esp2_motors_only.ino — keep the two copies
// identical.  This is the canonical location under the firmware/ tree.
//
// Wire protocol with jetson_driver/motor_bridge_node.py:
//
//   T<int>          drive throttle  (-255..255)   T120  / T-180
//   S<int>          steering        (-255..255)   S80   / S-120  (+ = right)
//   TS <d> <s>      combined drive + steer        TS 150 -90
//   B               hard brake (drive brake, servo centre)
//   P               park (coast drive, centre servo, emit EVT:PARKED)
//   H               heartbeat ping (refreshes safety watchdog)
//   STATUS          report current motor state
//
// All motion is COMMAND-DRIVEN: no time-based maneuvers in firmware.  The
// car moves only while the Jetson is publishing fresh commands; if the
// serial stream stalls, the safety watchdog brakes and centres the servo.
//
// See esp2_motors_only/esp2_motors_only.ino for the full wiring diagram
// and tuning notes.
// =====================================================================

#include <Arduino.h>

static const int DRIVE_IN1  = 25;
static const int DRIVE_IN2  = 26;
static const int DRIVE_PWM  = 27;
static const int STEER_PWM  = 13;

static const int DRIVE_LEDC_CH    = 0;
static const int DRIVE_LEDC_FREQ  = 20000;
static const int DRIVE_LEDC_BITS  = 8;
static const int DRIVE_PWM_MAX    = 255;

static const int SERVO_LEDC_CH    = 1;
static const int SERVO_LEDC_FREQ  = 50;
static const int SERVO_LEDC_BITS  = 16;
static const uint32_t SERVO_PERIOD_US = 20000UL;
static const uint32_t SERVO_FULL_DUTY = (1UL << SERVO_LEDC_BITS) - 1UL;

static const int SERVO_MIN_DEG    = 45;
static const int SERVO_CENTER_DEG = 90;
static const int SERVO_MAX_DEG    = 135;

static uint32_t servo_deg_to_duty(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  uint32_t pulse_us = 1000UL + (uint32_t)deg * 1000UL / 180UL;
  return (uint32_t)((uint64_t)pulse_us * SERVO_FULL_DUTY / SERVO_PERIOD_US);
}

static const unsigned long HEARTBEAT_TIMEOUT_MS = 5000;

static unsigned long lastCommandTime = 0;
static int currentDrive = 0;
static int currentSteer = 0;
static int currentServoDeg = SERVO_CENTER_DEG;

static void setDrive(int value) {
  value = constrain(value, -DRIVE_PWM_MAX, DRIVE_PWM_MAX);
  if (value > 0) {
    digitalWrite(DRIVE_IN1, HIGH);
    digitalWrite(DRIVE_IN2, LOW);
    ledcWrite(DRIVE_LEDC_CH, value);
  } else if (value < 0) {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, HIGH);
    ledcWrite(DRIVE_LEDC_CH, -value);
  } else {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, LOW);
    ledcWrite(DRIVE_LEDC_CH, 0);
  }
  currentDrive = value;
}

static void setSteer(int value) {
  value = constrain(value, -255, 255);
  int deg;
  if (value >= 0) {
    deg = SERVO_CENTER_DEG +
          (int)((long)value * (SERVO_MAX_DEG - SERVO_CENTER_DEG) / 255L);
  } else {
    deg = SERVO_CENTER_DEG +
          (int)((long)value * (SERVO_CENTER_DEG - SERVO_MIN_DEG) / 255L);
  }
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  ledcWrite(SERVO_LEDC_CH, servo_deg_to_duty(deg));
  currentSteer    = value;
  currentServoDeg = deg;
}

static void brake() {
  digitalWrite(DRIVE_IN1, HIGH);
  digitalWrite(DRIVE_IN2, HIGH);
  ledcWrite(DRIVE_LEDC_CH, DRIVE_PWM_MAX);
  ledcWrite(SERVO_LEDC_CH, servo_deg_to_duty(SERVO_CENTER_DEG));
  currentDrive    = 0;
  currentSteer    = 0;
  currentServoDeg = SERVO_CENTER_DEG;
}

static void park() {
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);
  ledcWrite(DRIVE_LEDC_CH, 0);
  ledcWrite(SERVO_LEDC_CH, servo_deg_to_duty(SERVO_CENTER_DEG));
  currentDrive    = 0;
  currentSteer    = 0;
  currentServoDeg = SERVO_CENTER_DEG;
}

static void emergencyStop(const char* reason) {
  brake();
  delay(150);
  setDrive(0);
  Serial.print("EVT:ESTOP ");
  Serial.println(reason);
}

static void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  lastCommandTime = millis();

  if (cmd.equalsIgnoreCase("STATUS")) {
    Serial.print("EVT:STATUS T=");
    Serial.print(currentDrive);
    Serial.print(" S=");
    Serial.print(currentSteer);
    Serial.print(" A=");
    Serial.println(currentServoDeg);
    return;
  }

  if (cmd.length() >= 2 &&
      (cmd.charAt(0) == 'T' || cmd.charAt(0) == 't') &&
      (cmd.charAt(1) == 'S' || cmd.charAt(1) == 's')) {
    int sp1 = cmd.indexOf(' ');
    if (sp1 > 0) {
      int sp2 = cmd.indexOf(' ', sp1 + 1);
      if (sp2 > 0) {
        int d = cmd.substring(sp1 + 1, sp2).toInt();
        int s = cmd.substring(sp2 + 1).toInt();
        setDrive(d);
        setSteer(s);
        return;
      }
    }
  }

  char first = toupper(cmd.charAt(0));
  switch (first) {
    case 'B':
      brake();
      Serial.println("EVT:BRAKE");
      return;
    case 'P':
      park();
      Serial.println("EVT:PARKED");
      return;
    case 'H':
      Serial.println("EVT:HB");
      return;
    case 'T':
      setDrive(cmd.substring(1).toInt());
      return;
    case 'S':
      setSteer(cmd.substring(1).toInt());
      return;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);

  ledcSetup(DRIVE_LEDC_CH, DRIVE_LEDC_FREQ, DRIVE_LEDC_BITS);
  ledcAttachPin(DRIVE_PWM, DRIVE_LEDC_CH);
  ledcWrite(DRIVE_LEDC_CH, 0);

  ledcSetup(SERVO_LEDC_CH, SERVO_LEDC_FREQ, SERVO_LEDC_BITS);
  ledcAttachPin(STEER_PWM, SERVO_LEDC_CH);
  ledcWrite(SERVO_LEDC_CH, servo_deg_to_duty(SERVO_CENTER_DEG));

  currentDrive    = 0;
  currentSteer    = 0;
  currentServoDeg = SERVO_CENTER_DEG;

  lastCommandTime = millis();
  Serial.println("EVT:READY");
}

void loop() {
  static String inbuf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) {
        parseCommand(inbuf);
        inbuf = "";
      }
    } else {
      inbuf += c;
      if (inbuf.length() > 96) inbuf = "";
    }
  }

  if (millis() - lastCommandTime > HEARTBEAT_TIMEOUT_MS) {
    if (currentDrive != 0) {
      emergencyStop("HB_TIMEOUT");
    }
    lastCommandTime = millis();
  }

  delay(2);
}
