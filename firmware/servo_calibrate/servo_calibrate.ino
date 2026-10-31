// Drives a single servo on GPIO25 from serial commands (115200 baud), for finding a joint's home and limits
// before it goes into the arm.
//
//   1500       go to 1500 us
//   + -        nudge 10 us             ++ --   nudge 50 us
//   c          centre (1500 us)        p       print the position
//   speed N    ramp speed in us/s (default 2000, 0 = no ramp)
//   wide       allow 500-2500 us       safe    back to 1000-2000 us (default)
//   off        limp                    on      pulses again at the current target
//
// The servo starts limp, so a reset never moves it. The first position you type is sent straight away; after that
// every move ramps.

const int SERVO_PIN = 25;
const int PWM_FREQ_HZ = 50;
const int PWM_RES_BITS = 16;
const uint32_t PERIOD_US = 1000000UL / PWM_FREQ_HZ;
const uint32_t FRAME_MS = 20;
const int CENTRE_US = 1500;

int lo = 1000, hi = 2000;
int target_us = CENTRE_US, current_us = CENTRE_US;
int speed_us_per_s = 2000;
bool driving = false;

// A duty of 0 sends no pulses at all, which leaves the servo limp.
void write_pulse(int us) {
  uint32_t duty = (uint64_t)us * ((1UL << PWM_RES_BITS) - 1) / PERIOD_US;
  ledcWrite(SERVO_PIN, driving ? duty : 0);
}

void set_target(int us) {
  int clamped = constrain(us, lo, hi);
  if (clamped != us) Serial.printf("clamped %d -> %d (range %d..%d; 'wide' for 500..2500)\n", us, clamped, lo, hi);
  target_us = clamped;
  if (!driving) {
    // the servo's real position is unknown, so jump instead of ramping from a guess
    driving = true;
    current_us = target_us;
    write_pulse(current_us);
    Serial.printf("pulses on, at %d us\n", current_us);
    return;
  }
  Serial.printf("target %d us\n", target_us);
}

void handle_command(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "+") set_target(target_us + 10);
  else if (cmd == "-") set_target(target_us - 10);
  else if (cmd == "++") set_target(target_us + 50);
  else if (cmd == "--") set_target(target_us - 50);
  else if (cmd == "c") set_target(CENTRE_US);
  else if (cmd == "p") {
    Serial.printf("at %d us, target %d us, range %d..%d, speed %d us/s, %s\n", current_us, target_us, lo, hi,
                  speed_us_per_s, driving ? "on" : "off (limp)");
  } else if (cmd.startsWith("speed")) {
    speed_us_per_s = max(0, (int)cmd.substring(5).toInt());
    Serial.printf("speed %d us/s\n", speed_us_per_s);
  } else if (cmd == "wide") {
    lo = 500;
    hi = 2500;
    Serial.println("range 500..2500: go slowly near the ends");
  } else if (cmd == "safe") {
    lo = 1000;
    hi = 2000;
    set_target(target_us);
  } else if (cmd == "off") {
    driving = false;
    write_pulse(current_us);
    Serial.println("pulses off (limp)");
  } else if (cmd == "on") {
    set_target(target_us);
  } else if (isDigit(cmd[0])) {
    set_target(cmd.toInt());
  } else {
    Serial.println("? commands: <us>  + - ++ --  c  p  speed N  wide safe  off on");
  }
}

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  write_pulse(current_us);
  delay(200);
  Serial.println("\nservo_calibrate: GPIO25, limp. Type a pulse in us to start (1000..2000; 'wide' for 500..2500).");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      handle_command(line);
      line = "";
    } else {
      line += ch;
    }
  }

  static uint32_t last_frame_ms = 0;
  if (driving && current_us != target_us && millis() - last_frame_ms >= FRAME_MS) {
    last_frame_ms = millis();
    int step = speed_us_per_s == 0 ? abs(target_us - current_us) : max(1, (int)(speed_us_per_s * FRAME_MS / 1000));
    current_us += constrain(target_us - current_us, -step, step);
    write_pulse(current_us);
    if (current_us == target_us) Serial.printf("at %d us\n", current_us);
  }
}
