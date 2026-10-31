// SAM firmware: drives the five servos from serial commands (115200 baud, one command per line).
// Joint keys: y yaw, s shoulder, e elbow, w wrist, g gripper.
//
//   s 1500          move a joint to 1500 us         s+ s- s++ s--   nudge by 10 / 50 us
//   p               print joints, limits and saved poses
//   lim s 600 2100  set a joint's limits            home s 600      set a joint's home
//   home            move every joint home
//   save 3, go 3    store / recall pose slot 3 (0-19)                clear   delete all poses
//   play, loop      run the saved poses in order, once or repeating  stop    end playback
//   speed 600       top joint speed in us/s (50-3000)                dwell 500   pause at each pose, ms
//   off, off s      all joints limp / one joint limp                 on      power all joints at their targets
//
// Limits, homes and poses are kept in flash, so they survive a reset.
//
// All joints start limp. The firmware can't know where a limp joint is, so the first command to it jumps
// straight to the target instead of ramping. Move the arm close to that position by hand first.

#include <Preferences.h>

const int JOINT_COUNT = 5;
const char JOINT_KEYS[JOINT_COUNT] = {'y', 's', 'e', 'w', 'g'};
const int PINS[JOINT_COUNT] = {27, 26, 25, 33, 32};
const int YAW = 0, GRIP = 4;

const int PWM_FREQ_HZ = 50;
const int PWM_RES_BITS = 16;
const uint32_t PERIOD_US = 1000000UL / PWM_FREQ_HZ;
const uint32_t FRAME_MS = 20;
const int POSE_SLOTS = 20;

// The yaw and gripper stop getting pulses shortly after each move. The yaw carries no gravity load and its gearbox
// holds the turret still, so this stops it hunting back and forth in the gear backlash. The gripper's gearbox
// friction keeps hold of an object, so a squeeze or a jam only stalls the servo for GRIP_HOLD_MS.
const uint32_t REST_MS = 400;
const uint32_t GRIP_HOLD_MS = 1500;

int limit_lo[JOINT_COUNT], limit_hi[JOINT_COUNT], home_us[JOINT_COUNT];
float current_us[JOINT_COUNT], start_us[JOINT_COUNT];
int target_us[JOINT_COUNT];
bool powered[JOINT_COUNT];   // false = limp
bool resting[JOINT_COUNT];   // powered, but pulses paused (yaw and gripper)
uint32_t rest_at_ms[JOINT_COUNT] = {0};

int poses[POSE_SLOTS][JOINT_COUNT];
bool pose_used[POSE_SLOTS];

int speed_us_per_s = 600, dwell_ms = 500;
bool moving = false;
uint32_t move_start_ms = 0, move_ms = 0;
int play_index = -1;   // -1 when not playing
bool looping = false;
uint32_t dwell_until_ms = 0;

Preferences prefs;

int joint_of(char key) {
  for (int i = 0; i < JOINT_COUNT; i++)
    if (JOINT_KEYS[i] == key) return i;
  return -1;
}

int rounded(float us) { return (int)(us + 0.5f); }

// A duty of 0 sends no pulses at all, which leaves the servo limp.
void write_pulse(int i) {
  uint32_t duty = 0;
  if (powered[i] && !resting[i])
    duty = (uint64_t)rounded(current_us[i]) * ((1UL << PWM_RES_BITS) - 1) / PERIOD_US;
  ledcWrite(PINS[i], duty);
}

void save_settings() {
  prefs.putBytes("lo", limit_lo, sizeof limit_lo);
  prefs.putBytes("hi", limit_hi, sizeof limit_hi);
  prefs.putBytes("home", home_us, sizeof home_us);
  prefs.putBytes("pose", poses, sizeof poses);
  prefs.putBytes("used", pose_used, sizeof pose_used);
}

// Moves every powered joint from where it is to its target, all arriving at the same time. The motion follows a
// smoothstep curve so it starts and stops gently. Smoothstep peaks at 1.5x the average speed, so the move time is
// picked to keep the joint with the furthest to go at speed_us_per_s.
void start_move() {
  int furthest = 0;
  for (int i = 0; i < JOINT_COUNT; i++) {
    // A resting joint only wakes up if its own target changed, so the yaw stays quiet and the gripper keeps its
    // grip while the other joints move.
    if (resting[i] && target_us[i] != rounded(current_us[i])) {
      resting[i] = false;
      write_pulse(i);
    }
    start_us[i] = current_us[i];
    if (powered[i]) furthest = max(furthest, abs(target_us[i] - (int)current_us[i]));
  }
  move_ms = (uint32_t)furthest * 1500UL / speed_us_per_s;
  move_start_ms = millis();
  moving = true;
  rest_at_ms[YAW] = rest_at_ms[GRIP] = 0;
}

// Powers a limp joint. Its real position is unknown, so it jumps to `us` instead of ramping.
void power_on_at(int i, int us) {
  powered[i] = true;
  current_us[i] = start_us[i] = us;
  write_pulse(i);
}

void set_joint(int i, int us) {
  int clamped = constrain(us, limit_lo[i], limit_hi[i]);
  if (clamped != us)
    Serial.printf("%c: clamped %d -> %d (limits %d..%d)\n", JOINT_KEYS[i], us, clamped, limit_lo[i], limit_hi[i]);
  target_us[i] = clamped;
  if (!powered[i]) {
    power_on_at(i, clamped);
    Serial.printf("%c: pulses on at %d us\n", JOINT_KEYS[i], clamped);
  }
  start_move();
}

void go_to_pose(int k) {
  for (int i = 0; i < JOINT_COUNT; i++) {
    target_us[i] = constrain(poses[k][i], limit_lo[i], limit_hi[i]);
    if (!powered[i]) power_on_at(i, target_us[i]);
  }
  start_move();
  Serial.printf("-> pose %d\n", k);
}

void go_home() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    target_us[i] = home_us[i];
    if (!powered[i]) power_on_at(i, target_us[i]);
  }
  start_move();
  Serial.println("-> home");
}

void stop_playback() {
  if (play_index >= 0) Serial.println("stopped");
  play_index = -1;
  looping = false;
}

int next_pose(int from) {
  for (int k = from; k < POSE_SLOTS; k++)
    if (pose_used[k]) return k;
  return -1;
}

// The control panel parses this output, so keep the format in sync with tools/arm_ui/arm_ui.py.
void print_status() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    const char *state = !powered[i] ? "limp" : resting[i] ? "rest" : "on  ";
    Serial.printf("%c  GPIO%-2d  %s  at %4d  target %4d  limits %4d..%4d  home %4d\n", JOINT_KEYS[i], PINS[i], state,
                  (int)current_us[i], target_us[i], limit_lo[i], limit_hi[i], home_us[i]);
  }
  for (int k = 0; k < POSE_SLOTS; k++) {
    if (!pose_used[k]) continue;
    int *p = poses[k];
    Serial.printf("pose %2d: y %d  s %d  e %d  w %d  g %d\n", k, p[0], p[1], p[2], p[3], p[4]);
  }
  Serial.printf("speed %d us/s, dwell %d ms\n", speed_us_per_s, dwell_ms);
}

int nudge_amount(const String &suffix) {
  if (suffix == "+") return 10;
  if (suffix == "-") return -10;
  if (suffix == "++") return 50;
  if (suffix == "--") return -50;
  return 0;
}

void handle_command(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  if (cmd != "p") stop_playback();   // p only reports, so the panel can poll it during playback

  String word = cmd, arg = "";
  int space = cmd.indexOf(' ');
  if (space > 0) {
    word = cmd.substring(0, space);
    arg = cmd.substring(space + 1);
    arg.trim();
  }
  int j = joint_of(word[0]);
  int nudge = j >= 0 ? nudge_amount(word.substring(1)) : 0;

  if (j >= 0 && word.length() == 1 && arg.length() && isDigit(arg[0])) {
    set_joint(j, arg.toInt());
  } else if (nudge != 0) {
    set_joint(j, (powered[j] ? target_us[j] : home_us[j]) + nudge);
  } else if (word == "p") {
    print_status();
  } else if (word == "stop") {
    // stop_playback() above already did it
  } else if (word == "lim") {
    int k = joint_of(arg[0]);
    int second_space = arg.indexOf(' ', 2);
    if (k < 0 || second_space < 0) {
      Serial.println("lim <y|s|e|w|g> <lo> <hi>");
      return;
    }
    int lo = arg.substring(2, second_space).toInt(), hi = arg.substring(second_space + 1).toInt();
    if (lo < 500 || hi > 2500 || lo >= hi) {
      Serial.println("limits must be 500 <= lo < hi <= 2500");
      return;
    }
    limit_lo[k] = lo;
    limit_hi[k] = hi;
    save_settings();
    Serial.printf("%c limits %d..%d\n", JOINT_KEYS[k], lo, hi);
  } else if (word == "home" && arg.length() >= 3) {
    int k = joint_of(arg[0]);
    if (k < 0) {
      Serial.println("home <y|s|e|w|g> <us>");
      return;
    }
    home_us[k] = constrain((int)arg.substring(2).toInt(), limit_lo[k], limit_hi[k]);
    save_settings();
    Serial.printf("%c home %d\n", JOINT_KEYS[k], home_us[k]);
  } else if (word == "home") {
    go_home();
  } else if (word == "save") {
    int k = arg.toInt();
    if (!arg.length() || k < 0 || k >= POSE_SLOTS) {
      Serial.println("save 0..19");
      return;
    }
    for (int i = 0; i < JOINT_COUNT; i++) poses[k][i] = powered[i] ? target_us[i] : home_us[i];
    pose_used[k] = true;
    save_settings();
    Serial.printf("pose %d saved\n", k);
  } else if (word == "clear") {
    for (int k = 0; k < POSE_SLOTS; k++) pose_used[k] = false;
    save_settings();
    Serial.println("poses cleared");
  } else if (word == "go") {
    int k = arg.toInt();
    if (k >= 0 && k < POSE_SLOTS && pose_used[k]) go_to_pose(k);
    else Serial.println("no such pose");
  } else if (word == "play" || word == "loop") {
    play_index = next_pose(0);
    if (play_index < 0) {
      Serial.println("no poses saved");
      return;
    }
    looping = word == "loop";
    dwell_until_ms = 0;
    go_to_pose(play_index);
  } else if (word == "speed") {
    if (arg.length()) speed_us_per_s = constrain((int)arg.toInt(), 50, 3000);
    Serial.printf("speed %d us/s\n", speed_us_per_s);
  } else if (word == "dwell") {
    dwell_ms = max(0, (int)arg.toInt());
    Serial.printf("dwell %d ms\n", dwell_ms);
  } else if (word == "off" && arg.length()) {
    int k = joint_of(arg[0]);
    if (k >= 0) {
      powered[k] = resting[k] = false;
      write_pulse(k);
      Serial.printf("%c limp\n", JOINT_KEYS[k]);
    }
  } else if (word == "off") {
    moving = false;
    for (int i = 0; i < JOINT_COUNT; i++) {
      powered[i] = resting[i] = false;
      write_pulse(i);
    }
    Serial.println("all limp");
  } else if (word == "on") {
    for (int i = 0; i < JOINT_COUNT; i++) {
      powered[i] = true;
      resting[i] = false;
      current_us[i] = target_us[i];
      write_pulse(i);
    }
    Serial.println("all on at their targets");
  } else {
    Serial.println("? y/s/e/w/g <us> | s+ s- s++ s-- | p | lim | home | save N | go N | play | loop | stop | speed | dwell | off | on");
  }
}

void setup() {
  Serial.begin(115200);
  prefs.begin("arm", false);
  for (int i = 0; i < JOINT_COUNT; i++) {
    limit_lo[i] = 1000;
    limit_hi[i] = 2000;
    home_us[i] = 1500;
  }
  memset(pose_used, 0, sizeof pose_used);
  prefs.getBytes("lo", limit_lo, sizeof limit_lo);
  prefs.getBytes("hi", limit_hi, sizeof limit_hi);
  prefs.getBytes("home", home_us, sizeof home_us);
  prefs.getBytes("pose", poses, sizeof poses);
  prefs.getBytes("used", pose_used, sizeof pose_used);

  for (int i = 0; i < JOINT_COUNT; i++) {
    current_us[i] = start_us[i] = target_us[i] = home_us[i];
    powered[i] = resting[i] = false;
    ledcAttach(PINS[i], PWM_FREQ_HZ, PWM_RES_BITS);
    write_pulse(i);
  }
  delay(200);
  Serial.println("\nSAM ready, all joints limp. 'p' lists them.");
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
  if (moving && millis() - last_frame_ms >= FRAME_MS) {
    last_frame_ms = millis();
    float t = move_ms == 0 ? 1.0f : min(1.0f, (float)(millis() - move_start_ms) / move_ms);
    float eased = t * t * (3.0f - 2.0f * t);   // smoothstep
    for (int i = 0; i < JOINT_COUNT; i++) {
      if (!powered[i]) continue;
      current_us[i] = start_us[i] + (target_us[i] - start_us[i]) * eased;
      write_pulse(i);
    }
    if (t >= 1.0f) {
      moving = false;
      dwell_until_ms = millis() + dwell_ms;
      rest_at_ms[YAW] = millis() + REST_MS;
      // nothing to hold at the open limit, so the gripper can rest right away there
      rest_at_ms[GRIP] = millis() + (target_us[GRIP] >= limit_hi[GRIP] ? REST_MS : GRIP_HOLD_MS);
    }
  }

  for (int i : {YAW, GRIP}) {
    if (moving || !rest_at_ms[i] || millis() < rest_at_ms[i]) continue;
    rest_at_ms[i] = 0;
    if (powered[i] && !resting[i]) {
      resting[i] = true;
      write_pulse(i);
    }
  }

  if (!moving && play_index >= 0 && millis() >= dwell_until_ms) {
    int k = next_pose(play_index + 1);
    if (k < 0 && looping) k = next_pose(0);
    if (k < 0) {
      Serial.println("done");
      play_index = -1;
    } else {
      play_index = k;
      go_to_pose(k);
    }
  }
}
