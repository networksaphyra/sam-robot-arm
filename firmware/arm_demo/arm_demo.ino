// SAM firmware: drives the five servos from serial commands (115200 baud, one command per line).
// Joint keys: y yaw, s shoulder, e elbow, w wrist, g gripper.
//
//   s 1500          move a joint to 1500 us         s+ s- s++ s--   nudge by 10 / 50 us
//   p               print joints, limits and saved poses
//   lim s 600 2100  set a joint's limits            home s 600      set a joint's home
//   home            move every joint home
//   save 3, go 3    store / recall pose slot 3 (0-19)                clear   delete all poses
//   play, loop      run the saved poses in order, once or repeating  stop    stop where it is
//   speed 600       top joint speed in us/s (50-3000)                dwell 500   pause at each pose, ms
//   off, off s      all joints limp / one joint limp                 on      power all joints at their targets
//   rest 400        how long the yaw (and the open gripper) waits before its pulses stop, ms
//   ease 3, ease 5  cubic or quintic easing; quintic also ramps the acceleration, about 25% slower at equal speed
//
// Paths: queue waypoints, then run them as one continuous motion. Joints left out of a waypoint keep their value
// from the one before. "ms" sets a segment's time, "v" its top speed; a waypoint with only "ms" is a pause.
//   wp s 1438 e 1860 v 350    wp ms 500    run    wpclear
// "path done" is printed when a path finishes.
//
// Limits, homes and poses are kept in flash, so they survive a reset.
//
// All joints start limp. The firmware can't know where a limp joint is, so the first command to it jumps
// straight to the target instead of ramping. Move the arm close to that position by hand first.

#include <Preferences.h>
#include "motion.h"

const char JOINT_KEYS[JOINT_COUNT] = {'y', 's', 'e', 'w', 'g'};
const int PINS[JOINT_COUNT] = {27, 26, 25, 33, 32};
const int YAW = 0, GRIP = 4;

const int PWM_FREQ_HZ = 50;
const int PWM_RES_BITS = 16;
const uint32_t PERIOD_US = 1000000UL / PWM_FREQ_HZ;
const uint32_t FRAME_MS = 20;
const int POSE_SLOTS = 20;

// The yaw and gripper stop getting pulses once they've been still for a moment. The yaw carries no gravity load and
// its gearbox holds the turret, so this stops it hunting back and forth in the gear backlash. The gripper's gearbox
// friction keeps hold of an object, so a squeeze or a jam only stalls the servo for GRIP_HOLD_MS.
uint32_t rest_ms = 400;
const uint32_t GRIP_HOLD_MS = 1500;
int ease_order = 3;

int limit_lo[JOINT_COUNT], limit_hi[JOINT_COUNT], home_us[JOINT_COUNT];
float current_us[JOINT_COUNT];
int target_us[JOINT_COUNT];  // where the current motion ends
bool powered[JOINT_COUNT];   // false = limp
bool resting[JOINT_COUNT];   // powered, but pulses paused (yaw and gripper)
uint32_t rest_at_ms[JOINT_COUNT] = {0};

int poses[POSE_SLOTS][JOINT_COUNT];
bool pose_used[POSE_SLOTS];

int speed_us_per_s = 600, dwell_ms = 500;

Path active;    // the motion being run
Path pending;   // waypoints queued with "wp"
bool pending_set[JOINT_COUNT];
bool moving = false;
bool path_command = false;  // the active path came from "run", so report when it ends
bool playing = false;       // the active path is the saved poses
bool looping = false;
int segment = 0;
uint32_t segment_start_ms = 0;

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

// Powers a limp joint. Its real position is unknown, so it jumps to `us` instead of ramping.
void power_on_at(int i, int us) {
  powered[i] = true;
  current_us[i] = us;
  write_pulse(i);
}

// Starts `active` from where the arm is now. Waypoint 0 must already hold the start positions.
void run_active(bool from_run_command) {
  plan_path(active, speed_us_per_s);
  for (int i = 0; i < JOINT_COUNT; i++) target_us[i] = rounded(active.us[active.count - 1][i]);
  path_command = from_run_command;
  if (active.count < 2) {
    moving = false;
    if (path_command) Serial.println("path done");
    return;
  }
  uint32_t total = 0;
  for (int k = 1; k < active.count; k++) total += active.ms[k];
  if (from_run_command) Serial.printf("path: %d waypoints, %lu ms\n", active.count - 1, (unsigned long)total);
  segment = 1;
  segment_start_ms = millis();
  moving = true;
}

void start_active_at_current() {
  active.count = 1;
  active.ease = ease_order;
  for (int i = 0; i < JOINT_COUNT; i++) active.us[0][i] = current_us[i];
}

// One eased move from where every joint is to target_us[], all joints arriving together.
void move_to_targets() {
  looping = playing = false;
  start_active_at_current();
  for (int i = 0; i < JOINT_COUNT; i++) active.us[1][i] = powered[i] ? target_us[i] : current_us[i];
  active.ms[1] = 0;
  active.speed[1] = 0;
  active.count = 2;
  run_active(false);
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
  move_to_targets();
}

void go_to(const int *us) {
  for (int i = 0; i < JOINT_COUNT; i++) {
    target_us[i] = constrain(us[i], limit_lo[i], limit_hi[i]);
    if (!powered[i]) power_on_at(i, target_us[i]);
  }
  move_to_targets();
}

int next_pose(int from) {
  for (int k = from; k < POSE_SLOTS; k++)
    if (pose_used[k]) return k;
  return -1;
}

// Saved poses in order as one path, with a pause of dwell_ms at each. With no dwell the arm flows through them.
void play_poses() {
  int first = next_pose(0);
  for (int i = 0; i < JOINT_COUNT; i++)
    if (!powered[i]) power_on_at(i, constrain(poses[first][i], limit_lo[i], limit_hi[i]));
  start_active_at_current();
  for (int k = first; k < POSE_SLOTS; k++) {
    if (!pose_used[k] || active.count > MAX_WAYPOINTS - 2) continue;
    int n = active.count;
    for (int i = 0; i < JOINT_COUNT; i++) active.us[n][i] = constrain(poses[k][i], limit_lo[i], limit_hi[i]);
    active.ms[n] = 0;
    active.speed[n] = 0;
    active.count++;
    if (dwell_ms > 0) {
      for (int i = 0; i < JOINT_COUNT; i++) active.us[n + 1][i] = active.us[n][i];
      active.ms[n + 1] = dwell_ms;
      active.speed[n + 1] = 0;
      active.count++;
    }
  }
  run_active(false);
  playing = true;
}

void stop_motion() {
  bool was_moving = moving || looping;
  moving = false;
  looping = playing = false;
  for (int i = 0; i < JOINT_COUNT; i++) target_us[i] = rounded(current_us[i]);
  if (was_moving) Serial.println("stopped");
}

// "wp" arguments: pairs of a joint key and a pulse, plus optional "ms N" and "v N".
void queue_waypoint(String arg) {
  if (pending.count == 0) {
    pending.count = 1;
    for (int i = 0; i < JOINT_COUNT; i++) {
      pending.us[0][i] = powered[i] ? target_us[i] : current_us[i];
      pending_set[i] = false;
    }
  }
  if (pending.count >= MAX_WAYPOINTS) {
    Serial.println("path full");
    return;
  }
  int n = pending.count;
  for (int i = 0; i < JOINT_COUNT; i++) pending.us[n][i] = pending.us[n - 1][i];
  pending.ms[n] = 0;
  pending.speed[n] = 0;

  arg.trim();
  while (arg.length()) {
    int space = arg.indexOf(' ');
    if (space < 0) {
      Serial.println("wp: every key needs a value");
      return;
    }
    String key = arg.substring(0, space);
    arg = arg.substring(space + 1);
    arg.trim();
    space = arg.indexOf(' ');
    int value = (space < 0 ? arg : arg.substring(0, space)).toInt();
    arg = space < 0 ? "" : arg.substring(space + 1);
    arg.trim();

    int j = key.length() == 1 ? joint_of(key[0]) : -1;
    if (j >= 0) {
      value = constrain(value, limit_lo[j], limit_hi[j]);
      // A limp joint jumps when the path starts, so it sits at its first value from the start.
      if (!powered[j] && !pending_set[j])
        for (int k = 0; k < n; k++) pending.us[k][j] = value;
      pending_set[j] = true;
      pending.us[n][j] = value;
    } else if (key == "ms") {
      pending.ms[n] = max(0, value);
    } else if (key == "v") {
      pending.speed[n] = constrain(value, 50, 3000);
    } else {
      Serial.printf("wp: unknown key %s\n", key.c_str());
      return;
    }
  }
  pending.count++;
}

void run_pending() {
  if (pending.count < 2) {
    Serial.println("no waypoints queued");
    return;
  }
  looping = playing = false;
  active = pending;
  active.ease = ease_order;
  pending.count = 0;
  for (int i = 0; i < JOINT_COUNT; i++) {
    if (powered[i]) {
      active.us[0][i] = current_us[i];
    } else if (pending_set[i]) {
      power_on_at(i, rounded(active.us[0][i]));
    } else {
      for (int k = 0; k < active.count; k++) active.us[k][i] = current_us[i];
    }
  }
  run_active(true);
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
  } else if (word == "wp") {
    queue_waypoint(arg);
  } else if (word == "wpclear") {
    pending.count = 0;
    Serial.println("waypoints cleared");
  } else if (word == "run") {
    run_pending();
  } else if (word == "stop") {
    stop_motion();
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
    go_to(home_us);
    Serial.println("-> home");
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
    if (k >= 0 && k < POSE_SLOTS && pose_used[k]) {
      go_to(poses[k]);
      Serial.printf("-> pose %d\n", k);
    } else {
      Serial.println("no such pose");
    }
  } else if (word == "play" || word == "loop") {
    if (next_pose(0) < 0) {
      Serial.println("no poses saved");
      return;
    }
    play_poses();
    looping = word == "loop";
  } else if (word == "speed") {
    if (arg.length()) speed_us_per_s = constrain((int)arg.toInt(), 50, 3000);
    Serial.printf("speed %d us/s\n", speed_us_per_s);
  } else if (word == "rest") {
    if (arg.length()) rest_ms = constrain((int)arg.toInt(), 0, 5000);
    Serial.printf("rest %lu ms\n", (unsigned long)rest_ms);
  } else if (word == "ease") {
    if (arg == "3" || arg == "5") ease_order = arg.toInt();
    Serial.printf("ease %d\n", ease_order);
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
    moving = looping = playing = false;
    for (int i = 0; i < JOINT_COUNT; i++) {
      powered[i] = resting[i] = false;
      write_pulse(i);
    }
    Serial.println("all limp");
  } else if (word == "on") {
    moving = looping = playing = false;
    for (int i = 0; i < JOINT_COUNT; i++) {
      powered[i] = true;
      resting[i] = false;
      current_us[i] = target_us[i];
      write_pulse(i);
    }
    Serial.println("all on at their targets");
  } else {
    Serial.println("? y/s/e/w/g <us> | s+ s- s++ s-- | p | lim | home | save N | go N | play | loop | stop | speed | dwell | rest | ease | wp | run | wpclear | off | on");
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
    current_us[i] = target_us[i] = home_us[i];
    powered[i] = resting[i] = false;
    ledcAttach(PINS[i], PWM_FREQ_HZ, PWM_RES_BITS);
    write_pulse(i);
  }
  delay(200);
  Serial.println("\nSAM ready, all joints limp. 'p' lists them.");
}

// The yaw and gripper rest once they've been still for a moment, and wake as soon as the path moves them again.
void update_rest(uint32_t now) {
  for (int i : {YAW, GRIP}) {
    bool still = !moving || !joint_moves(active, segment, i);
    if (!still) {
      rest_at_ms[i] = 0;
      if (resting[i]) {
        resting[i] = false;
        write_pulse(i);
      }
      continue;
    }
    if (!powered[i] || resting[i]) continue;
    if (rest_at_ms[i] == 0) {
      // nothing to hold at the gripper's open limit, so it can rest right away there
      bool holding = i == GRIP && rounded(current_us[i]) < limit_hi[i];
      rest_at_ms[i] = now + (holding ? GRIP_HOLD_MS : rest_ms);
    } else if ((int32_t)(now - rest_at_ms[i]) >= 0) {
      rest_at_ms[i] = 0;
      resting[i] = true;
      write_pulse(i);
    }
  }
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
  uint32_t now = millis();
  if (now - last_frame_ms < FRAME_MS) return;
  last_frame_ms = now;

  if (moving) {
    uint32_t t = now - segment_start_ms;
    while (moving && t >= active.ms[segment]) {
      t -= active.ms[segment];
      segment_start_ms += active.ms[segment];
      if (++segment >= active.count) moving = false;
    }
    for (int i = 0; i < JOINT_COUNT; i++) {
      if (!powered[i]) continue;
      current_us[i] = moving ? path_position(active, segment, t, i) : active.us[active.count - 1][i];
      write_pulse(i);
    }
    if (!moving) {
      if (path_command) Serial.println("path done");
      if (playing && looping) {
        play_poses();
      } else if (playing) {
        playing = false;
        Serial.println("done");
      }
    }
  }
  update_rest(now);
}
