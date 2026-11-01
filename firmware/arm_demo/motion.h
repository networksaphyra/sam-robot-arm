// Joint-space paths through a list of waypoints. Each segment is a Hermite curve: cubic, or quintic with zero
// acceleration at every waypoint, which starts and stops more gently. The arm starts and ends at rest and passes
// through the waypoints in between without stopping, except where a joint turns around or holds still, which
// brings that joint to a stop there.
#pragma once
#include <math.h>
#include <stdint.h>

const int JOINT_COUNT = 5;
const int MAX_WAYPOINTS = 48;

struct Path {
  int count = 0;                           // waypoints, the start included
  int ease = 3;                            // 3 = cubic, 5 = quintic
  float us[MAX_WAYPOINTS][JOINT_COUNT];    // joint positions at each waypoint
  uint32_t ms[MAX_WAYPOINTS];              // ms[k]: time from waypoint k-1 to k; 0 = work it out from the speed
  int speed[MAX_WAYPOINTS];                // speed[k]: top speed in us/s on the way to waypoint k; 0 = the default
  float vel[MAX_WAYPOINTS][JOINT_COUNT];   // velocity at each waypoint, us per ms
};

inline int segment_speed(const Path &p, int k, int default_speed) {
  return p.speed[k] > 0 ? p.speed[k] : default_speed;
}

// A segment that starts and stops at rest peaks at 1.5x its average speed (cubic) or 1.875x (quintic), so its time
// is set to keep that peak at the requested speed.
inline uint32_t segment_ms(const Path &p, int k, int default_speed) {
  float furthest = 0;
  for (int j = 0; j < JOINT_COUNT; j++) furthest = fmaxf(furthest, fabsf(p.us[k][j] - p.us[k - 1][j]));
  float peak = p.ease == 5 ? 1875.0f : 1500.0f;
  return (uint32_t)(furthest * peak / segment_speed(p, k, default_speed) + 0.5f);
}

// Fills in the segment times and the velocity at every waypoint. Waypoints that would take no time are dropped.
inline void plan_path(Path &p, int default_speed) {
  int kept = 1;
  for (int k = 1; k < p.count; k++) {
    uint32_t ms = p.ms[k] > 0 ? p.ms[k] : segment_ms(p, k, default_speed);
    if (ms == 0) continue;
    for (int j = 0; j < JOINT_COUNT; j++) p.us[kept][j] = p.us[k][j];
    p.ms[kept] = ms;
    p.speed[kept] = p.speed[k];
    kept++;
  }
  p.count = kept;

  for (int j = 0; j < JOINT_COUNT; j++) {
    p.vel[0][j] = p.vel[p.count - 1][j] = 0;
    for (int k = 1; k < p.count - 1; k++) {
      float before = (p.us[k][j] - p.us[k - 1][j]) / p.ms[k];
      float after = (p.us[k + 1][j] - p.us[k][j]) / p.ms[k + 1];
      // A joint that turns around or holds still stops at the waypoint. Otherwise it passes through at the harmonic
      // mean of the two slopes, which keeps it between the waypoints (no overshoot; Fritsch and Carlson), and no
      // faster than the slower of the two segments allows.
      float v = before * after <= 0 ? 0 : 2 * before * after / (before + after);
      float limit = fminf(segment_speed(p, k, default_speed), segment_speed(p, k + 1, default_speed)) / 1000.0f;
      p.vel[k][j] = fmaxf(-limit, fminf(limit, v));
    }
  }
}

inline bool joint_moves(const Path &p, int k, int j) { return p.us[k][j] != p.us[k - 1][j]; }

// Position of joint j, t ms into segment k (from waypoint k-1 to k).
inline float path_position(const Path &p, int k, uint32_t t, int j) {
  if (!joint_moves(p, k, j)) return p.us[k][j];   // a held joint has zero velocity at both ends
  float span = p.ms[k], s = t / span, s2 = s * s, s3 = s2 * s;
  float from = p.us[k - 1][j], to = p.us[k][j], v0 = span * p.vel[k - 1][j], v1 = span * p.vel[k][j];
  if (p.ease == 5) {
    float s4 = s3 * s, s5 = s4 * s;
    return (1 - 10 * s3 + 15 * s4 - 6 * s5) * from + (s - 6 * s3 + 8 * s4 - 3 * s5) * v0 +
           (-4 * s3 + 7 * s4 - 3 * s5) * v1 + (10 * s3 - 15 * s4 + 6 * s5) * to;
  }
  return (2 * s3 - 3 * s2 + 1) * from + (s3 - 2 * s2 + s) * v0 + (s3 - s2) * v1 + (3 * s2 - 2 * s3) * to;
}
