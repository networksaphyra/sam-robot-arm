#!/bin/bash
# One demo take as a single continuous path: the rack starts on the pick spot and ends on the place spot, the
# mirror of it about straight ahead (yaw 1500). The arm starts at rest.
# Usage: demo_take.sh [delay_s] [pick_yaw] [place_yaw]; swap the two yaws to carry the rack back.
PANEL=http://127.0.0.1:8765
C() {
  for c in "$@"; do
    curl -sf -X POST --data "$c" $PANEL/cmd >/dev/null || { echo "panel didn't take '$c'; is the ESP32 connected?"; exit 1; }
  done
}

# waits for the firmware's "path done" after the last "run" in the panel log
wait_done() {
  for _ in $(seq 900); do
    curl -s $PANEL/state | python3 -c '
import json, sys
log = json.load(sys.stdin)["log"]
runs = [i for i, line in enumerate(log) if line == "> run"]
sys.exit(0 if runs and "path done" in log[runs[-1] + 1:] else 1)' && return 0
    sleep 0.2
  done
  echo "timed out waiting for the path to finish"
  return 1
}

REST='s 1960 e 930 w 1345'
ABOVE='s 1060 e 980 w 1515'   # over a spot, clear of the rack
DOWN='s 860 e 980 w 1515'     # gripper around the rack
GRIP=1400
HOLD=$(( GRIP - 10 ))
PICK=${2:-1815}
PLACE=${3:-1185}
MIDDLE=$(( (PICK + PLACE) / 2 ))

sleep "${1:-3}"
C wpclear
# 1. rest, still
C "wp $REST y 1490 g 2200 v 350" "wp ms 2000"
# 2. wake up: look left and right, nod the wrist, flex the gripper
C "wp y 1650 w 1500 v 250" "wp y 1330 w 1190 v 250"
C "wp g 1800 v 400" "wp g 2200 v 400"
# 3. reach over the pick spot, pause, come straight down slowly, grab
C "wp $ABOVE y $PICK v 350" "wp ms 300"
C "wp $DOWN v 150" "wp ms 300" "wp g $GRIP v 300" "wp ms 500"
# 4. lift straight up, then present it high, wrist level, between the spots, with a nod. Every carry waypoint nudges
#    the grip by 10 us: the firmware only powers a gripper that is moving (its stall guard rests it after 1.5 s
#    still), so this keeps it squeezing until the rack is down.
C "wp $ABOVE g $HOLD v 150" "wp s 1737 e 1530 w 1012 y $MIDDLE g $GRIP v 350"
C "wp w 1160 g $HOLD v 250" "wp w 1012 g $GRIP v 250" "wp ms 400"
# 5. carry it over the place spot, pause, set it down slowly, let go, lift straight up
C "wp $ABOVE y $PLACE g $HOLD v 350" "wp ms 300"
C "wp $DOWN g $GRIP v 150" "wp ms 300" "wp g 2200 v 300" "wp ms 300"
C "wp $ABOVE v 150"
# 6. back to rest
C "wp $REST y 1490 v 350"
C run
wait_done && echo take done
