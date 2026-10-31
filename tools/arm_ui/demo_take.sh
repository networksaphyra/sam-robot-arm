#!/bin/bash
# One demo take: rack starts on the pick spot (yaw 1290), ends on the place spot (yaw 1650). Arm starts at rest.
C() { for c in "$@"; do curl -s -X POST --data "$c" http://127.0.0.1:8765/cmd >/dev/null; done; }
REST=('s 1960' 'e 930' 'w 1345')
PICK=${2:-1290}; PLACE=${3:-1650}
sleep "${1:-3}"
# 1. rest, still
C 'speed 350' "${REST[@]}" 'y 1490' 'g 2200'; sleep 2
# 2. wake up: look left and right, nod the wrist, flex the gripper
C 'speed 250' 'y 1650' 'w 1500'; sleep 2.2
C 'y 1330' 'w 1190'; sleep 3.4
C 'speed 400' 'g 1800'; sleep 1.9
C 'g 2200'; sleep 1.9
# 3. reach over the pick spot, approach, grab
C 'speed 350' 's 1438' 'e 1860' 'w 1280' "y $PICK"; sleep 4.4
C 's 1220' 'e 1590' 'w 1205'; sleep 1.6
C 'speed 400' 'g 1460'; sleep 3.0
# 4. lift and present it, high, wrist level, turned between the spots
C 'speed 350' 's 1438' 'e 1860' 'w 1280'; sleep 1.6
C 's 1737' 'e 1530' 'w 1012' 'y 1470'; sleep 2.6
C 'speed 250' 'w 1160'; sleep 1.2
C 'w 1012'; sleep 1.4
# 5. carry to the place spot, set it down, back off
C 'speed 350' 's 1438' 'e 1860' 'w 1280' "y $PLACE"; sleep 2.6
C 's 1220' 'e 1590' 'w 1205'; sleep 1.6
C 'speed 400' 'g 2200'; sleep 3.0
C 'speed 350' 's 1438' 'e 1860' 'w 1280'; sleep 1.6
# 6. home
C "${REST[@]}" 'y 1490'; sleep 4.6
echo take done
