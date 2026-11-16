#!/bin/sh
# The whole Apollo 11 LM mission in one run: Luminary lands from PDI, P12
# lifts off from wherever the LM came down (the game's L key), then the
# rendezvous shortcut and a docking with the hand controllers.
set -e
LAB=${1:-./build/linux/agc-lab}
OUT=$($LAB pdi crew watch:1680,40 landed liftoffkeep ascent:520 orbit nav rndz dockpilot:900)
echo "$OUT" | grep -E '^LANDING|^LIFTOFF|^ORBIT LM|^DOCKED|^DOCKFAIL|FAILREG'
echo "$OUT" | awk '
/^LANDING/ { landed = ($2 == "touchdown=1") }
/^ORBIT LM/ { orbit = 1; peri = $4; apo = $7 }
/FAILREG/ { if ($2 != "00000") { print "FAIL: program alarm " $2; exit 1 } }
/^DOCKED/ { res = $NF }
END {
    if (!landed) { print "FAIL: no landing"; exit 1 }
    if (!orbit || peri < 8 || apo < 60) { print "FAIL: ascent did not reach a safe orbit"; exit 1 }
    if (res != "hard") { print "FAIL: no hard dock"; exit 1 }
    print "MISSION TEST PASSED"
}'
