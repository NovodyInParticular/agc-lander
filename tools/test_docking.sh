#!/bin/sh
# Lift-off with P12, the rendezvous shortcut, then a docking flown only with
# the hand controllers (THC pulses, RHC in ATT HOLD) through Luminary's DAP
# (the crew's V48: 0.3 deg deadband, 2-jet X translation). Passes on a hard
# dock with a gentle, lined-up capture.
set -e
LAB=${1:-./build/linux/agc-lab}
OUT=$($LAB liftoff crew ascent:520 rndz dockpilot:900)
echo "$OUT" | grep -E '^DOCKED|^DOCKFAIL|Capture|hard dock' || true
echo "$OUT" | awk '
/^DOCKED/ { found = 1; speed = $4; radial = $7; angle = $10; res = $NF }
/^DOCKFAIL/ { print "FAIL: " $0; exit 1 }
END {
    if (!found) { print "FAIL: no capture"; exit 1 }
    if (res != "hard") { print "FAIL: not hard docked (" res ")"; exit 1 }
    if (speed > 0.3) { print "FAIL: capture too fast"; exit 1 }
    if (radial > 0.33) { print "FAIL: off axis"; exit 1 }
    printf "docked: %.3f m/s, %.0f cm off axis, %.2f deg\n", speed, radial * 100, angle
    print "DOCKING TEST PASSED"
}'
