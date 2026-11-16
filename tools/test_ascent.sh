#!/bin/sh
# Lunar lift-off flown by Luminary099's P12 with the scripted crew (ABORT
# STAGE, ENG ARM ASC, PRO at V99). Passes if the ascent stage reaches a safe
# orbit near Apollo 11's 9.4 x 46.7 nmi, in the CSM's plane, with no alarms.
set -e
LAB=${1:-./build/linux/agc-lab}
OUT=$($LAB liftoff crew ascent:520 orbit nav)
echo "$OUT" | grep -E 'crew|ORBIT|FAILREG'
echo "$OUT" | awk '
/^ORBIT LM/ { found = 1; peri = $4; apo = $7 }
/^ORBIT CSM/ { for (i = 1; i <= NF; i++) if ($i == "wedge") wedge = $(i + 1) }
/FAILREG/ { if ($2 != "00000") { print "FAIL: program alarm " $2; exit 1 } }
END {
    if (!found) { print "FAIL: no orbit report"; exit 1 }
    printf "orbit %.1f x %.1f km, wedge %.3f deg\n", peri, apo, wedge
    if (peri < 8) { print "FAIL: perilune too low"; exit 1 }
    if (apo < 60 || apo > 130) { print "FAIL: apolune off target"; exit 1 }
    if (wedge > 0.5) { print "FAIL: out of the CSM plane"; exit 1 }
    print "ASCENT TEST PASSED"
}'
