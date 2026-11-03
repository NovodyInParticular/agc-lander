#!/bin/sh
# Full Apollo 11 descent, flown by Luminary099 with a scripted crew.
# Passes if the LM touches down gently, near the target, with no alarms.
set -e
LAB=${1:-./build/linux/agc-lab}
OUT=$($LAB pdi crew watch:1680,40 landed nav)
echo "$OUT" | grep -E 'crew|LANDING|FAILREG'
echo "$OUT" | awk '
/^LANDING/ {
    found = 1
    for (i = 1; i <= NF; i++) {
        if ($i == "speed") speed = $(i + 1)
        if ($i == "N," || $i == "S,") dn = $(i - 2)
        if ($i == "E" || $i == "W") de = $(i - 2)
    }
    if ($2 != "touchdown=1") { print "FAIL: no touchdown"; exit 1 }
}
/FAILREG/ { if ($2 != "00000") { print "FAIL: program alarm " $2; exit 1 } }
END {
    if (!found) { print "FAIL: no landing report"; exit 1 }
    miss = sqrt(dn * dn + de * de)
    printf "touchdown %.2f m/s, miss %.0f m\n", speed, miss
    if (speed > 3.0) { print "FAIL: touchdown too hard"; exit 1 }
    if (miss > 3000) { print "FAIL: landed too far from target"; exit 1 }
    print "LANDING TEST PASSED"
}'
