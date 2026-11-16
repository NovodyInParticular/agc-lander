#!/bin/sh
# The scenarios: 1202/1201 alarms from the rendezvous radar fault, the low-fuel
# landing the computer would make on a boulder, and the retargeted second site.
set -e
LAB=${1:-./build/linux/agc-lab}
fail() { echo "FAIL: $1"; exit 1; }

# 1202 ALARMS: real executive overflows, Houston's GO, and the computer still lands.
OUT=$($LAB snapload:RADAR crew rrfault:6400 alarms:400 landed)
echo "$OUT" | grep -E 'Houston|^RESULT' | head -4
echo "$OUT" | grep -qE 'FAILREG 0120[12]' || fail "no 1202 / 1201 alarm with the radar fault"
echo "$OUT" | grep -q "We're GO on that alarm\|We're GO. Same type" || fail "no GO from Houston"
echo "$OUT" | grep -q '^RESULT good' || fail "no good landing through the alarms"

# LOW FUEL OVER THE BOULDERS: left to the computer, Eagle comes down on a boulder.
OUT=$($LAB snapload:BOULDERS crew watch:130,130 landed)
echo "$OUT" | grep -E 'Houston|^RESULT'
echo "$OUT" | grep -q 'Houston: "60 seconds."' || fail "no 60-seconds call"
echo "$OUT" | grep -q 'Houston: "30 seconds."' || fail "no 30-seconds call"
echo "$OUT" | grep -q '^RESULT crash cause 5' || fail "the computer's spot is not on a boulder"

# ANOTHER SITE: the computer lands near the new target.
OUT=$($LAB snapload:"SITE 2" crew watch:260,260 landed plane)
echo "$OUT" | grep -E '^RESULT|^PLANE'
echo "$OUT" | grep -q '^RESULT good' || fail "no good landing at site 2"
echo "$OUT" | awk '/^PLANE/ { dx = $4 + 7000; dz = $6 + 60; d = sqrt(dx * dx + dz * dz); printf "site 2: %.0f m from the target\n", d; if (d > 100) exit 1 }' ||
    fail "landed far from site 2"

echo "SCENARIO TEST PASSED"
