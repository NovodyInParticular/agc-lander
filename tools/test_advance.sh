#!/bin/sh
# "Advance to next event": the jumps are offered where nothing needs doing and
# the flight is the canonical one, and refused when the flight has deviated.
set -e
LAB=${1:-./build/linux/agc-lab}
fail() { echo "FAIL: $1"; exit 1; }

# 1. The automatic crew, advancing at every chance, still lands well.
OUT=$($LAB pdi terrain crew watch:60,60 advance watch:40,40 advance watch:5,5 advance watch:250,250 landed)
echo "$OUT" | grep -E '^ADVANCE|^LANDING|^RESULT'
for ev in "TIG-30" "RADAR" "HIGH GATE"; do
    echo "$OUT" | grep -q "^ADVANCE $ev " || fail "no advance to $ev"
done
echo "$OUT" | grep -q '^RESULT good' || fail "no good landing after advancing"

# 2. No V57 after the RADAR jump: never offered high gate.
OUT=$($LAB pdi terrain crew watch:60,60 advance watch:40,40 advance manual watch:5,5 nextevent watch:30,30 nextevent advance)
echo "$OUT" | grep -E '^NEXTEVENT'
echo "$OUT" | grep -q '^ADVANCE none' || fail "advanced without V57"
echo "$OUT" | grep -q 'FLGWRD11' || fail "missing V57 not noticed in the AGC flags"

# 3. Engine disarmed in the coast: refused; armed again: offered.
OUT=$($LAB pdi terrain crew watch:60,60 arm:0 nextevent arm:1 nextevent)
echo "$OUT" | grep -E '^NEXTEVENT'
echo "$OUT" | grep '^NEXTEVENT' | sed -n 1p | grep -q 'next event: none' || fail "advanced with the engine disarmed"
echo "$OUT" | grep '^NEXTEVENT' | sed -n 2p | grep -q 'next event: TIG-30' || fail "not offered TIG-30 when armed"

# 4. Yawed off the ignition attitude: refused.
OUT=$($LAB pdi terrain crew watch:60,60 manual mode:1 rhc:0,0.6,0 watch:4,4 rhc:0,0,0 watch:3,3 mode:2 watch:5,5 nextevent)
echo "$OUT" | grep -E '^NEXTEVENT'
echo "$OUT" | grep '^NEXTEVENT' | grep -q 'attitude .* next event: none' || fail "advanced while yawed off attitude"

echo "ADVANCE TEST PASSED"
