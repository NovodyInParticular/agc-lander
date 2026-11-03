#!/usr/bin/env python3
"""Compare a yaYUL .bin against a Virtual AGC .binsource rope transcription (15-bit data words)."""
import sys

def parse_binsource(path):
    banks, bank, parity = {}, None, 0
    for line in open(path):
        line = line.split(';')[0].strip()
        if not line:
            continue
        if line.startswith('PARITY='):
            parity = int(line[7:]); continue
        if line.startswith('BANK='):
            bank = int(line[5:], 8); banks.setdefault(bank, []); continue
        for tok in line.split():
            if tok.startswith('@'):
                continue
            w = int(tok, 8)
            banks[bank].append(w >> 3 if parity else w)  # last octal digit is the parity bit
    return banks

def parse_bin(path):
    data = open(path, 'rb').read()
    words = [((data[i] << 8 | data[i + 1]) >> 1) & 0x7FFF for i in range(0, len(data), 2)]
    order = [2, 3, 0, 1] + list(range(4, 36))  # yaYUL file order
    return {b: words[i * 1024:(i + 1) * 1024] for i, b in enumerate(order) if i * 1024 < len(words)}

ref, got = parse_binsource(sys.argv[1]), parse_bin(sys.argv[2])
total = diff = 0
for b, words in sorted(ref.items()):
    for i, w in enumerate(words):
        total += 1
        g = got.get(b, [None] * 1024)[i]
        if g != w:
            diff += 1
            if diff <= 10:
                print(f"bank {b:02o} addr {i:04o}: rope {w:05o} built {g if g is None else format(g, '05o')}")
print(f"compared {total} words, {diff} differ")
sys.exit(1 if diff else 0)
