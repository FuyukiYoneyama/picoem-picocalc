#!/usr/bin/env bash
set -u
BIN="./silicon-soak.220248.exe"
PROBE="${RP2354_PROBE:?set RP2354_PROBE to your RP2354 probe VID:PID:SERIAL}"
SEED=5831149830256
DUR=10m

for FILT in pwm pio0 uart0; do
  echo "==== SOAK START filter=${FILT} $(date +%T) ===="
  "$BIN" --soak "$DUR" --seed "$SEED" --filter "$FILT" --probe "$PROBE" \
      > "soak-${FILT}.log" 2>&1
  echo "==== SOAK END   filter=${FILT} exit=$? $(date +%T) ===="
  grep -E 'iterations:|reattach_count:|failing cases:' "soak-${FILT}.log" | head -10
done
echo "==== ALL DONE $(date +%T) ===="
