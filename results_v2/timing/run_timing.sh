#!/bin/sh
# Task 0.5 timing: network 1, seed 11, demand 1/10, six methods run concurrently.
# MDP-Q uses the legacy network-0 kernel sumo_results/kernel_dem10_foldA.json
# (copied here as kernel_dem10_foldB.json, the file network 1 loads), for timing only.
cd /e/Optimization/signal-aware-routing
PY=/e/Optimization/sr-venv311/Scripts/python.exe
export SR_NETSEED=1 SR_RESULTS=results_v2/timing
rm -f results_v2/timing/walltimes.txt
p=8871
for m in STATIC LA1 MDP TDOPT_replan TDOPT-Q MDP-Q; do
  ( s=$(date +%s); SR_PORT=$p $PY sumo_runner.py dem $m 11 10 > results_v2/timing/log_$m.txt 2>&1; echo "$m exit=$? wall_s=$(( $(date +%s)-s ))" >> results_v2/timing/walltimes.txt ) &
  p=$((p+1))
done
wait
