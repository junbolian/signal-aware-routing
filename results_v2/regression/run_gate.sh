#!/bin/sh
# Task 0.4 regression gate: network 0, seed 11, demand 1/15, five methods in parallel
cd /e/Optimization/signal-aware-routing
PY=/e/Optimization/sr-venv311/Scripts/python.exe
export SR_NETSEED=0 SR_RESULTS=results_v2/regression
p=8851
for m in STATIC LA1 TDOPT_replan TDOPT-Q LA2; do
  ( s=$(date +%s); SR_PORT=$p $PY sumo_runner.py dem $m 11 15 > results_v2/regression/log_$m.txt 2>&1; echo "$m exit=$? wall_s=$(( $(date +%s)-s ))" >> results_v2/regression/walltimes.txt ) &
  p=$((p+1))
done
wait
