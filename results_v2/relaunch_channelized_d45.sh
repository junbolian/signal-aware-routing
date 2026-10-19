#!/bin/sh
# channelized_d45 restarted at 23:01 on 2026-10-18 with 9 slots after the decision-100 drivers freed theirs:
# two processes with disjoint networks; finished cells are skipped, the five interrupted cells rerun.
cd /e/Optimization/signal-aware-routing
PY=/e/Optimization/sr-venv311/Scripts/python.exe
$PY run_v2.py --layout channelized --decide 45 --nets 0 1 2 3 4 --jobs 4 --port-base 9100 > results_v2/stdout_channelized_d45_a.txt 2>&1 &
$PY run_v2.py --layout channelized --decide 45 --nets 5 6 7 8 9 --jobs 5 --port-base 9150 > results_v2/stdout_channelized_d45_b.txt 2>&1 &
wait
date '+channelized_d45 finished %Y-%m-%d %H:%M:%S' >> results_v2/launch_times.txt
