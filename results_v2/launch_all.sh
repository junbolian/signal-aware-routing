#!/bin/sh
# Launch the four v2 drivers concurrently (22 slots on 24 cores). Logs: results_v2/<dir>/driver_log.txt
cd /e/Optimization/signal-aware-routing
PY=/e/Optimization/sr-venv311/Scripts/python.exe
date '+launch %Y-%m-%d %H:%M:%S' >> results_v2/launch_times.txt
$PY run_v2.py --layout shared      --decide 45  --jobs 13 > results_v2/stdout_shared_d45.txt 2>&1 &
$PY run_v2.py --layout channelized --decide 45  --jobs 5  > results_v2/stdout_channelized_d45.txt 2>&1 &
$PY run_v2.py --layout shared      --decide 100 --jobs 2  > results_v2/stdout_shared_d100.txt 2>&1 &
$PY run_v2.py --layout channelized --decide 100 --jobs 2  > results_v2/stdout_channelized_d100.txt 2>&1 &
wait
date '+all drivers finished %Y-%m-%d %H:%M:%S' >> results_v2/launch_times.txt
