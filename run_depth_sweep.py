"""Demand sweep on one platform for the value of timing depth: background
demand from zero upward on the same SUMO network, lane layout, and decision
point, with the queue-ignorant lookahead family LA-1 to LA-3, the full-timing
planner with and without the queue model, and the static baseline. Twenty
background seeds (11 to 30), four departures each; at demand zero the departure
phase is spread over the cycle instead (see sumo_runner.py). Resumable.

Usage:  python run_depth_sweep.py            # all seeds
        python run_depth_sweep.py 11 12 13   # a seed subset, for parallel processes
Results go to the directory named by SR_RESULTS (default sumo_results_depth)."""
import os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
RES = os.environ.get("SR_RESULTS", "sumo_results_depth")
os.environ["SR_RESULTS"] = RES
SEEDS = [int(s) for s in sys.argv[1:]] or list(range(11, 31))
LEVELS = [0, 100, 50, 25, 15, 10]            # insertion probability 1/den per corridor entry; 0 = empty
METHODS = ["STATIC", "LA1", "LA2", "LA3", "TDOPT_replan", "TDOPT-Q"]
TAG = {"TDOPT-Q": "TDOPTQ"}
PY = sys.executable
os.makedirs(RES, exist_ok=True)

def run(args, outfile):
    if os.path.exists(outfile):
        return
    print(f"[{time.strftime('%H:%M:%S')}] {' '.join(args)}", flush=True)
    p = subprocess.run([PY, "sumo_runner.py"] + args, capture_output=True, text=True)
    for line in (p.stdout + p.stderr).splitlines():
        if "Retrying" in line or "Could not connect" in line:
            continue
        print("   " + line, flush=True)
    if p.returncode != 0:
        print(f"   FAILED (exit {p.returncode})", flush=True)

for den in LEVELS:
    for s in SEEDS:
        for m in METHODS:
            run(["dem", m, str(s), str(den)], f"{RES}/dem{den}_{TAG.get(m, m)}_{s}.json")
print(f"[{time.strftime('%H:%M:%S')}] DONE seeds {SEEDS}", flush=True)
