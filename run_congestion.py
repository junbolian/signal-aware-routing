"""Congestion experiment on one SUMO build, resumable (cross-platform twin of
run_congestion.sh). A (scenario, method, seed) cell is skipped when its result
file exists. Order: the seven core methods at moderate, light and heavy demand
(usable partial results first), then the moderate-only extras and the
traffic-aware baselines, then the empty net.

Usage:  python run_congestion.py            # all seeds 11..20
        python run_congestion.py 11 12 13   # a subset of seeds (run several
                                            # processes with disjoint seeds to parallelise)"""
import os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
SEEDS = [int(s) for s in sys.argv[1:]] or list(range(11, 21))
CORE = ["STATIC", "LA1", "MDP", "TDOPT_replan", "STATIC-Q", "MDP-Q", "TDOPT-Q"]
EXTRA_MOD = ["GREEDY", "TDOPT_open", "STATIC-TRAFFIC", "STATIC-TRAFFIC-RT"]
EXTRA_DEM = ["STATIC-TRAFFIC-RT"]
TAG = {"STATIC-TRAFFIC-RT": "STATICTRAFFICRT", "STATIC-TRAFFIC": "STATICTRAFFIC",
       "STATIC-Q": "STATICQ", "MDP-Q": "MDPQ", "TDOPT-Q": "TDOPTQ"}
PY = sys.executable


def stamp():
    return time.strftime("%H:%M:%S")


def run(args, outfile):
    if os.path.exists(outfile):
        return
    print(f"[{stamp()}] {' '.join(args)}", flush=True)
    p = subprocess.run([PY, "sumo_runner.py"] + args, capture_output=True, text=True)
    for line in (p.stdout + p.stderr).splitlines():
        if "Retrying" in line or "Could not connect" in line:
            continue
        print("   " + line, flush=True)
    if p.returncode != 0:
        print(f"[{stamp()}] FAILED (exit {p.returncode}): {' '.join(args)}", flush=True)


def cell(mode, m, s, den=None):
    t = TAG.get(m, m)
    return f"sumo_results/mod_{t}_{s}.json" if mode == "mod" else f"sumo_results/dem{den}_{t}_{s}.json"


os.makedirs("sumo_results", exist_ok=True)
for s in SEEDS:
    for m in CORE:
        run(["mod", m, str(s)], cell("mod", m, s))
for den in (25, 10):
    for s in SEEDS:
        for m in CORE:
            run(["dem", m, str(s), str(den)], cell("dem", m, s, den))
for s in SEEDS:
    for m in EXTRA_MOD:
        run(["mod", m, str(s)], cell("mod", m, s))
for den in (25, 10):
    for s in SEEDS:
        for m in EXTRA_DEM:
            run(["dem", m, str(s), str(den)], cell("dem", m, s, den))
if SEEDS[0] == 11:      # the empty-net cells once, by the process that owns seed 11
    run(["empty"], "sumo_results/empty.json")
    run(["empty_one", "TDOPT_replan"], "sumo_results/empty_one_TDOPT_replan.json")
    run(["empty_one", "MDP"], "sumo_results/empty_one_MDP.json")
print(f"[{stamp()}] DONE seeds {SEEDS}", flush=True)
