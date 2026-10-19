"""Task 0.5: timing projection for the v2 sample. Run from the repository root:
python results_v2/timing/project.py   (writes results_v2/TIMING.txt)"""
import json
import os

T = "results_v2/timing"
meas = {}
for m, tag in [("STATIC", "STATIC"), ("LA1", "LA1"), ("MDP", "MDP"), ("TDOPT_replan", "TDOPT_replan"),
               ("TDOPT-Q", "TDOPTQ"), ("MDP-Q", "MDPQ")]:
    meas[m] = json.load(open(f"{T}/meta_dem10_{tag}_n1_11.json"))["wall_s"]
# unmeasured methods priced by their closest measured relative (demand 1/10, the heaviest level, for every level)
proxy = {"LA2": "LA1", "LA3": "LA1", "GREEDY": "STATIC", "TDOPT_open": "TDOPT_replan",
         "STATIC-TRAFFIC": "STATIC", "STATIC-TRAFFIC-RT": "STATIC", "STATIC-Q": "MDP-Q", "MDP-Q-OP": "MDP-Q"}
extra = {"STATIC-TRAFFIC": 1.25, "STATIC-TRAFFIC-RT": 1.25}    # one extra 240 s measurement simulation per run
sec = lambda m: meas[proxy.get(m, m)] * extra.get(m, 1.0) if m not in meas else meas[m]

# runs per network and seed (the order's run count)
runs = {}
add = lambda m, k: runs.__setitem__(m, runs.get(m, 0) + k)
for m in ("STATIC", "LA1", "LA2", "LA3", "MDP", "TDOPT_replan", "TDOPT-Q"):
    add(m, 6)                                   # Task 1, all six levels
for m in ("STATIC-Q", "MDP-Q"):
    add(m, 5)                                   # Task 1, five levels > 0
for m in ("GREEDY", "TDOPT_open"):
    add(m, 2)
add("STATIC-TRAFFIC", 1); add("STATIC-TRAFFIC-RT", 3)
for m in ("STATIC", "LA1", "MDP", "TDOPT_replan", "TDOPT-Q", "STATIC-Q", "MDP-Q"):
    add(m, 3)                                   # Task 2.1, channelized 25/15/10
for m in ("STATIC", "LA1", "MDP", "TDOPT_replan", "TDOPT-Q"):
    add(m, 2)                                   # Task 2.2, decision 100 m on both layouts
add("MDP-Q-OP", 4)                              # Task 3
per = sum(runs.values())
cpu_per_net_seed = sum(k * sec(m) for m, k in runs.items())
cores = os.cpu_count()
slots = cores - 2
CONTENTION = 2.0       # allowance for 22 concurrent runs on 8 P-cores + 16 E-cores versus 7 concurrent runs measured
L = ["Timing projection (Task 0.5)", "",
     "Measured: network 1, seed 11, demand 1/10, four trips per run, six runs concurrently (plus one legacy check run),",
     "wall seconds from meta_dem10_<METHOD>_n1_11.json:"]
L += [f"   {m:14s} {s:6.1f} s" for m, s in meas.items()]
L += ["", f"Runs per network and seed: {per}  (order: 95)",
      f"   " + ", ".join(f"{m} {k}" for m, k in sorted(runs.items())),
      f"Core-seconds per network and seed (demand 1/10 time for every level): {cpu_per_net_seed:.0f}", ""]
L.append(f"cores {cores}, concurrent slots {slots}, contention allowance x{CONTENTION}")
for K in (10, 8):
    tot = cpu_per_net_seed * 4 * K
    wall = tot * CONTENTION / slots / 3600
    L.append(f"K = {K:2d}: {per * 4 * K} runs, {tot / 3600:.1f} core-hours, projected wall time {wall:.1f} h")
wall10 = cpu_per_net_seed * 4 * 10 * CONTENTION / slots / 3600
L += ["", f"Rule: K = 8 only if the K = 10 projection exceeds 72 h. Projection {wall10:.1f} h -> K = {10 if wall10 <= 72 else 8}."]
open("results_v2/TIMING.txt", "w").write("\n".join(L) + "\n")
print("\n".join(L))
