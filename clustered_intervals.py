"""Task 6: network-clustered 95 percent intervals for the grid study.

The manuscript's Tables 1 and 3 and the scenario-ablation table report intervals
that treat the (network, departure) pairs as independent. Each network
contributes eight departures on the same offsets and the same link times, so
those observations are not independent and the per-instance interval understates
the sampling error of the network draw.

This recomputes every interval with the network as the cluster unit: aggregate
the eight departures of a network into one number first, then take the normal
approximation over the 60 (or 40) network means. The means are unchanged by
construction; only the intervals move.

No experiment is re-run. Tables 1 and the ablation table come from
raw_results.csv; the LA-k table under countdown error and the MDP numbers are
recomputed from the deterministic analytical suite (the same seeds and the same
code path that produced the printed tables, which the acceptance test of Task 2.1
showed reproduce bit for bit).

Writes clustered_intervals.txt and clustered_intervals.json.
"""
import csv, json, math, random, collections

import signal_routing as sr
from lookahead import static_V, la_route
import extend_lookahead as xl

Z = 1.96


def mean_ci(xs):
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(xs) / n
    if n < 2:
        return m, 0.0
    v = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, Z * math.sqrt(v / n)


def clustered_ci(by_cluster):
    """Interval over cluster means; the point estimate stays the pooled mean."""
    ms = [sum(v) / len(v) for v in by_cluster.values() if v]
    return mean_ci(ms)[1] if len(ms) > 1 else 0.0


def pooled(by_cluster):
    xs = [x for v in by_cluster.values() for x in v]
    return sum(xs) / len(xs), len(xs), len(by_cluster)


OUT = []          # (table, row, column, mean, ci_instance, ci_clustered)


def emit(table, row, col, by_cluster):
    m, n, k = pooled(by_cluster)
    ci_i = mean_ci([x for v in by_cluster.values() for x in v])[1]
    ci_c = clustered_ci(by_cluster)
    OUT.append({"table": table, "row": row, "column": col, "mean": m,
                "ci_instance": ci_i, "ci_clustered": ci_c,
                "ratio": (ci_c / ci_i if ci_i else float("nan")),
                "n_instances": n, "n_clusters": k})


# ------------------------------------------------------------------ Table 1
rows = list(csv.DictReader(open("raw_results.csv")))
by = collections.defaultdict(lambda: collections.defaultdict(list))
for r in rows:
    if r["scenario"] != "base_N8" or r["sigma"] not in ("", None):
        continue
    by[r["method"]][r["net"]].append(r)

opt = {}
for net, rs in by["TD-OPT"].items():
    for r in rs:
        opt[(net, r["t0"])] = float(r["time"])

for meth in ("SP-FF", "SP-STATIC", "GREEDY", "TD-OPT"):
    gap = collections.defaultdict(list)
    wait = collections.defaultdict(list)
    stops = collections.defaultdict(list)
    for net, rs in by[meth].items():
        for r in rs:
            o = opt[(net, r["t0"])]
            gap[net].append((float(r["time"]) - o) / o * 100.0)
            wait[net].append(float(r["wait"]))
            stops[net].append(float(r["stops"]))
    emit("Table 1 (base scenario)", meth, "gap vs TD-OPT (%)", gap)
    emit("Table 1 (base scenario)", meth, "wait (s)", wait)
    emit("Table 1 (base scenario)", meth, "stops", stops)

# ------------------------------------------------- scenario ablation table
SCEN = ["base_N8", "no_rtor", "left_first", "sync_zero", "progressive",
        "grid_N5", "grid_N12"]
for sc in SCEN:
    sby = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in rows:
        if r["scenario"] != sc or r["sigma"] not in ("", None):
            continue
        sby[r["method"]][r["net"]].append(r)
    if "TD-OPT" not in sby:
        continue
    sopt = {}
    for net, rs in sby["TD-OPT"].items():
        for r in rs:
            sopt[(net, r["t0"])] = float(r["time"])
    for meth in ("SP-STATIC", "GREEDY"):
        gap = collections.defaultdict(list)
        for net, rs in sby.get(meth, {}).items():
            for r in rs:
                o = sopt[(net, r["t0"])]
                gap[net].append((float(r["time"]) - o) / o * 100.0)
        emit("Scenario ablations", sc, f"{meth} gap (%)", gap)
    tdo = collections.defaultdict(list)
    for net, rs in sby["TD-OPT"].items():
        for r in rs:
            tdo[net].append(float(r["time"]))
    emit("Scenario ablations", sc, "TD-OPT time (s)", tdo)

# ------------------------------------------------------ VOI (accuracy) rows
# The VOI scenario stores only the TD-NOISY runs; its reference optimum is the
# TD-OPT run of base_N8 on the same network and departure (the VOI networks are
# the first 40 of the 60). Planning at sigma = 0 is planning on the truth, so
# TD-NOISY at sigma = 0 must reproduce TD-OPT exactly; that is asserted below.
vby = collections.defaultdict(lambda: collections.defaultdict(list))
vopt = {}
for r in rows:
    if r["scenario"] == "base_N8" and r["method"] == "TD-OPT":
        vopt[(r["net"], r["t0"])] = float(r["time"])
_chk = [abs(float(r["time"]) - vopt[(r["net"], r["t0"])])
        for r in rows if r["scenario"] == "VOI" and r["sigma"] == "0"
        and (r["net"], r["t0"]) in vopt]
assert _chk and max(_chk) < 1e-6, f"sigma=0 does not reproduce TD-OPT (max {max(_chk)})"
print(f"VOI check: sigma = 0 reproduces TD-OPT on {len(_chk)} instances "
      f"(max deviation {max(_chk):.2e} s)")
for r in rows:
    if r["scenario"] != "VOI" or r["method"] != "TD-NOISY":
        continue
    vby[r["sigma"]][r["net"]].append(r)
for sig in sorted(vby, key=lambda s: float(s)):
    gap = collections.defaultdict(list)
    for net, rs in vby[sig].items():
        for r in rs:
            o = vopt.get((net, r["t0"]))
            if o:
                gap[net].append((float(r["time"]) - o) / o * 100.0)
    emit("VOI (accuracy sweep)", f"sigma = {sig} s", "gap vs TD-OPT (%)", gap)

# ------------------------------- Table 3: LA-k under countdown error
C, gs, gl = 120.0, 40.0, 20.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8
orig, dest = (N - 1, 0), (0, N - 1)
deps = [q * C / 8 for q in range(8)]
KS = [1, 2, 3, 4, 5]
SIGS = [0, 5, 10]
lak = {(k, s): collections.defaultdict(list) for k in KS for s in SIGS}
optv = collections.defaultdict(list)
statv = collections.defaultdict(list)
for i in range(40):
    rng = random.Random(1000 + i)
    net = sr.Net(N, rng, C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
    V = xl.static_V(net, tm, dest)
    p_st = sr.static_route(net, tm, orig, dest, use_ewait=True)
    prng = random.Random(70000 + i)
    pofs = {s: ({kk: (o + prng.uniform(-s, s)) % C for kk, o in net.offset.items()}
                if s > 0 else net.offset) for s in SIGS}
    for t0 in deps:
        op, _ = sr.td_route(net, tm, orig, dest, t0)
        o = sr.evaluate(net, tm, op, t0)["time"]
        optv[i].append(o)
        statv[i].append((sr.evaluate(net, tm, p_st, t0)["time"] - o) / o * 100.0)
        for k in KS:
            for s in SIGS:
                T = xl.la_route_noisy(net, tm, orig, dest, t0, k, V, pofs[s]) - t0
                lak[(k, s)][i].append((T - o) / o * 100.0)
emit("Table 3 (LA-k x sigma)", "k = 0 (SP-STATIC)", "sigma = 0 s", statv)
for k in KS:
    for s in SIGS:
        emit("Table 3 (LA-k x sigma)", f"k = {k}", f"sigma = {s} s", lak[(k, s)])

# ------------------------------------------- MDP gap and LA-1 minus MDP
import mdp_policy as mp                      # runs its own suite on import
mdp_gap = collections.defaultdict(list)
la1_minus_mdp = collections.defaultdict(list)
for i in range(40):
    rng = random.Random(1000 + i)
    net = sr.Net(N, rng, C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
    V, idx, moves, links, _ = mp.solve_mdp(net)
    Vs = static_V(net, tm, dest)
    for t0 in deps:
        op, _ = sr.td_route(net, tm, orig, dest, t0)
        o = sr.evaluate(net, tm, op, t0)["time"]
        T, _rv = mp.mdp_sim(net, V, idx, moves, t0)
        mdp_gap[i].append((T - o) / o * 100.0)
        la1 = sr.evaluate(net, tm, la_route(net, tm, orig, dest, t0, 1, Vs), t0)["time"]
        la1_minus_mdp[i].append(la1 - T)
emit("Proposition 7", "MDP", "gap vs TD-OPT (%)", mdp_gap)
emit("Proposition 7", "LA-1 minus MDP", "paired difference (s)", la1_minus_mdp)

# --------------------------------------------------------------- output
lines = []
lines.append("Network-clustered 95 percent intervals for the grid study")
lines.append("The mean is the pooled mean and does not change; the interval is")
lines.append("recomputed over network means (the eight departures of a network")
lines.append("are aggregated first). ratio = clustered / per-instance.")
lines.append("")
hdr = f"{'table':26s} {'row':20s} {'column':24s} {'mean':>10s} {'+- inst':>9s} {'+- clust':>9s} {'ratio':>6s} {'n':>5s} {'nets':>5s}"
lines.append(hdr)
lines.append("-" * len(hdr))
last = None
for r in OUT:
    t = r["table"] if r["table"] != last else ""
    last = r["table"]
    lines.append(f"{t:26s} {r['row']:20s} {r['column']:24s} {r['mean']:10.2f} "
                 f"{r['ci_instance']:9.2f} {r['ci_clustered']:9.2f} {r['ratio']:6.2f} "
                 f"{r['n_instances']:5d} {r['n_clusters']:5d}")
ratios = [r["ratio"] for r in OUT if r["ratio"] == r["ratio"]]
lines.append("")
lines.append(f"ratio over {len(ratios)} cells: min {min(ratios):.2f}, median "
             f"{sorted(ratios)[len(ratios)//2]:.2f}, max {max(ratios):.2f}")
txt = "\n".join(lines)
open("clustered_intervals.txt", "w").write(txt + "\n")
json.dump({"method": "network-clustered normal approximation; departures of a "
                     "network aggregated before the interval is taken",
           "z": Z, "cells": OUT}, open("clustered_intervals.json", "w"), indent=1)
print(txt)
