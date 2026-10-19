"""Aggregate the v2 sample of record (Task 4 and Task 5 of the v2 work order).

Usage: python aggregate_v2.py [--res results_v2]
Writes results_v2/summary_v2.json (every number; keys documented in
results_v2/README_v2.md) and results_v2/tables_v2.txt (human-readable).

Unit of inference: the network. For every cell the trips of each network are
averaged first and the interval is a t interval over the network means with
(number of networks - 1) degrees of freedom. Secondary intervals cluster on
(network, seed) and on the trip. Every interval is reported as a 95 percent
half-width. Paired differences are matched on (network, seed, departure);
a trip with no arrival (None) drops out of every comparison it enters."""
import argparse
import glob
import json
import math
import os
import re
import time

from scipy import stats

ap = argparse.ArgumentParser()
ap.add_argument("--res", default="results_v2")
A = ap.parse_args()
RES = A.res
DIRS = [("shared", 45), ("channelized", 45), ("shared", 100), ("channelized", 100)]
T0S = [0.0, 30.0, 60.0, 90.0]
C = 120.0
NCROSS = 13                      # crossings of a corner-to-corner monotone trip on the 8 x 8 grid
ORDER = ["STATIC", "GREEDY", "LA1", "LA2", "LA3", "MDP", "TDOPT_open", "TDOPT_replan", "TDOPTQ",
         "STATICQ", "MDPQ", "MDPQOP", "STATICTRAFFIC", "STATICTRAFFICRT"]
ROLLING = ["GREEDY", "LA1", "LA2", "LA3", "MDP", "TDOPT_replan", "STATICQ", "MDPQ", "MDPQOP", "TDOPTQ"]
BASES = ["STATIC", "LA1", "TDOPTQ", "TDOPT_replan"]
EXTRA_PAIRS = [("TDOPTQ", "TDOPT_replan"), ("STATICQ", "STATIC"), ("MDPQ", "MDP"), ("MDPQOP", "MDP"),
               ("MDPQOP", "MDPQ"), ("LA1", "MDP"), ("LA2", "LA1"), ("LA3", "LA1")]
EMPTY_METHODS = ["STATIC", "GREEDY", "LA1", "LA2", "LA3", "MDP", "TDOPT_open", "TDOPT_replan", "TDOPTQ"]
FNAME = re.compile(r"^dem(\d+)_(.+)_n(\d+)_(\d+)\.json$")


# ------------------------------------------------------------------ statistics

def tci(xs):
    """Mean and 95 percent t half-width with len(xs) - 1 degrees of freedom."""
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(xs) / n
    if n < 2:
        return m, float("nan")
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))
    return m, stats.t.ppf(0.975, n - 1) * sd / math.sqrt(n)


def sd_of(xs):
    n = len(xs)
    if n < 2:
        return float("nan")
    m = sum(xs) / n
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))


def clustered(rows, key):
    """rows: list of (cluster key tuple, value); t half-width over cluster means."""
    by = {}
    for k, v in rows:
        by.setdefault(key(k), []).append(v)
    ms = [sum(v) / len(v) for v in by.values()]
    return tci(ms)[1], len(ms), {str(k): sum(v) / len(v) for k, v in by.items()}


def describe(rows):
    """rows: [((net, seed, dep), value)] -> mean and the three intervals."""
    vals = [v for _, v in rows]
    m, ci_t = tci(vals)
    ci_n, kn, by_net = clustered(rows, lambda k: k[0])
    ci_ns, kns, _ = clustered(rows, lambda k: (k[0], k[1]))
    return {"n": len(vals), "mean": m, "ci_net": ci_n, "n_nets": kn, "ci_netseed": ci_ns,
            "n_netseed": kns, "ci_trip": ci_t, "net_means": by_net}


# ------------------------------------------------------------------ loading

def load_dir(d):
    out, diag, meta = {}, {}, {}
    for f in glob.glob(os.path.join(d, "dem*_n*_*.json")):
        mt = FNAME.match(os.path.basename(f))
        if not mt:
            continue
        den, m, j, s = int(mt.group(1)), mt.group(2), int(mt.group(3)), int(mt.group(4))
        out.setdefault((den, m), {})[(j, s)] = json.load(open(f))
        g = os.path.join(d, "diag_" + os.path.basename(f))
        if os.path.exists(g):
            diag.setdefault((den, m), {})[(j, s)] = json.load(open(g))
        g = os.path.join(d, "meta_" + os.path.basename(f))
        if os.path.exists(g):
            meta.setdefault((den, m), {})[(j, s)] = json.load(open(g))
    return out, diag, meta


def trips(cell):
    """{(net, seed): [t0..t3]} -> {(net, seed, dep): time} without missing trips."""
    return {(j, s, i): x for (j, s), xs in cell.items() for i, x in enumerate(xs) if x is not None}


def paired(ca, cb):
    a, b = trips(ca), trips(cb)
    keys = sorted(set(a) & set(b))
    if not keys:
        return None
    rows = [(k, a[k] - b[k]) for k in keys]
    r = describe(rows)
    r.pop("net_means")
    r["a_faster"] = sum(1 for _, v in rows if v < 0)
    r["ties"] = sum(1 for _, v in rows if v == 0)
    return r


DATA = {}
for lay, dec in DIRS:
    d = os.path.join(RES, f"{lay}_d{dec}")
    if os.path.isdir(d):
        DATA[(lay, dec)] = load_dir(d)

S = {"meta": {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "results_dir": RES,
              "interval": "95 percent half-width; ci_net: t over network means (n_nets - 1 df); "
                          "ci_netseed: t over (network, seed) means; ci_trip: t over trips"},
     "means": [], "paired": [], "shortfall": [], "crossing_diag": [], "value_calibration": [],
     "traffic_routers": [], "empty_validation": {}, "cross_check": {}, "dem0_check": {},
     "run_health": [], "kernels": []}
T = []      # lines of tables_v2.txt


def emit(s=""):
    T.append(s)


nets_all = sorted({j for (o, dg, mt) in DATA.values() for c in o.values() for (j, s) in c})
seeds_all = sorted({s for (o, dg, mt) in DATA.values() for c in o.values() for (j, s) in c})
S["meta"].update({"nets": nets_all, "seeds": seeds_all, "K": len(nets_all)})
emit(f"v2 sample of record: networks {nets_all}, seeds {seeds_all}  (generated {S['meta']['generated']})")
emit("Intervals are 95 percent half-widths. net = network-clustered t interval (K-1 df, primary); "
     "n x s = network-by-seed clusters; trip = per trip.")

# ------------------------------------------------------------------ 5.1 and 5.2

for (lay, dec), (O, DG, MT) in DATA.items():
    levels = sorted({den for den, _ in O}, key=lambda x: (0 if x == 0 else 1, -x))
    for den in levels:
        # demand-0 STATICQ/MDPQ are the one-cell identity check (network 0, seed 11), reported in dem0_check only
        meths = [m for m in ORDER if (den, m) in O and not (den == 0 and m in ("STATICQ", "MDPQ"))]
        emit("")
        emit(f"=== {lay} lanes, decision {dec} m, demand {'0 (empty)' if den == 0 else '1/' + str(den)} ===")
        emit("5.1 mean travel time (s)")
        emit(f"   {'method':16s} {'n':>4s} {'miss':>4s} {'mean':>8s} {'net':>7s} {'n x s':>7s} {'trip':>7s}")
        for m in meths:
            rows = sorted(trips(O[(den, m)]).items())
            miss = sum(1 for xs in O[(den, m)].values() for x in xs if x is None)
            r = describe(rows)
            r.update({"layout": lay, "decide": dec, "den": den, "method": m, "n_missing": miss})
            S["means"].append(r)
            emit(f"   {m:16s} {r['n']:4d} {miss:4d} {r['mean']:8.1f} {r['ci_net']:7.1f} {r['ci_netseed']:7.1f} {r['ci_trip']:7.1f}")
        pairs = []
        for b in BASES:
            pairs += [(a, b) for a in meths if a != b and (den, b) in O]
        pairs += [(a, b) for a, b in EXTRA_PAIRS if a in meths and b in meths and (a, b) not in pairs]
        emit("5.2 paired differences a - b (s), matched on (network, seed, departure)")
        emit(f"   {'a':16s} {'b':14s} {'mean':>7s} {'net':>6s} {'n x s':>6s} {'trip':>6s} {'a faster':>9s}")
        for a, b in pairs:
            r = paired(O[(den, a)], O[(den, b)])
            if r is None:
                continue
            r.update({"layout": lay, "decide": dec, "den": den, "a": a, "b": b})
            S["paired"].append(r)
            emit(f"   {a:16s} {b:14s} {r['mean']:+7.1f} {r['ci_net']:6.1f} {r['ci_netseed']:6.1f} {r['ci_trip']:6.1f} {r['a_faster']:4d}/{r['n']:<4d}")
        # 5.3 one-step shortfall
        if (den, "LA1") in O and len(meths) > 1:
            mm = {m: next(x for x in S["means"] if (x["layout"], x["decide"], x["den"], x["method"]) == (lay, dec, den, m))["mean"]
                  for m in meths}
            best = min(mm, key=mm.get)
            vs = {}
            for m in meths:
                if m == "LA1":
                    continue
                r = paired(O[(den, "LA1")], O[(den, m)])
                tb = trips(O[(den, m)]); ta = trips(O[(den, "LA1")])
                keys = sorted(set(ta) & set(tb))
                base_mean = sum(tb[k] for k in keys) / len(keys)
                vs[m] = {"la1_minus": r["mean"], "ci_net": r["ci_net"], "n": r["n"],
                         "shortfall_pct": 100.0 * r["mean"] / base_mean, "other_mean_matched": base_mean}
            rec = {"layout": lay, "decide": dec, "den": den, "best_method": best, "best_mean": mm[best],
                   "la1_mean": mm["LA1"], "la1_is_best": best == "LA1",
                   "la1_shortfall_pct": 0.0 if best == "LA1" else vs[best]["shortfall_pct"],
                   "la1_minus_best": None if best == "LA1" else {"mean": vs[best]["la1_minus"], "ci_net": vs[best]["ci_net"]},
                   "vs": vs}
            S["shortfall"].append(rec)
            emit(f"5.3 one-step shortfall: lowest mean {best} ({mm[best]:.1f} s); LA1 shortfall "
                 + ("0 (LA1 is lowest)" if best == "LA1" else
                    f"{rec['la1_shortfall_pct']:+.1f} % (LA1 - {best} {vs[best]['la1_minus']:+.1f} +- {vs[best]['ci_net']:.1f} s, net)"))
            emit("   LA1 against each method: " + "; ".join(f"{m} {v['shortfall_pct']:+.1f} % ({v['la1_minus']:+.1f} +- {v['ci_net']:.1f})" for m, v in vs.items()))

# ------------------------------------------------------------------ 5.4 crossing diagnostics

emit("")
emit("5.4 crossing diagnostics at the decided stopline: realised minus planned crossing time (s)")
emit(f"   {'layout':12s} {'dec':>3s} {'den':>4s} {'method':13s} {'mv':2s} {'n':>5s} {'q_ahead':>7s} {'mean':>7s} {'sd':>6s}")
for (lay, dec), (O, DG, MT) in DATA.items():
    for den in (25, 15, 10):
        for m in ROLLING:
            if (den, m) not in DG:
                continue
            recs = [r for (j, s), deps in DG[(den, m)].items() for dep in deps for r in dep if "cross" in r]
            for mv in "SLR":
                rs = [r for r in recs if r["move"] == mv]
                if not rs:
                    continue
                err = [r["cross"] - r["pred"] for r in rs]
                rec = {"layout": lay, "decide": dec, "den": den, "method": m, "move": mv, "n": len(rs),
                       "q_ahead_mean": sum(r["q_ahead"] for r in rs) / len(rs),
                       "err_mean": sum(err) / len(err), "err_sd": sd_of(err)}
                S["crossing_diag"].append(rec)
                emit(f"   {lay:12s} {dec:3d} {den:4d} {m:13s} {mv:2s} {rec['n']:5d} {rec['q_ahead_mean']:7.2f} {rec['err_mean']:+7.1f} {rec['err_sd']:6.1f}")

# ------------------------------------------------------------------ Task 4 / 5.7 empty network

kappa = {}
if ("shared", 45) in DATA:
    import sumo_aggregate_q as SA
    O = DATA[("shared", 45)][0]
    ctx = {}
    pred = {}       # (method, net, seed, dep) -> (sumo, model)
    for m in EMPTY_METHODS:
        for (j, s), xs in O.get((0, m), {}).items():
            if j not in ctx:
                ctx[j] = SA.analytic_context(j)
            for i, x in enumerate(xs):
                t0 = (T0S[i] + 12.0 * (s - 11)) % C
                pred[(m, j, s, i)] = (x, SA.model_pred(m, t0, ctx[j]))
    fold = lambda j: "A" if j % 2 == 0 else "B"
    for F in "AB":
        d = [(x - y) / NCROSS for (m, j, s, i), (x, y) in pred.items() if m == "TDOPT_replan" and fold(j) == F and x is not None]
        kappa[F] = sum(d) / len(d) if d else float("nan")
        kappa["n_" + F] = len(d)
    dall = [(x - y) / NCROSS for (m, j, s, i), (x, y) in pred.items() if m == "TDOPT_replan" and x is not None]
    kappa["all"] = sum(dall) / len(dall) if dall else float("nan")
    ev = {}
    emit("")
    emit(f"5.7 empty-network validation (Task 4): kappa_A = {kappa['A']:.3f} s (fit on fold A, even networks, "
         f"{kappa['n_A']} TDOPT_replan trips), kappa_B = {kappa['B']:.3f} s ({kappa['n_B']} trips); pooled {kappa['all']:.3f} s")
    emit("   deviation of SUMO from the analytical prediction + 13 kappa of the other fold, percent of that prediction")
    emit(f"   {'method':14s} {'n':>4s} {'mean %':>8s} {'net':>6s} {'trip':>6s} {'fold A %':>9s} {'fold B %':>9s}")
    for m in EMPTY_METHODS:
        rows = []
        for (mm, j, s, i), (x, y) in sorted(pred.items()):
            if mm != m or x is None:
                continue
            k = kappa["B" if fold(j) == "A" else "A"]
            p = y + NCROSS * k
            rows.append(((j, s, i), 100.0 * (x - p) / p))
        if not rows:
            continue
        r = describe(rows)
        r["mean_fold_A"] = tci([v for k, v in rows if fold(k[0]) == "A"])[0]
        r["mean_fold_B"] = tci([v for k, v in rows if fold(k[0]) == "B"])[0]
        r["n_missing"] = sum(1 for (mm, j, s, i), (x, y) in pred.items() if mm == m and x is None)
        ev[m] = r
        emit(f"   {m:14s} {r['n']:4d} {r['mean']:+8.2f} {r['ci_net']:6.2f} {r['ci_trip']:6.2f} {r['mean_fold_A']:+9.2f} {r['mean_fold_B']:+9.2f}")
    S["empty_validation"] = {"kappa_A": kappa["A"], "kappa_B": kappa["B"], "kappa_pooled": kappa["all"],
                             "n_fit_A": kappa["n_A"], "n_fit_B": kappa["n_B"], "n_crossings": NCROSS,
                             "methods": ev,
                             "trips": [{"method": m, "net": j, "seed": s, "dep": i, "t0": (T0S[i] + 12.0 * (s - 11)) % C,
                                        "sumo": x, "model": y} for (m, j, s, i), (x, y) in sorted(pred.items())]}

# ------------------------------------------------------------------ 5.5 value calibration by action

if ("shared", 45) in DATA and kappa:
    O, DG, MT = DATA[("shared", 45)]
    emit("")
    emit("5.5 value calibration by action, shared lanes, decision 45 m: planned - realised remaining time + kappa x remaining crossings (s)")
    emit("   kappa of the other network fold (Task 4); grouped by the movement chosen and the count ahead at the decision")
    emit(f"   {'den':>4s} {'method':8s} {'mv':2s} {'ahead':>5s} {'n':>5s} {'mean':>8s} {'net':>7s}")
    for den in (25, 15, 10):
        for m in ("MDP", "MDPQ", "MDPQOP", "TDOPTQ"):
            if (den, m) not in DG:
                continue
            rows = {}
            for (j, s), deps in DG[(den, m)].items():
                k = kappa["B" if j % 2 == 0 else "A"]
                for i, dep in enumerate(deps):
                    recs = [r for r in dep if r.get("plan_arr") is not None and r.get("arr") is not None]
                    nrec = len(dep)
                    for idx, r in enumerate(dep):
                        if r.get("plan_arr") is None or r.get("arr") is None:
                            continue
                        rem = nrec - idx
                        v = r["plan_arr"] - r["arr"] + k * rem
                        g = (r["move"], "0" if r["q_ahead"] == 0 else ">=1")
                        rows.setdefault(g, []).append(((j, s, i), v))
            for mv in "SLR":
                for qg in ("0", ">=1"):
                    if (mv, qg) not in rows:
                        continue
                    r = describe(rows[(mv, qg)])
                    r.pop("net_means")
                    r.update({"den": den, "method": m, "move": mv, "ahead": qg})
                    S["value_calibration"].append(r)
                    emit(f"   {den:4d} {m:8s} {mv:2s} {qg:>5s} {r['n']:5d} {r['mean']:+8.1f} {r['ci_net']:7.1f}")

# ------------------------------------------------------------------ 5.6 traffic-aware routers

if ("shared", 45) in DATA:
    O, DG, MT = DATA[("shared", 45)]
    emit("")
    emit("5.6 traffic-aware routers, shared lanes, decision 45 m (paired as in 5.2)")
    for den in (25, 15, 10):
        for a in ("STATICTRAFFIC", "STATICTRAFFICRT"):
            if (den, a) not in O or (den, "STATIC") not in O:
                continue
            r = paired(O[(den, a)], O[(den, "STATIC")])
            same = tot = 0
            if a == "STATICTRAFFICRT":
                for js, mt in MT.get((den, a), {}).items():
                    ms = MT.get((den, "STATIC"), {}).get(js)
                    if not ms:
                        continue
                    for ta, tb in zip(mt["trips"], ms["trips"]):
                        tot += 1
                        same += ta["route"] == tb["route"]
            r.update({"den": den, "a": a, "b": "STATIC", "route_equals_static": same if a == "STATICTRAFFICRT" else None,
                      "route_compared": tot if a == "STATICTRAFFICRT" else None})
            S["traffic_routers"].append(r)
            emit(f"   1/{den:<3d} {a:16s} - STATIC {r['mean']:+7.1f} net {r['ci_net']:5.1f} n x s {r['ci_netseed']:5.1f} trip {r['ci_trip']:5.1f}"
                 f"  a faster {r['a_faster']}/{r['n']}" + (f"  route = STATIC route in {same}/{tot} trips" if a == "STATICTRAFFICRT" else ""))

# ------------------------------------------------------------------ 5.8 cross-check with sumo_results_depth

if ("shared", 45) in DATA:
    O = DATA[("shared", 45)][0]
    diffs, ncmp = [], 0
    for den in (0, 100, 50, 25, 15, 10):
        for m in ("STATIC", "LA1", "LA2", "LA3", "TDOPT_replan", "TDOPTQ"):
            for s in (11, 12, 13, 14):
                f = f"sumo_results_depth/dem{den}_{m}_{s}.json"
                new = O.get((den, m), {}).get((0, s))
                if new is None or not os.path.exists(f):
                    continue
                old = json.load(open(f))
                for i, (x, y) in enumerate(zip(new, old)):
                    ncmp += 1
                    if x != y:
                        diffs.append({"den": den, "method": m, "seed": s, "dep": i, "v2": x, "depth": y})
    S["cross_check"] = {"n_trips_compared": ncmp, "n_differences": len(diffs), "differences": diffs}
    emit("")
    emit(f"5.8 cross-check, network 0, seeds 11-14, against sumo_results_depth: {ncmp} trips compared, {len(diffs)} differ")
    for d in diffs:
        emit(f"   dem{d['den']} {d['method']} seed {d['seed']} dep {d['dep']}: v2 {d['v2']} depth {d['depth']}")
    # demand-0 identity check of STATIC-Q / MDP-Q (network 0, seed 11)
    chk = {}
    MT = DATA[("shared", 45)][2]
    for a, b in (("STATICQ", "STATIC"), ("MDPQ", "MDP")):
        x, y = O.get((0, a), {}).get((0, 11)), O.get((0, b), {}).get((0, 11))
        if x is None or y is None:
            continue
        ra = [t["route"] for t in MT[(0, a)][(0, 11)]["trips"]]
        rb = [t["route"] for t in MT[(0, b)][(0, 11)]["trips"]]
        chk[f"{a}-{b}"] = {"a": x, "b": y, "times_identical": x == y, "routes_identical": ra == rb}
    S["dem0_check"] = chk
    for k, v in chk.items():
        emit(f"   demand-0 check {k} (network 0, seed 11): {v['a']} vs {v['b']}  times identical {v['times_identical']}, routes identical {v['routes_identical']}")

# ------------------------------------------------------------------ run health and kernels

emit("")
emit("Run health: runs, trips, trips without arrival, trips with any teleport in the network, trips where the ego teleported, total wall time")
for (lay, dec), (O, DG, MT) in DATA.items():
    runs = sum(len(c) for c in O.values())
    tr = sum(len(xs) for c in O.values() for xs in c.values())
    none = [(den, m, j, s, i) for (den, m), c in O.items() for (j, s), xs in c.items() for i, x in enumerate(xs) if x is None]
    tele = sum(1 for c in MT.values() for mt in c.values() for t in mt["trips"] if t["teleports"] > 0)
    ego = sum(1 for c in MT.values() for mt in c.values() for t in mt["trips"] if t["ego_teleported"])
    wall = sum(mt["wall_s"] for c in MT.values() for mt in c.values())
    S["run_health"].append({"layout": lay, "decide": dec, "runs": runs, "trips": tr, "no_arrival": len(none),
                            "no_arrival_cells": [list(x) for x in none], "trips_with_teleports": tele,
                            "ego_teleported": ego, "wall_s_sum": wall})
    emit(f"   {lay:12s} d{dec:<3d} runs {runs:5d} trips {tr:5d} no arrival {len(none):3d} teleport trips {tele:4d} ego teleported {ego:3d} wall {wall / 3600:6.1f} core-h")
    for kf in sorted(glob.glob(os.path.join(RES, f"{lay}_d{dec}", "kernel_*.json"))):
        k = json.load(open(kf))
        S["kernels"].append({"layout": lay, "decide": dec, "file": os.path.basename(kf), "n_transitions": k["n_transitions"],
                             "estimation_nets": k.get("estimation_nets"), "planning_nets": k.get("planning_nets"),
                             "sources": k.get("sources"), "n_transitions_by_net": k.get("n_transitions_by_net")})
emit("Kernels: file, transitions, estimated on networks, used by networks, sources")
for k in S["kernels"]:
    emit(f"   {k['layout']:12s} d{k['decide']:<3d} {k['file']:28s} {k['n_transitions']:6d}  est {k['estimation_nets']}  used by {k['planning_nets']}  {k['sources']}")

json.dump(S, open(os.path.join(RES, "summary_v2.json"), "w"), indent=1)
open(os.path.join(RES, "tables_v2.txt"), "w").write("\n".join(T) + "\n")
print("\n".join(T))
print(f"\nwrote {RES}/summary_v2.json and {RES}/tables_v2.txt")
