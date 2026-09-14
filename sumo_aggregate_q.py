"""Aggregate the queue-aware congestion experiment (one SUMO build).

Usage: python3 sumo_aggregate_q.py [results_dir]   (default sumo_results)

Reads mod_<METHOD>_<seed>.json (moderate, 1/15), dem25_*.json (light) and
dem10_*.json (heavy), each a list of four departure travel times, and the
matching diag_*.json per-crossing prediction records. Prints:
  A. mean travel time per method and demand level (95 percent CI, n);
  B. paired differences on shared (seed, departure) instances, with the
     per-instance CI and the seed-clustered CI (departures share a background
     realisation, so the clustered interval is the conservative one);
  C. per-crossing prediction diagnostic: planned versus realised delay at the
     decided stopline, by method and movement;
  D. empty-net calibration, as in the paper (per-crossing constant delta).
Writes congestion_summary.json with every number printed."""
import glob
import json
import math
import os
import sys

RES = sys.argv[1] if len(sys.argv) > 1 else "sumo_results"
LEVELS = [("light", "dem25", 25), ("moderate", "mod", 15), ("heavy", "dem10", 10)]
NAMES = {"STATIC": "SP-STATIC", "GREEDY": "GREEDY", "LA1": "LA-1", "MDP": "MDP",
         "TDOPT_open": "TD-OPT open", "TDOPT_replan": "TD-OPT replan",
         "STATICQ": "SP-STATIC-Q", "MDPQ": "MDP-Q", "TDOPTQ": "TD-OPT-Q",
         "STATICTRAFFIC": "STATIC-TRAFFIC", "STATICTRAFFICRT": "STATIC-TRAFFIC-RT"}
ORDER = ["STATIC", "GREEDY", "LA1", "MDP", "TDOPT_open", "TDOPT_replan",
         "STATICQ", "MDPQ", "TDOPTQ", "STATICTRAFFIC", "STATICTRAFFICRT"]
PAIRS = [
    ("queue information, by phase regime", [("STATICQ", "STATIC"), ("MDPQ", "MDP"), ("TDOPTQ", "TDOPT_replan")]),
    ("phase information, queue-ignorant", [("MDP", "STATIC"), ("LA1", "STATIC"), ("TDOPT_replan", "STATIC"), ("TDOPT_replan", "MDP")]),
    ("phase information, queue-aware", [("MDPQ", "STATICQ"), ("TDOPTQ", "STATICQ"), ("TDOPTQ", "MDPQ")]),
    ("deployable rule against the queue-aware ladder", [("LA1", "MDP"), ("LA1", "STATICQ"), ("LA1", "MDPQ"), ("LA1", "TDOPTQ")]),
    ("traffic-aware baselines", [("STATICTRAFFICRT", "STATIC"), ("STATICTRAFFIC", "STATIC"), ("STATICQ", "STATICTRAFFICRT"), ("MDPQ", "STATICTRAFFICRT")]),
    ("moderate-only references", [("GREEDY", "STATIC"), ("TDOPT_replan", "TDOPT_open"), ("STATIC", "TDOPT_open")]),
]


def mean_ci(xs):
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(xs) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, 1.96 * math.sqrt(var / n)


def load(prefix):
    out = {}
    for f in glob.glob(os.path.join(RES, prefix + "_*.json")):
        p = os.path.basename(f)[:-5].split("_")
        meth = "_".join(p[1:-1])
        out.setdefault(meth, {})[int(p[-1])] = json.load(open(f))
    return out


def load_diag(prefix):
    out = {}
    for f in glob.glob(os.path.join(RES, "diag_" + prefix + "_*.json")):
        p = os.path.basename(f)[:-5].split("_")
        meth = "_".join(p[2:-1])
        out.setdefault(meth, {})[int(p[-1])] = json.load(open(f))
    return out


def paired(A, B, a, b):
    seeds = sorted(set(A.get(a, {})) & set(B.get(b, {})))
    inst, per_seed = [], []
    for sd in seeds:
        d = [x - y for x, y in zip(A[a][sd], B[b][sd]) if x and y]
        inst += d
        if d:
            per_seed.append(sum(d) / len(d))
    if not inst:
        return None
    m, ci = mean_ci(inst)
    mc, cic = mean_ci(per_seed)
    return {"mean": m, "ci": ci, "wins": sum(1 for x in inst if x < 0), "n": len(inst),
            "ci_clustered": cic, "n_seeds": len(per_seed)}


summary = {"levels": {}, "diagnostic": {}, "empty": {}}
for label, prefix, den in LEVELS:
    src = load(prefix)
    if not src:
        continue
    print(f"\n=== {label} demand (insertion probability 1/{den}) ===")
    print("A. mean travel time")
    lev = {"means": {}, "paired": {}}
    for m in ORDER:
        if m not in src:
            continue
        xs = [x for sd in sorted(src[m]) for x in src[m][sd] if x]
        miss = sum(1 for sd in src[m] for x in src[m][sd] if not x)
        t, ci = mean_ci(xs)
        lev["means"][m] = {"mean": t, "ci": ci, "n": len(xs), "missing": miss, "seeds": sorted(src[m])}
        print(f"   {NAMES[m]:18s} n={len(xs):3d}  {t:7.1f} +- {ci:5.1f}" + (f"   (missing {miss})" if miss else ""))
    print("B. paired differences a - b (s): per-instance CI | seed-clustered CI | a faster in")
    for title, pairs in PAIRS:
        rows = [(a, b, paired(src, src, a, b)) for a, b in pairs]
        rows = [(a, b, r) for a, b, r in rows if r is not None]
        if not rows:
            continue
        print(f"   -- {title}")
        for a, b, r in rows:
            lev["paired"][f"{a}-{b}"] = r
            print(f"   {NAMES[a]:16s} - {NAMES[b]:16s} {r['mean']:+7.1f} +- {r['ci']:5.1f} | +- {r['ci_clustered']:5.1f} ({r['n_seeds']} seeds) | {r['wins']:2d}/{r['n']}")
    # C. diagnostic
    dg = load_diag(prefix)
    if dg:
        print("C. prediction diagnostic at decided stoplines: planned vs realised delay (s)")
        print(f"   {'method':14s} {'mv':2s} {'n':>4s} {'q_ahead':>7s} {'planned':>8s} {'realised':>8s} {'error':>7s} {'sd':>6s}")
        lev["diagnostic"] = {}
        for m in ORDER:
            if m not in dg:
                continue
            rows = [r for sd in dg[m] for dep in dg[m][sd] for r in dep if "cross" in r]
            for mv in ("S", "L", "R"):
                rs = [r for r in rows if r["move"] == mv]
                if not rs:
                    continue
                pdl = [r["pred"] - r["t_free"] for r in rs]
                rdl = [r["cross"] - r["t_free"] for r in rs]
                err = [x - y for x, y in zip(rdl, pdl)]
                me, se = mean_ci(err)
                sd = se / 1.96 * math.sqrt(len(err)) if len(err) > 1 else 0.0
                rec = {"n": len(rs), "q_ahead": sum(r["q_ahead"] for r in rs) / len(rs),
                       "planned": sum(pdl) / len(pdl), "realised": sum(rdl) / len(rdl), "error": me, "sd": sd}
                lev["diagnostic"][f"{m}/{mv}"] = rec
                print(f"   {NAMES[m]:14s} {mv:2s} {rec['n']:4d} {rec['q_ahead']:7.2f} {rec['planned']:8.1f} {rec['realised']:8.1f} {rec['error']:+7.1f} {rec['sd']:6.1f}")
    summary["levels"][label] = lev

# D. empty net
emp_path = os.path.join(RES, "empty.json")
if os.path.exists(emp_path):
    import signal_routing as sr
    from lookahead import la_route
    from sumo_runner import ANET, TM, ORIG, DEST, VSTAT, T0S, VMDP
    import queue_model as qm

    def model_pred(m, t0):
        if m == "STATIC":
            p = sr.static_route(ANET, TM, ORIG, DEST, use_ewait=True)
            return sr.evaluate(ANET, TM, p, t0)["time"]
        if m == "GREEDY":
            return sr.evaluate(ANET, TM, sr.greedy_route(ANET, TM, ORIG, DEST, t0), t0)["time"]
        if m == "LA1":
            return sr.evaluate(ANET, TM, la_route(ANET, TM, ORIG, DEST, t0, 1, VSTAT), t0)["time"]
        if m == "MDP":
            best = min(((ANET.link_t(ORIG, w) + VMDP[(ORIG, w)], (ORIG, w)) for w, _ in ANET.nbrs(ORIG)))
            e = best[1]; t = t0 + ANET.link_t(*e); path = [e]
            while e[1] != DEST:
                u, v = e
                ax = sr.axis_of((v[0] - u[0], v[1] - u[1]))
                bm = (float("inf"), None)
                for mm, e2, _ in qm.monotone_moves(ANET, e, DEST):
                    sc = TM.wait(t, ANET.offset[v], ax, mm) + ANET.t_cross + ANET.link_t(*e2) + VMDP[e2]
                    if sc < bm[0] - 1e-9:
                        bm = (sc, e2)
                e = bm[1]; path.append(e)
                t = t + TM.wait(t, ANET.offset[e[0]], ax, sr.move_type((path[-2][1][0] - path[-2][0][0], path[-2][1][1] - path[-2][0][1]), (e[1][0] - e[0][0], e[1][1] - e[0][1]))) + ANET.t_cross + ANET.link_t(*e)
            return t - t0
        p, _ = sr.td_route(ANET, TM, ORIG, DEST, t0)
        return sr.evaluate(ANET, TM, p, t0)["time"]

    emp = json.load(open(emp_path))
    for extra in ("TDOPT_replan", "MDP"):
        f = os.path.join(RES, f"empty_one_{extra}.json")
        if os.path.exists(f):
            emp[extra] = json.load(open(f))
    dif = [s - model_pred("TDOPT_replan", t0) for s, t0 in zip(emp["TDOPT_replan"], T0S)]
    delta = sum(dif) / len(dif) / 13.0
    print(f"\n=== EMPTY NET: model vs SUMO (per-crossing constant delta = {delta:.2f} s) ===")
    summary["empty"]["delta"] = delta
    for m in ["STATIC", "GREEDY", "LA1", "MDP", "TDOPT_open", "TDOPT_replan"]:
        if m not in emp:
            continue
        devs = [100 * (s - (model_pred(m, t0) + 13 * delta)) / (model_pred(m, t0) + 13 * delta) for s, t0 in zip(emp[m], T0S)]
        dm, dci = mean_ci(devs)
        summary["empty"][m] = {"sumo": emp[m], "dev_pct": dm, "dev_ci": dci}
        print(f"   {NAMES[m]:14s} sumo {[int(x) for x in emp[m]]}  deviation after calibration {dm:+5.1f} % +- {dci:.1f}")

json.dump(summary, open(os.path.join(RES, "congestion_summary.json"), "w"), indent=1)
print(f"\nwrote {os.path.join(RES, 'congestion_summary.json')}")
