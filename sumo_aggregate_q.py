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


LAK_PRED = {"LA1": 1, "LA2": 2, "LA3": 3}


def analytic_context(j=None):
    """Analytical network and its static and Proposition 7 values: network j
    of the v2 sample (offsets from random.Random(1000 + j)), or with j None the
    network sumo_runner built at import (SR_NETSEED, default network 0)."""
    import sumo_runner as R
    import queue_model as qm
    from lookahead import static_V
    if j is None:
        return {"anet": R.ANET, "vstat": R.VSTAT, "vmdp": R.VMDP}
    anet = R.build_anet(j)
    v0, _ = qm.solve_values(anet, R.TM, R.DEST, None, R.sedge)
    return {"anet": anet, "vstat": static_V(anet, R.TM, R.DEST), "vmdp": {e: float(v[0]) for e, v in v0.items()}}


def model_pred(m, t0, ctx=None):
    """Analytical travel time of method m departing at t0 on the network of ctx
    (analytic_context), no queues and no startup constant. TDOPT_open,
    TDOPT_replan and TDOPTQ coincide in the model (deterministic, no queues)."""
    import signal_routing as sr
    import queue_model as qm
    from lookahead import la_route
    from sumo_runner import TM, ORIG, DEST
    ctx = ctx or analytic_context()
    ANET, VSTAT, VMDP = ctx["anet"], ctx["vstat"], ctx["vmdp"]
    if m == "STATIC":
        p = sr.static_route(ANET, TM, ORIG, DEST, use_ewait=True)
        return sr.evaluate(ANET, TM, p, t0)["time"]
    if m == "GREEDY":
        return sr.evaluate(ANET, TM, sr.greedy_route(ANET, TM, ORIG, DEST, t0), t0)["time"]
    if m in LAK_PRED:
        return sr.evaluate(ANET, TM, la_route(ANET, TM, ORIG, DEST, t0, LAK_PRED[m], VSTAT), t0)["time"]
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
    if m in ("TDOPT_open", "TDOPT_replan", "TDOPTQ"):
        p, _ = sr.td_route(ANET, TM, ORIG, DEST, t0)
        return sr.evaluate(ANET, TM, p, t0)["time"]
    raise ValueError(f"no analytical prediction for {m}")


def main():
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
        # E. value calibration: planned remaining time of the chosen option against the
        #    realised remaining time, by number of crossings already decided
        if dg:
            rows_e = {}
            for m in ("MDP", "STATICQ", "MDPQ", "TDOPTQ"):
                if m not in dg:
                    continue
                trips = [[r for r in dep if "cross" in r and r.get("plan_arr") and r.get("arr")] for sd in dg[m] for dep in dg[m][sd]]
                trips = [tr for tr in trips if tr]
                if not trips:
                    continue
                per_k = []
                for k in range(13):
                    pl = [tr[k]["plan_arr"] - tr[k]["t_free"] for tr in trips if k < len(tr)]
                    re = [tr[k]["arr"] - tr[k]["t_free"] for tr in trips if k < len(tr)]
                    if pl:
                        per_k.append({"decision": k + 1, "planned": sum(pl) / len(pl), "realised": sum(re) / len(re), "n": len(pl)})
                rows_e[m] = per_k
            if rows_e:
                print("E. value calibration: planned vs realised remaining time (s) at decision k (both exclude the per-crossing startup constant delta on the planned side)")
                ks = [1, 4, 7, 10, 13]
                print("   " + f"{'method':14s}" + "".join(f"{'k=' + str(k):>22s}" for k in ks))
                for m, per_k in rows_e.items():
                    cells = []
                    for k in ks:
                        rec = next((x for x in per_k if x["decision"] == k), None)
                        cells.append(f"{rec['planned']:7.0f}/{rec['realised']:5.0f} ({rec['realised'] - rec['planned']:+4.0f})" if rec else " " * 22)
                    print("   " + f"{NAMES[m]:14s}" + "".join(f"{c:>22s}" for c in cells))
                lev["value_calibration"] = rows_e
        summary["levels"][label] = lev

    # D. empty net
    emp_path = os.path.join(RES, "empty.json")
    if os.path.exists(emp_path):
        from sumo_runner import T0S

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


if __name__ == "__main__":
    main()
