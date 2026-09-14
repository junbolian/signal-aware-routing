"""Aggregate the depth-by-demand sweep of run_depth_sweep.py.

Usage: python aggregate_depth.py [results_dir]   (default sumo_results_depth)
Prints, per demand level, the mean travel time of each method (per-instance
95 percent interval) and the paired differences against SP-STATIC and against
TD-OPT-Q with seed-clustered intervals and wins; writes depth_summary.json and
fig_depth_demand.png (paired difference to SP-STATIC against demand)."""
import glob, json, math, os, sys
RES = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SR_RESULTS", "sumo_results_depth")
LEVELS = [0, 100, 50, 25, 15, 10]
METHODS = ["STATIC", "LA1", "LA2", "LA3", "TDOPT_replan", "TDOPTQ"]
NAMES = {"STATIC": "SP-STATIC", "LA1": "LA-1", "LA2": "LA-2", "LA3": "LA-3", "TDOPT_replan": "TD-OPT replan", "TDOPTQ": "TD-OPT-Q"}

def mean_ci(xs):
    n = len(xs); m = sum(xs) / n
    if n < 2: return m, 0.0
    v = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, 1.96 * math.sqrt(v / n)

def load(den):
    out = {}
    for f in glob.glob(f"{RES}/dem{den}_*.json"):
        p = os.path.basename(f)[:-5].split("_"); meth = "_".join(p[1:-1])
        out.setdefault(meth, {})[int(p[-1])] = json.load(open(f))
    return out

def paired(src, a, b):
    seeds = sorted(set(src.get(a, {})) & set(src.get(b, {})))
    inst, per = [], []
    for sd in seeds:
        d = [x - y for x, y in zip(src[a][sd], src[b][sd]) if x and y]
        inst += d
        if d: per.append(sum(d) / len(d))
    if not inst: return None
    m, ci = mean_ci(inst); mc, cic = mean_ci(per)
    return {"mean": m, "ci": ci, "ci_clustered": cic, "wins": sum(1 for x in inst if x < 0), "n": len(inst), "n_seeds": len(per)}

summary = {}
for den in LEVELS:
    src = load(den)
    if not src: continue
    lab = "empty" if den == 0 else f"1/{den}"
    print(f"\n=== demand {lab} ===")
    lev = {"means": {}, "vs_static": {}, "vs_tdoptq": {}}
    for m in METHODS:
        if m not in src: continue
        xs = [x for sd in src[m] for x in src[m][sd] if x]
        t, ci = mean_ci(xs); lev["means"][m] = {"mean": t, "ci": ci, "n": len(xs)}
        print(f"   {NAMES[m]:14s} n={len(xs):3d} {t:7.1f} +- {ci:5.1f}")
    for m in METHODS:
        for ref, key in (("STATIC", "vs_static"), ("TDOPTQ", "vs_tdoptq")):
            if m == ref: continue
            r = paired(src, m, ref)
            if r:
                lev[key][m] = r
                print(f"   {NAMES[m]:14s} - {NAMES[ref]:13s} {r['mean']:+7.1f} +- {r['ci']:5.1f} | +- {r['ci_clustered']:5.1f} ({r['n_seeds']} seeds) | {r['wins']}/{r['n']}")
    summary[lab] = lev
json.dump(summary, open(f"{RES}/depth_summary.json", "w"), indent=1)
try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "mathtext.fontset": "cm", "font.size": 9, "axes.grid": True, "grid.alpha": 0.25, "legend.frameon": False, "figure.dpi": 300, "savefig.bbox": "tight"})
    COL = {"LA1": "#228833", "LA2": "#66AA55", "LA3": "#BBCC33", "TDOPT_replan": "#000000", "TDOPTQ": "#4477AA"}
    fig, ax = plt.subplots(figsize=(4.8, 3.0))
    labs = [k for k in summary]
    xs = range(len(labs))
    for m in ["LA1", "LA2", "LA3", "TDOPT_replan", "TDOPTQ"]:
        ys = [summary[l]["vs_static"].get(m, {}).get("mean", float("nan")) for l in labs]
        es = [summary[l]["vs_static"].get(m, {}).get("ci_clustered", 0.0) for l in labs]
        ax.errorbar(list(xs), ys, yerr=es, marker="o", ms=3, capsize=2, color=COL[m], label=NAMES[m])
    ax.set_xticks(list(xs)); ax.set_xticklabels(labs); ax.set_xlabel("background demand (insertion probability)")
    ax.set_ylabel("paired difference to SP-STATIC (s)"); ax.axhline(0, color="k", lw=0.6); ax.legend(fontsize=7)
    fig.savefig(f"{RES}/fig_depth_demand.png"); print("wrote figure")
except Exception as e:
    print("figure skipped:", e)
