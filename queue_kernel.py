"""Empirical transition kernel of the platoon count ahead of the ego.

The queue-aware unknown-offset planners (MDP-Q, STATIC-Q) need, for every
crossing they plan beyond the current one, the distribution of the number of
vehicles that will be ahead of the ego at the next stopline given the number
ahead now, the movement taken, and (for MDP-Q) the phase at which the crossing
happens. This is a property of the background traffic, not of the planner, and
a fleet operator estimates it from its probe history. Here it is estimated from
the per-crossing records of the queue-ignorant rolling planners (LA1, MDP,
TDOPT_replan, GREEDY) of the same demand level and build, written by
sumo_runner.py to sumo_results/diag_*.json: consecutive decisions of one trip
give one transition (q, move, phase bin) -> q'.

Phase bin of a crossing: position of the realised passage relative to the
cross-axis through window at that node: 0 = during the cross-axis through
green, 1 = within 40 s after it ended, 2 = later. Counts are pooled over
q >= QB_MAX and q' >= QK (the planner's state is capped there), smoothed with
half a count per cell, and the phase-conditioned rows fall back to the marginal
row when they hold fewer than MIN_N observations.

Cross-fitting: the kernel used for a seed is estimated from the other fold of
seeds (seeds 11-15 plan with the kernel estimated on seeds 16-20 and vice
versa), so a planner never uses statistics of its own background realisation.

Usage: python3 queue_kernel.py [results_dir]   writes <results_dir>/kernel_<prefix>_foldA.json
       (estimated on seeds 16-20, used by seeds 11-15) and kernel_<prefix>_foldB.json
       (estimated on seeds 11-15, used by seeds 16-20) for every prefix (mod,
       dem25, dem10) with diagnostics present.

Sample of record v2 (diag files tagged diag_dem<den>_<METHOD>_n<j>_<seed>.json):
folds are networks, fold A = even j, fold B = odd j. kernel_dem<den>[_<tag>]_foldA.json
is used by the networks of fold A and estimated on the crossings of fold B (and
vice versa), pooling every seed of every network of the estimating fold; each
crossing is binned with the offsets of its own network j.

Usage (v2): python3 queue_kernel.py <results_dir> [--levels 25 15 ...] [--nets 0 1 ... 9]
                                    [--sources MDPQ ...] [--tag op]
       --levels  demand denominators (default: every level with v2 diagnostics)
       --nets    networks the kernels may pool over (default: all present)
       --sources planner identifiers whose crossings are used (default LA1 MDP TDOPT_replan GREEDY)
       --tag     kernel name suffix, e.g. op -> kernel_dem<den>_op_fold<A|B>.json"""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np

SOURCES = ("LA1", "MDP", "TDOPT_replan", "GREEDY")
QB_MAX = 3            # q rows 0, 1, 2, 3+
QK = 8                # q' columns 0 .. 8+
NBIN = 3
MIN_N = 15
SMOOTH = 0.5


def phase_bin(since, g):
    if since < g:
        return 0
    return 1 if since < g + 40.0 else 2


FOLDS = {"A": (range(16, 21), range(11, 16)),   # (estimation seeds, planning seeds)
         "B": (range(11, 16), range(16, 21))}


def fold_of(seed):
    return "A" if seed <= 15 else "B"


def fold_of_net(j):
    """v2 network folds: A = even networks, B = odd networks."""
    return "A" if j % 2 == 0 else "B"


def other_fold(f):
    return "B" if f == "A" else "A"


def kernel_path(prefix, net, res):
    """Kernel used by network `net`: estimated on the other network fold."""
    return os.path.join(res, f"kernel_{prefix}_fold{fold_of_net(net)}.json")


def _count(counts, f, e2l, offset, axis_of, window, C):
    """Add the transitions of one diag file to counts, binned with `offset`."""
    MV = {"S": 0, "L": 1, "R": 2}
    n_obs = 0
    for dep in json.load(open(f)):
        for a, b in zip(dep, dep[1:]):
            if "cross" not in a:
                continue
            u, v = e2l(a["edge"])
            ax = axis_of((v[0] - u[0], v[1] - u[1]))
            cax = "EW" if ax == "NS" else "NS"
            s, g = window(cax, "S")
            P = (a["cross"] - offset[v]) % C
            pb = phase_bin((P - s) % C, g)
            counts[MV[a["move"]], pb, min(a["q_ahead"], QB_MAX), min(b["q_ahead"], QK)] += 1
            n_obs += 1
    return n_obs


def estimate(res, prefix, e2l, offset, axis_of, window, C, seeds=None):
    counts = np.full((3, NBIN, QB_MAX + 1, QK + 1), 0.0)   # move, bin, q, q'
    n_obs = 0
    for m in SOURCES:
        for f in glob.glob(os.path.join(res, f"diag_{prefix}_{m}_*.json")):
            sd = int(os.path.basename(f)[:-5].split("_")[-1])
            if seeds is not None and sd not in seeds:
                continue
            n_obs += _count(counts, f, e2l, offset, axis_of, window, C)
    out = _finish(counts, n_obs, SOURCES)
    out["estimation_seeds"] = (list(seeds) if seeds is not None else "all")
    return out


def estimate_nets(res, prefix, nets, offsets_of, e2l, axis_of, window, C, sources=SOURCES):
    """v2: pool the transitions of every seed of the networks `nets`, each
    binned with its own offsets (offsets_of(j)). Returns the kernel dict with
    the files and per-network transition counts it was estimated from."""
    counts = np.full((3, NBIN, QB_MAX + 1, QK + 1), 0.0)
    n_obs, files, per_net = 0, [], {}
    for j in nets:
        off = offsets_of(j)
        for m in sources:
            for f in sorted(glob.glob(os.path.join(res, f"diag_{prefix}_{m}_n{j}_*.json"))):
                k = _count(counts, f, e2l, off, axis_of, window, C)
                n_obs += k
                per_net[str(j)] = per_net.get(str(j), 0) + k
                files.append(os.path.basename(f))
    out = _finish(counts, n_obs, sources)
    out.update({"estimation_nets": list(nets), "n_transitions_by_net": per_net, "files": files})
    return out


def _finish(counts, n_obs, sources):
    marg = counts.sum(axis=1)                                   # move, q, q'
    K = np.empty((3, NBIN, QB_MAX + 1, QK + 1))
    Kmarg = np.empty((3, QB_MAX + 1, QK + 1))
    for mv in range(3):
        for q in range(QB_MAX + 1):
            row = marg[mv, q] + SMOOTH
            Kmarg[mv, q] = row / row.sum()
            for pb in range(NBIN):
                c = counts[mv, pb, q]
                if c.sum() >= MIN_N:
                    K[mv, pb, q] = (c + SMOOTH) / (c + SMOOTH).sum()
                else:
                    K[mv, pb, q] = Kmarg[mv, q]
    return {"K": K.tolist(), "K_marginal": Kmarg.tolist(), "n_transitions": int(n_obs),
            "counts": counts.tolist(), "sources": list(sources), "qb_max": QB_MAX, "qk": QK,
            "mean_qnext_by_move_bin": [[float((counts[mv, pb].sum(axis=0) * np.arange(QK + 1)).sum() / max(counts[mv, pb].sum(), 1))
                                         for pb in range(NBIN)] for mv in range(3)]}


class Kernel:
    """Loaded kernel with the two lookups the planners need."""
    MV = {"S": 0, "L": 1, "R": 2}

    def __init__(self, d):
        self.K = np.array(d["K"])
        self.Km = np.array(d["K_marginal"])
        self.n = d["n_transitions"]
        self.path = None

    def row(self, m, q, pb=None):
        q = min(q, QB_MAX)
        return self.Km[self.MV[m], q] if pb is None else self.K[self.MV[m], pb, q]


def load(prefix, seed=None, path=None, res="sumo_results", net=None):
    """Kernel for planning trips of `seed` (legacy seed folds) or on network
    `net` (v2 network folds): the fold estimated on the other seeds or
    networks, read from the results directory of the network being run."""
    if path is None and net is not None:
        path = kernel_path(prefix, net, res)
    path = path or os.path.join(res, f"kernel_{prefix}_fold{fold_of(seed)}.json" if seed is not None else f"kernel_{prefix}.json")
    kern = Kernel(json.load(open(path)))
    kern.path = path
    if kern.n < 200:
        raise RuntimeError(f"{path}: only {kern.n} transitions; run the queue-ignorant methods of the other fold first, then python3 queue_kernel.py")
    return kern


V2_DIAG = re.compile(r"^diag_dem(\d+)_(.+)_n(\d+)_(\d+)\.json$")


def main_v2(res, argv):
    import sumo_runner as R
    import signal_routing as sr
    ap = argparse.ArgumentParser()
    ap.add_argument("--levels", type=int, nargs="*")
    ap.add_argument("--nets", type=int, nargs="*")
    ap.add_argument("--sources", nargs="*", default=list(SOURCES))
    ap.add_argument("--tag", default="")
    a = ap.parse_args(argv)
    found = [V2_DIAG.match(os.path.basename(f)) for f in glob.glob(os.path.join(res, "diag_dem*_n*_*.json"))]
    found = [m for m in found if m and m.group(2) in a.sources]
    levels = a.levels if a.levels else sorted({int(m.group(1)) for m in found})
    nets = sorted(set(a.nets) if a.nets else {int(m.group(3)) for m in found})
    offs = {}

    def offsets_of(j):
        if j not in offs:
            offs[j] = R.build_anet(j).offset
        return offs[j]
    for den in levels:
        prefix = f"dem{den}"
        name = prefix + (f"_{a.tag}" if a.tag else "")
        for fold in ("A", "B"):
            est = [j for j in nets if fold_of_net(j) == other_fold(fold)]
            plan = [j for j in nets if fold_of_net(j) == fold]
            d = estimate_nets(res, prefix, est, offsets_of, R.e2l, sr.axis_of, R.TM._window, R.C, sources=a.sources)
            d.update({"fold": fold, "planning_nets": plan, "demand_den": den, "tag": a.tag})
            out = os.path.join(res, f"kernel_{name}_fold{fold}.json")
            json.dump(d, open(out, "w"))
            print(f"{out}: used by networks {plan}, estimated on networks {est} from {a.sources}: "
                  f"{d['n_transitions']} transitions ({len(d['files'])} files)", flush=True)
            for mv, nm in enumerate("SLR"):
                print(f"   {nm}: mean q' by phase bin " + "  ".join(f"{x:4.2f}" for x in d["mean_qnext_by_move_bin"][mv]))


if __name__ == "__main__":
    res = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SR_RESULTS", "sumo_results")
    if len(sys.argv) > 2 or glob.glob(os.path.join(res, "diag_dem*_n*_*.json")):
        main_v2(res, sys.argv[2:])
        sys.exit(0)
    import sumo_runner as R
    import signal_routing as sr
    for prefix in ("mod", "dem25", "dem10"):
        if not glob.glob(os.path.join(res, f"diag_{prefix}_*.json")):
            continue
        for fold, (est, plan) in FOLDS.items():
            d = estimate(res, prefix, R.e2l, R.ANET.offset, sr.axis_of, R.TM._window, R.C, seeds=set(est))
            json.dump(d, open(os.path.join(res, f"kernel_{prefix}_fold{fold}.json"), "w"))
            print(f"{prefix} fold {fold} (estimated on seeds {list(est)}, used by {list(plan)}): {d['n_transitions']} transitions; "
                  "mean q' by move and phase bin (cross green, <40 s after, later):")
            for mv, name in enumerate("SLR"):
                print(f"   {name}: " + "  ".join(f"{x:4.2f}" for x in d["mean_qnext_by_move_bin"][mv]))
