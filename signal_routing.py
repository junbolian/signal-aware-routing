"""
signal_routing.py

Movement-level signal-aware routing on an NxN grid, right-hand traffic.
Deterministic fixed-time 4-phase signals, single vehicle, uncongested (no queues).

Methods compared (all evaluated under the TRUE signal timings):
  SP-FF      free-flow shortest path, signals ignored
  SP-STATIC  free-flow + expected uniform delay r^2/(2C) per movement  [proxy for current nav]
  GREEDY     user's myopic rule: take-the-green, right-on-red, wait-for-straight
  TD-OPT     exact time-dependent shortest path with full timing knowledge
  TD-NOISY   TD planned with per-intersection offset error ~ U[-sigma, sigma], evaluated on truth

Outputs: raw csv, summary csv, three figures, stdout summary.
"""

import heapq
import math
import random
import csv
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------- geometry ------------------------------------

N_DIR = (0, 1)
S_DIR = (0, -1)
E_DIR = (1, 0)
W_DIR = (-1, 0)
DIRS = [N_DIR, S_DIR, E_DIR, W_DIR]
LEFT = {N_DIR: W_DIR, W_DIR: S_DIR, S_DIR: E_DIR, E_DIR: N_DIR}
RIGHT = {N_DIR: E_DIR, E_DIR: S_DIR, S_DIR: W_DIR, W_DIR: N_DIR}


def axis_of(h):
    return "NS" if h in (N_DIR, S_DIR) else "EW"


def l1(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ----------------------------- signals -------------------------------------

class Timing:
    """Four-phase fixed-time plan, identical plan at every intersection,
    per-intersection offsets. Right turns free unless rtor=False (then right
    shares the straight phase of its axis)."""

    def __init__(self, C=120.0, gs=40.0, gl=20.0, straight_first=True, rtor=True):
        self.C = C
        self.gs = gs
        self.gl = gl
        self.rtor = rtor
        if straight_first:
            seq = [("NS", "S", gs), ("NS", "L", gl), ("EW", "S", gs), ("EW", "L", gl)]
        else:
            seq = [("NS", "L", gl), ("NS", "S", gs), ("EW", "L", gl), ("EW", "S", gs)]
        t = 0.0
        self.win = {}
        for a, m, d in seq:
            self.win[(a, m)] = (t, d)
            t += d
        assert abs(t - C) < 1e-9, "phase durations must sum to cycle"

    def _window(self, ax, mtype):
        if mtype == "R":
            mtype = "S"  # only used when rtor is False
        return self.win[(ax, mtype)]

    def wait(self, t, offset, ax, mtype):
        if mtype == "R" and self.rtor:
            return 0.0
        s, d = self._window(ax, mtype)
        tau = (t - offset) % self.C
        if s <= tau < s + d:
            return 0.0
        return (s - tau) % self.C

    def ewait(self, ax, mtype):
        if mtype == "R" and self.rtor:
            return 0.0
        _, d = self._window(ax, mtype)
        r = self.C - d
        return r * r / (2.0 * self.C)


# ----------------------------- network -------------------------------------

class Net:
    def __init__(self, N, rng, C, link_lo=40.0, link_hi=90.0, t_cross=2.0,
                 offset_mode="random", delta=65.0):
        self.N = N
        self.t_cross = t_cross
        self.tau = {}
        for x in range(N):
            for y in range(N):
                for d in (E_DIR, N_DIR):
                    nx, ny = x + d[0], y + d[1]
                    if 0 <= nx < N and 0 <= ny < N:
                        self.tau[frozenset(((x, y), (nx, ny)))] = rng.uniform(link_lo, link_hi)
        self.offset = {}
        for x in range(N):
            for y in range(N):
                if offset_mode == "random":
                    self.offset[(x, y)] = rng.uniform(0.0, C)
                elif offset_mode == "zero":
                    self.offset[(x, y)] = 0.0
                elif offset_mode == "progressive":
                    self.offset[(x, y)] = (((N - 1 - x) + y) * delta) % C
                else:
                    raise ValueError(offset_mode)

    def link_t(self, u, v):
        return self.tau[frozenset((u, v))]

    def nbrs(self, v):
        for d in DIRS:
            w = (v[0] + d[0], v[1] + d[1])
            if 0 <= w[0] < self.N and 0 <= w[1] < self.N:
                yield w, d


def move_type(h, d):
    if d == h:
        return "S"
    if d == LEFT[h]:
        return "L"
    if d == RIGHT[h]:
        return "R"
    return None  # U-turn


# ----------------------------- evaluation ----------------------------------

def evaluate(net, tm, path, t0):
    """Realized travel under TRUE offsets for a fixed link path.
    path = [(O,x1),(x1,x2),...,(xk,D)]. Arrival defined at stopline of D."""
    u, v = path[0]
    t = t0 + net.link_t(u, v)
    wsum = 0.0
    stops = 0
    lefts = 0
    rights = 0
    away = 0
    for i in range(1, len(path)):
        pu, pv = path[i - 1]
        cu, cv = path[i]
        assert cu == pv
        h = (pv[0] - pu[0], pv[1] - pu[1])
        d = (cv[0] - cu[0], cv[1] - cu[1])
        m = move_type(h, d)
        wt = tm.wait(t, net.offset[cu], axis_of(h), m)
        if wt > 1e-9:
            stops += 1
        wsum += wt
        if m == "L":
            lefts += 1
        if m == "R":
            rights += 1
        t = t + wt + net.t_cross + net.link_t(cu, cv)
    return {"time": t - t0, "wait": wsum, "stops": stops, "lefts": lefts,
            "rights": rights, "links": len(path)}


def count_away_moves(path, dest):
    away = 0
    for (u, v) in path:
        if l1(v, dest) > l1(u, dest):
            away += 1
    return away


# ----------------------------- routers -------------------------------------

def td_route(net, tm, orig, dest, t0, plan_offsets=None):
    """Time-dependent Dijkstra on the directed-link expanded graph.
    FIFO holds (wait-until-next-green is nondecreasing), so label-setting is exact.
    plan_offsets: offsets used for PLANNING (defaults to truth)."""
    off = net.offset if plan_offsets is None else plan_offsets
    best = {}
    pred = {}
    pq = []
    for w, d in net.nbrs(orig):
        e = (orig, w)
        t = t0 + net.link_t(orig, w)
        if t < best.get(e, float("inf")):
            best[e] = t
            pred[e] = None
            heapq.heappush(pq, (t, e))
    goal = None
    while pq:
        t, e = heapq.heappop(pq)
        if t > best.get(e, float("inf")) + 1e-9:
            continue
        u, v = e
        if v == dest:
            goal = e
            break
        h = (v[0] - u[0], v[1] - u[1])
        for w, d in net.nbrs(v):
            if w == u:
                continue
            m = move_type(h, d)
            if m is None:
                continue
            wt = tm.wait(t, off[v], axis_of(h), m)
            t2 = t + wt + net.t_cross + net.link_t(v, w)
            e2 = (v, w)
            if t2 < best.get(e2, float("inf")) - 1e-9:
                best[e2] = t2
                pred[e2] = e
                heapq.heappush(pq, (t2, e2))
    assert goal is not None
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = pred[cur]
    path.reverse()
    return path, best[goal]


def static_route(net, tm, orig, dest, use_ewait):
    """Time-independent Dijkstra with additive costs (free-flow, optionally +E[wait])."""
    best = {}
    pred = {}
    pq = []
    for w, d in net.nbrs(orig):
        e = (orig, w)
        c = net.link_t(orig, w)
        if c < best.get(e, float("inf")):
            best[e] = c
            pred[e] = None
            heapq.heappush(pq, (c, e))
    goal = None
    while pq:
        c, e = heapq.heappop(pq)
        if c > best.get(e, float("inf")) + 1e-9:
            continue
        u, v = e
        if v == dest:
            goal = e
            break
        h = (v[0] - u[0], v[1] - u[1])
        for w, d in net.nbrs(v):
            if w == u:
                continue
            m = move_type(h, d)
            if m is None:
                continue
            add = (tm.ewait(axis_of(h), m) if use_ewait else 0.0) + net.t_cross + net.link_t(v, w)
            e2 = (v, w)
            if c + add < best.get(e2, float("inf")) - 1e-9:
                best[e2] = c + add
                pred[e2] = e
                heapq.heappush(pq, (c + add, e2))
    assert goal is not None
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = pred[cur]
    path.reverse()
    return path


def greedy_route(net, tm, orig, dest, t0, tie="W"):
    """User's rule. Only moves that strictly reduce L1 distance to dest.
    Priority at each stopline: green straight > green left > right(-on-red)
    > wait for straight > wait for left. Start on the axis with the larger
    remaining gap (tie: West)."""
    cands = [(w, d) for w, d in net.nbrs(orig) if l1(w, dest) < l1(orig, dest)]
    dx, dy = dest[0] - orig[0], dest[1] - orig[1]

    def start_key(item):
        w, d = item
        gap = abs(dx) if d in (E_DIR, W_DIR) else abs(dy)
        pref = 0 if ((tie == "W" and d == W_DIR) or (tie == "N" and d == N_DIR)) else 1
        return (-gap, pref)

    cands.sort(key=start_key)
    w, d = cands[0]
    path = [(orig, w)]
    t = t0 + net.link_t(orig, w)
    v, h = w, d
    guard = 0
    while v != dest:
        guard += 1
        if guard > 10 * net.N * net.N:
            raise RuntimeError("greedy stuck")
        opts = {}
        for w2, d2 in net.nbrs(v):
            if l1(w2, dest) >= l1(v, dest):
                continue
            m = move_type(h, d2)
            if m is not None:
                opts[m] = (w2, d2)
        choice = None
        if "S" in opts and tm.wait(t, net.offset[v], axis_of(h), "S") <= 1e-9:
            choice = "S"
        elif "L" in opts and tm.wait(t, net.offset[v], axis_of(h), "L") <= 1e-9:
            choice = "L"
        elif "R" in opts and tm.wait(t, net.offset[v], axis_of(h), "R") <= 1e-9:
            choice = "R"
        elif "S" in opts:
            choice = "S"
        elif "L" in opts:
            choice = "L"
        elif "R" in opts:
            choice = "R"
        else:
            raise RuntimeError("no approaching option")
        w2, d2 = opts[choice]
        wt = tm.wait(t, net.offset[v], axis_of(h), choice)
        t = t + wt + net.t_cross + net.link_t(v, w2)
        path.append((v, w2))
        v, h = w2, d2
    return path


# ----------------------------- experiment ----------------------------------

def perturbed_offsets(net, tm, sigma, rng):
    return {k: (o + rng.uniform(-sigma, sigma)) % tm.C for k, o in net.offset.items()}


def run_scenario(name, N, n_nets, C, gs, gl, straight_first, rtor, offset_mode,
                 n_dep, seed_base, writer):
    tm = Timing(C=C, gs=gs, gl=gl, straight_first=straight_first, rtor=rtor)
    orig = (N - 1, 0)          # bottom-right
    dest = (0, N - 1)          # top-left
    deps = [k * C / n_dep for k in range(n_dep)]
    rows = []
    for i in range(n_nets):
        rng = random.Random(seed_base + i)
        net = Net(N, rng, C=C, offset_mode=offset_mode)
        p_ff = static_route(net, tm, orig, dest, use_ewait=False)
        p_st = static_route(net, tm, orig, dest, use_ewait=True)
        for t0 in deps:
            res = {}
            res["SP-FF"] = evaluate(net, tm, p_ff, t0)
            res["SP-STATIC"] = evaluate(net, tm, p_st, t0)
            g_path = greedy_route(net, tm, orig, dest, t0)
            res["GREEDY"] = evaluate(net, tm, g_path, t0)
            o_path, o_t = td_route(net, tm, orig, dest, t0)
            res["TD-OPT"] = evaluate(net, tm, o_path, t0)
            assert abs(res["TD-OPT"]["time"] - (o_t - t0)) < 1e-6
            away = count_away_moves(o_path, dest)
            for meth, r in res.items():
                row = dict(scenario=name, N=N, net=i, t0=round(t0, 1), method=meth,
                           time=r["time"], wait=r["wait"], stops=r["stops"],
                           lefts=r["lefts"], rights=r["rights"], links=r["links"],
                           away=(away if meth == "TD-OPT" else ""))
                writer.writerow(row)
                rows.append(row)
    return rows


def run_voi(N, n_nets, C, gs, gl, n_dep, sigmas, seed_base, writer):
    tm = Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
    orig, dest = (N - 1, 0), (0, N - 1)
    deps = [k * C / n_dep for k in range(n_dep)]
    rows = []
    for i in range(n_nets):
        rng = random.Random(seed_base + i)
        net = Net(N, rng, C=C, offset_mode="random")
        prng = random.Random(90000 + i)
        for t0 in deps:
            for sg in sigmas:
                po = net.offset if sg == 0 else perturbed_offsets(net, tm, sg, prng)
                p, _ = td_route(net, tm, orig, dest, t0, plan_offsets=po)
                r = evaluate(net, tm, p, t0)
                row = dict(scenario="VOI", N=N, net=i, t0=round(t0, 1),
                           method="TD-NOISY", sigma=sg, time=r["time"], wait=r["wait"],
                           stops=r["stops"], lefts=r["lefts"], rights=r["rights"],
                           links=r["links"], away="")
                writer.writerow(row)
                rows.append(row)
    return rows


# ----------------------------- summarize -----------------------------------

def mean_ci(xs):
    n = len(xs)
    m = sum(xs) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, 1.96 * math.sqrt(var / n)


def paired_gap(rows, meth, base="TD-OPT"):
    key = lambda r: (r["net"], r["t0"])
    b = {key(r): r["time"] for r in rows if r["method"] == base}
    gaps = [(r["time"] - b[key(r)]) / b[key(r)] * 100.0
            for r in rows if r["method"] == meth]
    return mean_ci(gaps)


def summarize(rows, label, out):
    methods = ["SP-FF", "SP-STATIC", "GREEDY", "TD-OPT"]
    print(f"\n=== {label} ===")
    print(f"{'method':10s} {'time(s)':>10s} {'wait(s)':>9s} {'stops':>6s} "
          f"{'lefts':>6s} {'rights':>7s} {'gap%vsOPT':>12s}")
    for meth in methods:
        rs = [r for r in rows if r["method"] == meth]
        tm_, tci = mean_ci([r["time"] for r in rs])
        wm, _ = mean_ci([r["wait"] for r in rs])
        sm, _ = mean_ci([r["stops"] for r in rs])
        lm, _ = mean_ci([r["lefts"] for r in rs])
        rm, _ = mean_ci([r["rights"] for r in rs])
        gm, gci = paired_gap(rows, meth)
        print(f"{meth:10s} {tm_:8.1f}+-{tci:4.1f} {wm:9.1f} {sm:6.2f} "
              f"{lm:6.2f} {rm:7.2f} {gm:8.2f}+-{gci:4.2f}")
        out.append(dict(scenario=label, method=meth, mean_time=round(tm_, 2),
                        ci_time=round(tci, 2), mean_wait=round(wm, 2),
                        mean_stops=round(sm, 2), mean_lefts=round(lm, 2),
                        mean_rights=round(rm, 2), gap_pct=round(gm, 2),
                        gap_ci=round(gci, 2)))
    # anecdote check: greedy vs static, paired
    key = lambda r: (r["net"], r["t0"])
    st = {key(r): r["time"] for r in rows if r["method"] == "SP-STATIC"}
    gr = {key(r): r["time"] for r in rows if r["method"] == "GREEDY"}
    wins = sum(1 for k in gr if gr[k] < st[k] - 1e-6)
    adv = [st[k] - gr[k] for k in gr]
    am, aci = mean_ci(adv)
    print(f"GREEDY beats SP-STATIC in {wins}/{len(gr)} = {100*wins/len(gr):.1f}% "
          f"of instances; mean advantage {am:+.1f}+-{aci:.1f} s")
    # near-optimality of greedy
    opt = {key(r): r["time"] for r in rows if r["method"] == "TD-OPT"}
    within = sum(1 for k in gr if (gr[k] - opt[k]) / opt[k] <= 0.05)
    print(f"GREEDY within 5% of TD-OPT in {100*within/len(gr):.1f}% of instances")
    # detours in the optimum
    aw = [r["away"] for r in rows if r["method"] == "TD-OPT"]
    n_d = sum(1 for a in aw if a and a > 0)
    print(f"TD-OPT uses at least one away-from-destination move (detour) in "
          f"{100*n_d/len(aw):.1f}% of instances")


def main():
    C, gs, gl = 120.0, 40.0, 20.0
    n_dep = 8
    raw_path = "raw_results.csv"
    fields = ["scenario", "N", "net", "t0", "method", "sigma", "time", "wait",
              "stops", "lefts", "rights", "links", "away"]
    f = open(raw_path, "w", newline="")
    writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()

    summary = []
    scen = {}
    scen["base_N8"] = run_scenario("base_N8", 8, 60, C, gs, gl, True, True,
                                   "random", n_dep, 1000, writer)
    scen["no_rtor"] = run_scenario("no_rtor", 8, 60, C, gs, gl, True, False,
                                   "random", n_dep, 1000, writer)
    scen["left_first"] = run_scenario("left_first", 8, 60, C, gs, gl, False, True,
                                      "random", n_dep, 1000, writer)
    scen["sync_zero"] = run_scenario("sync_zero", 8, 60, C, gs, gl, True, True,
                                     "zero", n_dep, 1000, writer)
    scen["progressive"] = run_scenario("progressive", 8, 60, C, gs, gl, True, True,
                                       "progressive", n_dep, 1000, writer)
    for Ng in (5, 12):
        scen[f"grid_N{Ng}"] = run_scenario(f"grid_N{Ng}", Ng, 40, C, gs, gl, True,
                                           True, "random", n_dep, 2000 + Ng, writer)

    sigmas = [0, 5, 10, 20, 30, 45, 60]
    voi_rows = run_voi(8, 40, C, gs, gl, n_dep, sigmas, 1000, writer)
    f.close()

    for label, rows in scen.items():
        summarize(rows, label, summary)

    # VOI summary
    print("\n=== VOI (N=8, planning offset error sigma, evaluated on truth) ===")
    voi_curve = []
    for sg in sigmas:
        m, ci = mean_ci([r["time"] for r in voi_rows if r["sigma"] == sg])
        voi_curve.append((sg, m, ci))
        print(f"sigma={sg:3d}s  mean time {m:7.1f} +- {ci:4.1f}")
        summary.append(dict(scenario="VOI", method=f"TD-NOISY(s={sg})",
                            mean_time=round(m, 2), ci_time=round(ci, 2)))

    with open("summary.csv", "w", newline="") as sf:
        sw = csv.DictWriter(sf, fieldnames=sorted({k for d in summary for k in d}))
        sw.writeheader()
        for d in summary:
            sw.writerow(d)

    # --------------------------- figures -----------------------------------
    base = scen["base_N8"]
    methods = ["SP-FF", "SP-STATIC", "GREEDY", "TD-OPT"]
    means, cis = [], []
    for meth in methods:
        m, ci = mean_ci([r["time"] for r in base if r["method"] == meth])
        means.append(m)
        cis.append(ci)
    plt.figure(figsize=(6, 4))
    plt.bar(methods, means, yerr=cis, capsize=4, color=["#999", "#c77", "#7a7", "#57a"])
    plt.ylabel("Mean travel time (s)")
    plt.title("8x8 grid, random offsets, RTOR on (n=480, 95% CI)")
    plt.tight_layout()
    plt.savefig("figures/fig1_base_methods.png", dpi=150)
    plt.close()

    plt.figure(figsize=(6, 4))
    xs = [c[0] for c in voi_curve]
    ys = [c[1] for c in voi_curve]
    es = [c[2] for c in voi_curve]
    plt.errorbar(xs, ys, yerr=es, marker="o", label="TD planned with noisy offsets")
    for meth, style in (("SP-STATIC", "--"), ("GREEDY", ":"), ("TD-OPT", "-")):
        m, _ = mean_ci([r["time"] for r in base if r["method"] == meth])
        plt.axhline(m, linestyle=style, linewidth=1, color="k")
        plt.text(xs[-1], m, f" {meth}", va="center", fontsize=8)
    plt.xlabel("Offset knowledge error sigma (s), cycle C = 120 s")
    plt.ylabel("Mean realized travel time (s)")
    plt.title("Value of signal-timing information (N=8)")
    plt.tight_layout()
    plt.savefig("figures/fig2_voi.png", dpi=150)
    plt.close()

    plt.figure(figsize=(6, 4))
    grids = [5, 8, 12]
    for meth, mk in (("SP-STATIC", "s"), ("GREEDY", "^")):
        ys, es = [], []
        for Ng in grids:
            rows = scen["base_N8"] if Ng == 8 else scen[f"grid_N{Ng}"]
            g, ci = paired_gap(rows, meth)
            ys.append(g)
            es.append(ci)
        plt.errorbar(grids, ys, yerr=es, marker=mk, label=meth)
    plt.xlabel("Grid size N (OD = corner to corner)")
    plt.ylabel("Mean gap vs TD-OPT (%)")
    plt.title("Myopic and static gaps vs network size")
    plt.legend()
    plt.tight_layout()
    plt.savefig("figures/fig3_gap_vs_size.png", dpi=150)
    plt.close()

    print("\nDone. Files: raw_results.csv, summary.csv, fig1..fig3")


if __name__ == "__main__":
    main()
