"""Rolling-horizon lookahead-k policies.
At each stopline, choose the move minimizing realized time over the next k
signalized crossings (true phases) plus static expected cost-to-go V beyond.
k=0 (analytically) = SP-STATIC replan; k=inf = TD-OPT. GREEDY = phase, no V."""
import random, math, heapq
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr

def static_V(net, tm, dest):
    """V[e] = expected remaining time from stopline of e's head, static ewait."""
    arcs_rev = {}
    links = []
    for x in range(net.N):
        for y in range(net.N):
            u = (x, y)
            for v, d in net.nbrs(u):
                links.append((u, v))
    for e in links:
        u, v = e
        h = (v[0]-u[0], v[1]-u[1])
        for w, d in net.nbrs(v):
            if w == u: continue
            m = sr.move_type(h, d)
            if m is None: continue
            c = tm.ewait(sr.axis_of(h), m) + net.t_cross + net.link_t(v, w)
            arcs_rev.setdefault((v, w), []).append((e, c))
    V = {}
    pq = []
    for e in links:
        if e[1] == dest:
            V[e] = 0.0
            heapq.heappush(pq, (0.0, e))
    while pq:
        val, e = heapq.heappop(pq)
        if val > V.get(e, float("inf")) + 1e-9: continue
        for e_prev, c in arcs_rev.get(e, []):
            nv = val + c
            if nv < V.get(e_prev, float("inf")) - 1e-9:
                V[e_prev] = nv
                heapq.heappush(pq, (nv, e_prev))
    return V

def la_route(net, tm, orig, dest, t0, k, V):
    def moves(e):
        u, v = e
        h = (v[0]-u[0], v[1]-u[1])
        out = []
        for w, d in net.nbrs(v):
            if w == u: continue
            m = sr.move_type(h, d)
            if m is not None: out.append((m, (v, w)))
        return out
    def rec(e, t, d):
        if e[1] == dest: return t
        if d == k: return t + V[e]
        best = float("inf")
        u, v = e
        h = (v[0]-u[0], v[1]-u[1])
        for m, e2 in moves(e):
            wt = tm.wait(t, net.offset[v], sr.axis_of(h), m)
            t2 = t + wt + net.t_cross + net.link_t(*e2)
            best = min(best, rec(e2, t2, d + 1))
        return best
    # first link
    best = (float("inf"), None)
    for w, d in net.nbrs(orig):
        e = (orig, w)
        val = rec(e, t0 + net.link_t(orig, w), 0)
        if val < best[0] - 1e-9: best = (val, e)
    e = best[1]
    path = [e]
    t = t0 + net.link_t(*e)
    guard = 0
    while e[1] != dest:
        guard += 1
        if guard > 10 * net.N * net.N: raise RuntimeError("LA stuck")
        u, v = e
        h = (v[0]-u[0], v[1]-u[1])
        bm = (float("inf"), None, None)
        for m, e2 in moves(e):
            wt = tm.wait(t, net.offset[v], sr.axis_of(h), m)
            t2 = t + wt + net.t_cross + net.link_t(*e2)
            val = rec(e2, t2, 1)
            if val < bm[0] - 1e-9: bm = (val, e2, t2)
        e, t = bm[1], bm[2]
        path.append(e)
    return path

def mean_ci(xs):
    n = len(xs); m = sum(xs)/n
    if n < 2: return m, 0.0
    var = sum((x-m)**2 for x in xs)/(n-1)
    return m, 1.96*math.sqrt(var/n)

def run(link_lo, link_hi, label):
    C, gs, gl = 120.0, 40.0, 20.0
    tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
    N = 8; orig = (N-1, 0); dest = (0, N-1)
    deps = [q*C/8 for q in range(8)]
    ks = [1, 2, 3]
    res = {m: [] for m in ["SP-STATIC", "GREEDY", "LA-1", "LA-2", "LA-3", "TD-OPT"]}
    for i in range(40):
        rng = random.Random(1000+i)
        net = sr.Net(N, rng, C=C, link_lo=link_lo, link_hi=link_hi, offset_mode="random")
        V = static_V(net, tm, dest)
        p_st = sr.static_route(net, tm, orig, dest, use_ewait=True)
        for t0 in deps:
            res["SP-STATIC"].append(sr.evaluate(net, tm, p_st, t0)["time"])
            res["GREEDY"].append(sr.evaluate(net, tm, sr.greedy_route(net, tm, orig, dest, t0), t0)["time"])
            for k in ks:
                pk = la_route(net, tm, orig, dest, t0, k, V)
                res[f"LA-{k}"].append(sr.evaluate(net, tm, pk, t0)["time"])
            op, _ = sr.td_route(net, tm, orig, dest, t0)
            res["TD-OPT"].append(sr.evaluate(net, tm, op, t0)["time"])
    opt = res["TD-OPT"]
    print(f"\n=== {label} (n={len(opt)}) ===")
    out = {}
    for m in res:
        t, ci = mean_ci(res[m])
        g, gci = mean_ci([(a-b)/b*100 for a, b in zip(res[m], opt)])
        stat_gap_recovered = ""
        out[m] = (t, ci, g, gci)
        print(f"{m:10s} time {t:7.1f}+-{ci:4.1f}   gap vs OPT {g:6.2f}+-{gci:4.2f}%")
    s = out["SP-STATIC"][2]
    for m in ["GREEDY", "LA-1", "LA-2", "LA-3"]:
        rec_pct = 100*(s-out[m][2])/s if s > 0 else float('nan')
        print(f"   {m}: closes {rec_pct:5.1f}% of the static-vs-OPT gap")
    return out

if __name__ == "__main__":
    resA = run(40.0, 90.0, "heterogeneous links U[40,90]")
    resB = run(65.0, 65.0, "uniform links 65s")

    plt.figure(figsize=(6.5, 4))
    for res, lbl, mk in ((resA, "heterogeneous", "o"), (resB, "uniform", "s")):
        xs = [0, 1, 2, 3]
        ys = [res["SP-STATIC"][2], res["LA-1"][2], res["LA-2"][2], res["LA-3"][2]]
        es = [res["SP-STATIC"][3], res["LA-1"][3], res["LA-2"][3], res["LA-3"][3]]
        plt.errorbar(xs, ys, yerr=es, marker=mk, label=f"LA-k, {lbl} (k=0 is static)")
        plt.axhline(res["GREEDY"][2], linestyle=":", linewidth=1)
    plt.xlabel("Lookahead depth k (intersections with true phase knowledge)")
    plt.ylabel("Gap vs TD-OPT (%)")
    plt.title("How far ahead must the optimal rule look? (N=8, dotted = GREEDY)")
    plt.legend(fontsize=8)
    plt.xticks([0, 1, 2, 3])
    plt.tight_layout()
    plt.savefig("figures/fig5_lookahead.png", dpi=150)
    print("\nsaved fig5")
