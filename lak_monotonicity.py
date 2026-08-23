"""Pointwise monotonicity audit of rolling LA-k (sigma = 0).
Fresh recompute; checks LA-k >= TD-OPT per instance and counts adjacent-depth
pairs where the deeper policy is slower. Backs the remark in the paper."""
import random, statistics
import signal_routing as sr, lookahead as la, extend_lookahead as el

C, gs, gl = 120.0, 40.0, 20.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8; orig = (N-1, 0); dest = (0, N-1); deps = [k*C/8 for k in range(8)]
KS = [1, 2, 3, 4, 5]
tt = {}; lt = {k: {} for k in KS}; sanity = 0
for i in range(40):
    rng = random.Random(1000+i)
    net = sr.Net(N, rng, C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
    V = la.static_V(net, tm, dest)
    for t0 in deps:
        p, _ = sr.td_route(net, tm, orig, dest, t0)
        tt[(i, t0)] = sr.evaluate(net, tm, p, t0)["time"]
        for k in KS:
            T = el.la_route_noisy(net, tm, orig, dest, t0, k, V, net.offset) - t0
            lt[k][(i, t0)] = T
            if T < tt[(i, t0)] - 1e-6: sanity += 1
viol = [(k, lt[k+1][key]-lt[k][key]) for key in tt for k in KS[:-1]
        if lt[k+1][key]-lt[k][key] > 1e-6]
print(f"sanity LA-k < TD-OPT violations: {sanity} (must be 0)")
print(f"adjacent-depth pairs: {len(tt)*4}  deeper-slower: {len(viol)} "
      f"({len(viol)/(len(tt)*4)*100:.1f}%)  mean +{statistics.mean(v[1] for v in viol):.1f}s  "
      f"max +{max(v[1] for v in viol):.1f}s")
