"""Diagnostic for the coupling bound (Prop: coupled sensitivity to offset error).
For each VOI instance: plan under o_hat, execute path p_hat; trace both timelines
along p_hat; classify crossings matched/mismatched; check
|g_hat - g| <= sigma + sum r_m * M and the matched-only case |g_hat - g| <= sigma."""
import random, math, json
import signal_routing as sr

C, gs, gl = 120.0, 40.0, 20.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8; orig = (N-1, 0); dest = (0, N-1); deps = [k*C/8 for k in range(8)]
SIGMAS = [5, 10, 20, 30, 45, 60]

def trace(net, path, t0):
    """per-crossing (movement, arrival, wait, passage, red_dur) under net offsets."""
    u, v = path[0]
    t = t0 + net.link_t(u, v)
    rec = []
    for i in range(1, len(path)):
        pu, pv = path[i-1]; cu, cv = path[i]
        h = (pv[0]-pu[0], pv[1]-pu[1]); d = (cv[0]-cu[0], cv[1]-cu[1])
        m = sr.move_type(h, d)
        ax = sr.axis_of(h)
        wt = tm.wait(t, net.offset[cu], ax, m)
        g = tm._window(ax, m)[1] if hasattr(tm, "_window") else None
        # red duration for this movement
        if m == "R" and tm.rtor: rdur = 0.0
        elif m == "L": rdur = C - gl
        else: rdur = C - gs
        rec.append((m, t, wt, t + wt, rdur))
        t = t + wt + net.t_cross + net.link_t(cu, cv)
    return rec, t - t0

viol = 0; total = 0; zero_mm = 0; zero_mm_maxdev = 0.0
tight = []; mmrate = {}
for sig in SIGMAS:
    mm_cross = 0; n_cross = 0
    for i in range(40):
        rng = random.Random(1000+i)
        net = sr.Net(N, rng, C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
        prng = random.Random(50000+1000*sig+i)
        net_hat = sr.Net(N, random.Random(1000+i), C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
        for node in net_hat.offset:
            net_hat.offset[node] = (net.offset[node] + prng.uniform(-sig, sig)) % C
        for t0 in deps:
            p_hat, _ = sr.td_route(net_hat, tm, orig, dest, t0)
            rec_h, g_hat = trace(net_hat, p_hat, t0)
            rec_t, g_true = trace(net, p_hat, t0)
            dev = abs(g_hat - g_true)
            bound = sig; M = 0
            for (mh, ah, wh, ph, rh), (mt, at_, wt_, pt, rt) in zip(rec_h, rec_t):
                green_h = wh <= 1e-9; green_t = wt_ <= 1e-9
                matched = (green_h and green_t) or ((not green_h) and (not green_t) and abs(ph - pt) <= sig + 1e-6)
                if not matched:
                    M += 1; bound += rh
                    mm_cross += 1
                n_cross += 1
            total += 1
            if dev > bound + 1e-6: viol += 1
            if M == 0:
                zero_mm += 1
                zero_mm_maxdev = max(zero_mm_maxdev, dev)
            else:
                tight.append(dev / bound)
    mmrate[sig] = mm_cross / n_cross
print(f"instances: {total}  bound violations: {viol}")
print(f"zero-mismatch instances: {zero_mm}  max |g_hat-g| there: {zero_mm_maxdev:.3f} s (must be <= sigma)")
ts = sorted(tight)
print(f"with mismatches: n={len(tight)}  tightness dev/bound median {ts[len(ts)//2]:.3f}  p90 {ts[int(.9*len(ts))]:.3f}  max {ts[-1]:.3f}")
for s in SIGMAS: print(f"  sigma={s:2d}: mismatch rate per crossing {mmrate[s]:.4f}  (heuristic 2*sigma/C = {2*s/C:.4f})")
json.dump({"violations": viol, "total": total, "zero_mm": zero_mm,
           "zero_mm_maxdev": zero_mm_maxdev, "mmrate": mmrate}, open("p4_diag.json","w"))
