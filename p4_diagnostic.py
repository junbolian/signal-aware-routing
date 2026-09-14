"""Diagnostic for the coupling bound (Prop: coupled sensitivity to offset error).
For each VOI instance: plan under o_hat, execute path p_hat; trace both timelines
along p_hat; classify crossings matched/mismatched; check
|g_hat - g| <= sigma + sum r_m * M and the matched-only case |g_hat - g| <= sigma.

Corrected definition of a match (Proposition 4): a crossing is matched when the
planned and the realized timeline depart it in the SAME CYCLE, that is when each
of them either passes during that cycle's green window or waits for that window's
onset; it is mismatched otherwise. The cycle is identified by the index
k = floor((passage - offset - window_start) / C), which is the same integer for
both timelines exactly when they leave on the same green. An uncontrolled right
turn has no window and never waits, so it is matched by construction.

The previous indicator compared the two passage times against sigma directly,
which is neither necessary nor sufficient for leaving on the same green.
"""
import random, math, json
import signal_routing as sr

C, gs, gl = 120.0, 40.0, 20.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8; orig = (N-1, 0); dest = (0, N-1); deps = [k*C/8 for k in range(8)]
SIGMAS = [5, 10, 20, 30, 45, 60]


def trace(net, path, t0):
    """per-crossing (movement, arrival, wait, passage, red_dur, window_start,
    node_offset, free) under net offsets; free marks the uncontrolled right turn."""
    u, v = path[0]
    t = t0 + net.link_t(u, v)
    rec = []
    for i in range(1, len(path)):
        pu, pv = path[i-1]; cu, cv = path[i]
        h = (pv[0]-pu[0], pv[1]-pu[1]); d = (cv[0]-cu[0], cv[1]-cu[1])
        m = sr.move_type(h, d)
        ax = sr.axis_of(h)
        wt = tm.wait(t, net.offset[cu], ax, m)
        free = (m == "R" and tm.rtor)
        s_win = 0.0 if free else tm._window(ax, m)[0]
        # red duration for this movement
        if free: rdur = 0.0
        elif m == "L": rdur = C - gl
        else: rdur = C - gs
        rec.append((m, t, wt, t + wt, rdur, s_win, net.offset[cu], free))
        t = t + wt + net.t_cross + net.link_t(cu, cv)
    return rec, t - t0


def cycle_index(passage, offset, s_win):
    """Index of the green window the timeline departs on."""
    return math.floor((passage - offset - s_win) / C + 1e-9)


viol = 0; total = 0; zero_mm = 0; zero_mm_maxdev = 0.0
tight = []; mmrate = {}; per_sigma = {}
for sig in SIGMAS:
    mm_cross = 0; n_cross = 0; z_n = 0; z_max = 0.0; t_sig = []
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
            for (mh, ah, wh, ph, rh, sh, oh, fh), (mt, at_, wt_, pt, rt, st, ot, ft) in zip(rec_h, rec_t):
                if fh:
                    matched = True          # uncontrolled right turn: no window, never waits
                else:
                    matched = cycle_index(ph, oh, sh) == cycle_index(pt, ot, st)
                if not matched:
                    M += 1; bound += rh
                    mm_cross += 1
                n_cross += 1
            total += 1
            if dev > bound + 1e-6: viol += 1
            if M == 0:
                zero_mm += 1; z_n += 1
                zero_mm_maxdev = max(zero_mm_maxdev, dev)
                z_max = max(z_max, dev)
            else:
                tight.append(dev / bound); t_sig.append(dev / bound)
    mmrate[sig] = mm_cross / n_cross
    ts_ = sorted(t_sig)
    per_sigma[sig] = {"zero_mm": z_n, "zero_mm_maxdev": z_max, "n_mm": len(t_sig),
                      "tight_median": (ts_[len(ts_)//2] if ts_ else None),
                      "tight_max": (ts_[-1] if ts_ else None),
                      "mmrate": mmrate[sig]}
print(f"instances: {total}  bound violations: {viol}")
print(f"zero-mismatch instances: {zero_mm}  max |g_hat-g| there: {zero_mm_maxdev:.3f} s (pooled over sigma)")
ts = sorted(tight)
print(f"with mismatches: n={len(tight)}  tightness dev/bound median {ts[len(ts)//2]:.3f}  p90 {ts[int(.9*len(ts))]:.3f}  max {ts[-1]:.3f}")
print("per sigma: the matched-only bound is |g_hat-g| <= sigma, checked on the")
print("zero-mismatch evaluations of that sigma alone")
for s in SIGMAS:
    d = per_sigma[s]
    ok = "ok" if d["zero_mm_maxdev"] <= s + 1e-6 else "VIOLATED"
    md = f"{d['tight_median']:.3f}" if d["tight_median"] is not None else "-"
    mx = f"{d['tight_max']:.3f}" if d["tight_max"] is not None else "-"
    print(f"  sigma={s:2d}: zero-mismatch n={d['zero_mm']:4d} max dev {d['zero_mm_maxdev']:7.3f} s ({ok});"
          f" with mismatches n={d['n_mm']:4d} dev/bound median {md} max {mx};"
          f" mismatch rate per crossing {d['mmrate']:.4f} (2*sigma/C = {2*s/C:.4f})")
json.dump({"violations": viol, "total": total, "zero_mm": zero_mm,
           "zero_mm_maxdev": zero_mm_maxdev, "mmrate": mmrate,
           "per_sigma": per_sigma,
           "definition": "matched iff both timelines depart the crossing on the same "
                         "green window (equal cycle index); uncontrolled right turns "
                         "are matched by construction"},
          open("p4_diag.json", "w"), indent=1)
