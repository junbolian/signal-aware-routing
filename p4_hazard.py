"""First-mismatch hazard on the selected paths of the coupling diagnostic.
Before the first mismatch the gap is at most sigma, so for a fixed path with
uniform planned arrival phases the hazard per controlled crossing is at most
2*sigma/C. A hazard above that indicates selection effects."""
import sys, random, math
sys.path.insert(0, ".")
exec(open("p4_diagnostic.py").read().split("viol = 0")[0])
out={}
for sig in SIGMAS:
    risk = 0; first = 0; risk_all = 0
    for i in range(40):
        net = sr.Net(N, random.Random(1000+i), C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
        prng = random.Random(50000+1000*sig+i)
        net_hat = sr.Net(N, random.Random(1000+i), C=C, link_lo=40.0, link_hi=90.0, offset_mode="random")
        for node in net_hat.offset:
            net_hat.offset[node] = (net.offset[node] + prng.uniform(-sig, sig)) % C
        for t0 in deps:
            p_hat, _ = sr.td_route(net_hat, tm, orig, dest, t0)
            rec_h, _ = trace(net_hat, p_hat, t0); rec_t, _ = trace(net, p_hat, t0)
            for (mh, ah, wh, ph, rh, sh, oh, fh), (mt, at_, wt_, pt, rt, st, ot, ft) in zip(rec_h, rec_t):
                risk_all += 1
                if fh: continue
                risk += 1
                oh_u = ot + ((oh - ot + C / 2) % C - C / 2)
                if cycle_index(ph, oh_u, sh) != cycle_index(pt, ot, st):
                    first += 1; break
    out[sig]={"hazard":first/risk,"n_risk":risk,"firsts":first}
    print(f"sigma={sig:2d} first-mismatch hazard per controlled crossing at risk {first/risk:.4f} (n_risk={risk}, firsts={first}); 2sigma/C={2*sig/C:.4f}")
json.dump(out, open("p4_hazard.json","w"), indent=1)
