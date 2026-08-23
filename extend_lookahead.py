"""LA-k for k=1..5, with noisy countdown information (offset error sigma
inside the lookahead only; execution uses true phases). Heterogeneous links."""
import random, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr
from lookahead import static_V

def la_route_noisy(net, tm, orig, dest, t0, k, V, plan_off):
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
            wt = tm.wait(t, plan_off[v], sr.axis_of(h), m)
            t2 = t + wt + net.t_cross + net.link_t(*e2)
            best = min(best, rec(e2, t2, d + 1))
        return best
    best = (float("inf"), None)
    for w, d in net.nbrs(orig):
        e = (orig, w)
        val = rec(e, t0 + net.link_t(orig, w), 0)
        if val < best[0] - 1e-9: best = (val, e)
    e = best[1]
    path = [e]
    t = t0 + net.link_t(*e)   # true clock
    guard = 0
    while e[1] != dest:
        guard += 1
        if guard > 10 * net.N * net.N: raise RuntimeError("stuck")
        u, v = e
        h = (v[0]-u[0], v[1]-u[1])
        bm = (float("inf"), None)
        for m, e2 in moves(e):
            wt_p = tm.wait(t, plan_off[v], sr.axis_of(h), m)
            val = rec(e2, t + wt_p + net.t_cross + net.link_t(*e2), 1)
            if val < bm[0] - 1e-9: bm = (val, e2, m, h)
        e2 = bm[1]
        wt_true = tm.wait(t, net.offset[e[1]], sr.axis_of(bm[3]), bm[2])
        t = t + wt_true + net.t_cross + net.link_t(*e2)
        e = e2
        path.append(e)
    return t

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    if n<2: return m,0.0
    var=sum((x-m)**2 for x in xs)/(n-1)
    return m,1.96*math.sqrt(var/n)

if __name__ == "__main__":
    C,gs,gl=120.0,40.0,20.0
    tm=sr.Timing(C=C,gs=gs,gl=gl,straight_first=True,rtor=True)
    N=8; orig=(N-1,0); dest=(0,N-1)
    deps=[q*C/8 for q in range(8)]
    ks=[1,2,3,4,5]; sigmas=[0,5,10]
    res={(k,s):[] for k in ks for s in sigmas}
    opt=[]; stat=[]
    for i in range(40):
        rng=random.Random(1000+i)
        net=sr.Net(N,rng,C=C,link_lo=40.0,link_hi=90.0,offset_mode="random")
        V=static_V(net,tm,dest)
        p_st=sr.static_route(net,tm,orig,dest,use_ewait=True)
        prng=random.Random(70000+i)
        pofs={s:({kk:(o+prng.uniform(-s,s))%C for kk,o in net.offset.items()} if s>0 else net.offset) for s in sigmas}
        for t0 in deps:
            stat.append(sr.evaluate(net,tm,p_st,t0)["time"])
            op,_=sr.td_route(net,tm,orig,dest,t0)
            opt.append(sr.evaluate(net,tm,op,t0)["time"])
            for k in ks:
                for s in sigmas:
                    T=la_route_noisy(net,tm,orig,dest,t0,k,V,pofs[s])-t0
                    res[(k,s)].append(T)
    sm,_=mean_ci(stat); om,_=mean_ci(opt)
    sgap,_=mean_ci([(a-b)/b*100 for a,b in zip(stat,opt)])
    print(f"SP-STATIC gap {sgap:.2f}%   TD-OPT {om:.1f}s")
    print(f"{'k':>3s} " + " ".join(f"sigma={s:>2d}gap%" for s in sigmas))
    curves={s:[] for s in sigmas}
    for k in ks:
        row=f"{k:3d} "
        for s in sigmas:
            g,gci=mean_ci([(a-b)/b*100 for a,b in zip(res[(k,s)],opt)])
            curves[s].append((k,g,gci))
            row+=f"  {g:5.2f}+-{gci:4.2f}"
        print(row)
    plt.figure(figsize=(6.5,4))
    plt.axhline(sgap,color="k",linestyle="--",linewidth=1)
    plt.text(4.4,sgap," SP-STATIC",fontsize=8,va="bottom")
    for s,mk in ((0,"o"),(5,"s"),(10,"^")):
        xs=[c[0] for c in curves[s]]; ys=[c[1] for c in curves[s]]; es=[c[2] for c in curves[s]]
        plt.errorbar(xs,ys,yerr=es,marker=mk,label=f"countdown error sigma={s}s")
    plt.xlabel("Lookahead depth k"); plt.ylabel("Gap vs TD-OPT (%)")
    plt.title("Depth and accuracy of phase information (hetero, N=8, n=320)")
    plt.legend(fontsize=8); plt.xticks(ks); plt.tight_layout()
    plt.savefig("figures/fig6_lookahead_noisy.png",dpi=150)
    print("saved fig6")
