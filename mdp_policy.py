"""P5: unknown iid offsets MDP. Value iteration over links with expectation
over the observed local phase; threshold policy; simulation on true offsets."""
import random, math
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr
from lookahead import static_V, la_route

C,gs,gl=120.0,40.0,20.0
tm=sr.Timing(C=C,gs=gs,gl=gl,straight_first=True,rtor=True)
N=8; orig=(N-1,0); dest=(0,N-1)
DXI=0.25; XI=np.arange(0.0,C,DXI)
W={}
for ax in ("NS","EW"):
    for mt in ("S","L"):
        W[(ax,mt)]=np.array([tm.wait(x,0.0,ax,mt) for x in XI])

def build(net):
    links=[]; idx={}
    for x in range(net.N):
        for y in range(net.N):
            u=(x,y)
            for v,d in net.nbrs(u):
                idx[(u,v)]=len(links); links.append((u,v))
    moves=[[] for _ in links]
    for i,(u,v) in enumerate(links):
        if v==dest: continue
        h=(v[0]-u[0],v[1]-u[1])
        for w,d in net.nbrs(v):
            if w==u: continue
            mt=sr.move_type(h,d)
            if mt is None: continue
            moves[i].append((mt,idx[(v,w)],net.link_t(v,w)))
    return links,idx,moves

def solve_mdp(net):
    links,idx,moves=build(net)
    V=np.zeros(len(links))
    order=sorted(range(len(links)),key=lambda i:sr.l1(links[i][1],dest))
    sweeps=0
    for sweeps in range(1,301):
        delta=0.0
        for i in order:
            u,v=links[i]
            if v==dest: continue
            ax=sr.axis_of((v[0]-u[0],v[1]-u[1]))
            best=np.full(len(XI),np.inf)
            for (mt,j,taun) in moves[i]:
                b=net.t_cross+taun+V[j]
                cost=(np.full(len(XI),b) if (mt=="R" and tm.rtor)
                      else W[(ax,mt)]+b)
                np.minimum(best,cost,out=best)
            nv=best.mean()
            delta=max(delta,abs(nv-V[i])); V[i]=nv
        if delta<1e-4: break
    return V,idx,moves,links,sweeps

def mdp_sim(net,V,idx,moves,t0):
    best=(float("inf"),None)
    for w,d in net.nbrs(orig):
        c=net.link_t(orig,w)+V[idx[(orig,w)]]
        if c<best[0]-1e-9: best=(c,(orig,w))
    e=best[1]; t=t0+net.link_t(*e)
    seen=set(); revisits=0
    guard=0
    while e[1]!=dest:
        guard+=1
        if guard>10*net.N*net.N: raise RuntimeError("mdp stuck")
        u,v=e
        if v in seen: revisits+=1
        seen.add(v)
        ax=sr.axis_of((v[0]-u[0],v[1]-u[1]))
        i=idx[e]; bm=(float("inf"),None,None)
        for (mt,j,taun) in moves[i]:
            wt=tm.wait(t,net.offset[v],ax,mt)
            sc=wt+net.t_cross+taun+V[j]
            if sc<bm[0]-1e-9: bm=(sc,j,wt+net.t_cross+taun)
        t+=bm[2]; e_next=None
        # recover link tuple
        e=( [lk for lk,ii in idx.items() if ii==bm[1]][0] )
    return t-t0,revisits

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    if n<2: return m,0.0
    var=sum((x-m)**2 for x in xs)/(n-1)
    return m,1.96*math.sqrt(var/n)

deps=[q*C/8 for q in range(8)]
res={m:[] for m in ["SP-STATIC","GREEDY","LA-1","MDP","LA-2","LA-3","TD-OPT"]}
pred=[]; rev_total=0; max_sweeps=0
for i in range(40):
    rng=random.Random(1000+i)
    net=sr.Net(N,rng,C=C,link_lo=40.0,link_hi=90.0,offset_mode="random")
    V,idx,moves,links,sw=solve_mdp(net)
    max_sweeps=max(max_sweeps,sw)
    Vs=static_V(net,tm,dest)
    p_st=sr.static_route(net,tm,orig,dest,use_ewait=True)
    pred.append(min(net.link_t(orig,w)+V[idx[(orig,w)]] for w,_ in net.nbrs(orig)))
    for t0 in deps:
        res["SP-STATIC"].append(sr.evaluate(net,tm,p_st,t0)["time"])
        res["GREEDY"].append(sr.evaluate(net,tm,sr.greedy_route(net,tm,orig,dest,t0),t0)["time"])
        for k in (1,2,3):
            res[f"LA-{k}"].append(sr.evaluate(net,tm,la_route(net,tm,orig,dest,t0,k,Vs),t0)["time"])
        T,rv=mdp_sim(net,V,idx,moves,t0); res["MDP"].append(T); rev_total+=rv
        op,_=sr.td_route(net,tm,orig,dest,t0)
        res["TD-OPT"].append(sr.evaluate(net,tm,op,t0)["time"])
opt=res["TD-OPT"]
print(f"VI max sweeps {max_sweeps}, intersection revisits across all trips: {rev_total}")
pm,_=mean_ci(pred); mm,mci=mean_ci(res["MDP"])
print(f"MDP predicted mean {pm:.1f}s vs simulated {mm:.1f}+-{mci:.1f}s")
print(f"{'method':10s} {'time':>8s} {'gap%vsOPT':>12s}")
gaps={}
for m in res:
    t,ci=mean_ci(res[m])
    g,gci=mean_ci([(a-b)/b*100 for a,b in zip(res[m],opt)])
    gaps[m]=(g,gci)
    print(f"{m:10s} {t:8.1f} {g:9.2f}+-{gci:4.2f}")
pairs=[(a-b) for a,b in zip(res["LA-1"],res["MDP"])]
am,aci=mean_ci(pairs)
print(f"MDP vs LA-1 paired advantage {am:+.1f}+-{aci:.1f}s, "
      f"MDP better in {100*sum(1 for x in pairs if x>0)/len(pairs):.1f}% of instances")

# threshold example: interior westbound link on net seed 1000
rng=random.Random(1000); net=sr.Net(N,rng,C=C,link_lo=40.0,link_hi=90.0,offset_mode="random")
V,idx,moves,links,_=solve_mdp(net)
e=((4,3),(3,3)); i=idx[e]
print("\nThreshold example, westbound link (4,3)->(3,3):")
bs={}
for (mt,j,taun) in moves[i]:
    b=net.t_cross+taun+V[j]; bs[mt]=b
    print(f"  move {mt}: continuation b = {b:7.1f}s")
if "R" in bs and "S" in bs:
    print(f"  take R iff w_S > {bs['R']-bs['S']:.1f}s"
          + (f" and w_L > {bs['R']-bs['L']:.1f}s" if "L" in bs else ""))
if "S" in bs and "L" in bs:
    print(f"  prefer S over L iff w_S - w_L <= {bs['L']-bs['S']:.1f}s")

plt.figure(figsize=(6.5,4))
order=["SP-STATIC","GREEDY","LA-1","MDP","LA-2","LA-3","TD-OPT"]
ys=[gaps[m][0] for m in order]; es=[gaps[m][1] for m in order]
plt.bar(order,ys,yerr=es,capsize=4,
        color=["#c77","#b55","#7a7","#57a","#7a7","#7a7","#333"])
plt.ylabel("Gap vs TD-OPT (%)")
plt.title("Optimal policy under unknown offsets (MDP) vs lookahead ladder")
plt.xticks(rotation=30,fontsize=8); plt.tight_layout()
plt.savefig("/home/claude/fig7_mdp.png",dpi=150)
print("saved fig7")
