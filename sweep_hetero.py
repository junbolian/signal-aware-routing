import random, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    if n<2: return m,0.0
    var=sum((x-m)**2 for x in xs)/(n-1)
    return m, 1.96*math.sqrt(var/n)

C,gs,gl=120.0,40.0,20.0
tm=sr.Timing(C=C,gs=gs,gl=gl,straight_first=True,rtor=True)
N=8; orig=(N-1,0); dest=(0,N-1)
deps=[k*C/8 for k in range(8)]
spreads=[0,5,10,15,20,25]
adv_curve=[]; ggap_curve=[]; sgap_curve=[]
print(f"{'half-width h':>12s} {'CV%':>5s} {'GREEDY-adv(s)':>14s} {'win%':>6s} {'gGap%':>7s} {'sGap%':>7s}")
for h in spreads:
    res={m:[] for m in ["SP-STATIC","GREEDY","TD-OPT"]}
    for i in range(40):
        rng=random.Random(1000+i)
        net=sr.Net(N,rng,C=C,link_lo=65.0-h,link_hi=65.0+h,offset_mode="random")
        p_st=sr.static_route(net,tm,orig,dest,use_ewait=True)
        for t0 in deps:
            res["SP-STATIC"].append(sr.evaluate(net,tm,p_st,t0)["time"])
            res["GREEDY"].append(sr.evaluate(net,tm,sr.greedy_route(net,tm,orig,dest,t0),t0)["time"])
            op,_=sr.td_route(net,tm,orig,dest,t0)
            res["TD-OPT"].append(sr.evaluate(net,tm,op,t0)["time"])
    adv=[a-b for a,b in zip(res["SP-STATIC"],res["GREEDY"])]
    am,aci=mean_ci(adv); wins=100*sum(1 for x in adv if x>0)/len(adv)
    gg,ggci=mean_ci([(a-b)/b*100 for a,b in zip(res["GREEDY"],res["TD-OPT"])])
    sg,sgci=mean_ci([(a-b)/b*100 for a,b in zip(res["SP-STATIC"],res["TD-OPT"])])
    cv=100*(h/math.sqrt(3))/65.0
    adv_curve.append((h,am,aci)); ggap_curve.append((h,gg,ggci)); sgap_curve.append((h,sg,sgci))
    print(f"{h:12d} {cv:5.1f} {am:+10.1f}+-{aci:4.1f} {wins:6.1f} {gg:7.2f} {sg:7.2f}")

plt.figure(figsize=(6,4))
xs=[c[0] for c in adv_curve]; ys=[c[1] for c in adv_curve]; es=[c[2] for c in adv_curve]
plt.errorbar(xs,ys,yerr=es,marker="o",color="#7a7")
plt.axhline(0,color="k",linewidth=1)
plt.xlabel("Link-time half-width h (s), tau ~ U[65-h, 65+h]")
plt.ylabel("Mean advantage of GREEDY over SP-STATIC (s)")
plt.title("Green-chasing pays only in homogeneous grids (N=8, n=320/pt)")
plt.tight_layout(); plt.savefig("figures/fig4_heterogeneity.png",dpi=150)
print("saved fig4")
