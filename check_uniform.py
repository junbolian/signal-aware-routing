import random, math
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
res={m:[] for m in ["SP-FF","SP-STATIC","GREEDY","TD-OPT"]}
wait={m:[] for m in res}
for i in range(60):
    rng=random.Random(1000+i)
    net=sr.Net(N,rng,C=C,link_lo=65.0,link_hi=65.0,offset_mode="random")
    p_ff=sr.static_route(net,tm,orig,dest,use_ewait=False)
    p_st=sr.static_route(net,tm,orig,dest,use_ewait=True)
    for t0 in deps:
        r={}
        r["SP-FF"]=sr.evaluate(net,tm,p_ff,t0)
        r["SP-STATIC"]=sr.evaluate(net,tm,p_st,t0)
        r["GREEDY"]=sr.evaluate(net,tm,sr.greedy_route(net,tm,orig,dest,t0),t0)
        op,ot=sr.td_route(net,tm,orig,dest,t0)
        r["TD-OPT"]=sr.evaluate(net,tm,op,t0)
        for m in res: res[m].append(r[m]["time"]); wait[m].append(r[m]["wait"])
print("UNIFORM links (65s), 8x8, random offsets, RTOR on, n=480")
opt=res["TD-OPT"]
for m in res:
    t,ci=mean_ci(res[m]); w,_=mean_ci(wait[m])
    gaps=[(a-b)/b*100 for a,b in zip(res[m],opt)]
    g,gci=mean_ci(gaps)
    print(f"{m:10s} time {t:7.1f}+-{ci:4.1f}  wait {w:6.1f}  gap {g:6.2f}+-{gci:4.2f}%")
adv=[a-b for a,b in zip(res["SP-STATIC"],res["GREEDY"])]
wins=sum(1 for x in adv if x>1e-6)
am,aci=mean_ci(adv)
print(f"GREEDY beats SP-STATIC: {wins}/480 = {100*wins/480:.1f}%  mean adv {am:+.1f}+-{aci:.1f}s")
near=sum(1 for a,b in zip(res["GREEDY"],opt) if (a-b)/b<=0.05)
print(f"GREEDY within 5% of OPT: {100*near/480:.1f}%")
