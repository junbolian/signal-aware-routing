import json, math, glob, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr
from lookahead import la_route
from sumo_runner import ANET, TM, ORIG, DEST, VSTAT, T0S

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    if n<2: return m,0.0
    var=sum((x-m)**2 for x in xs)/(n-1)
    return m,1.96*math.sqrt(var/n)

def model_pred(m,t0):
    if m=="STATIC":
        p=sr.static_route(ANET,TM,ORIG,DEST,use_ewait=True)
        return sr.evaluate(ANET,TM,p,t0)["time"]
    if m=="GREEDY":
        return sr.evaluate(ANET,TM,sr.greedy_route(ANET,TM,ORIG,DEST,t0),t0)["time"]
    if m=="LA1":
        return sr.evaluate(ANET,TM,la_route(ANET,TM,ORIG,DEST,t0,1,VSTAT),t0)["time"]
    p,_=sr.td_route(ANET,TM,ORIG,DEST,t0)
    return sr.evaluate(ANET,TM,p,t0)["time"]

emp=json.load(open("sumo_results/empty.json"))
emp["TDOPT_replan"]=json.load(open("sumo_results/empty_one_TDOPT_replan.json"))
methods=["STATIC","GREEDY","LA1","TDOPT_open","TDOPT_replan"]
dif=[s-model_pred("TDOPT_replan",t0) for s,t0 in zip(emp["TDOPT_replan"],T0S)]
delta=sum(dif)/len(dif)/13.0
print(f"=== EMPTY NET: model vs SUMO (per-crossing loss delta={delta:.2f}s) ===")
for m in methods:
    devs=[100*(s-(model_pred(m,t0)+13*delta))/(model_pred(m,t0)+13*delta)
          for s,t0 in zip(emp[m],T0S)]
    dm,dci=mean_ci(devs)
    print(f"  {m:12s} sumo {[int(x) for x in emp[m]]}  dev after calib {dm:+5.1f}%+-{dci:.1f}")

mod={}
for f in glob.glob("sumo_results/mod_*.json"):
    parts=os.path.basename(f)[:-5].split("_")
    seed=parts[-1]; meth="_".join(parts[1:-1])
    mod.setdefault(meth,{})[int(seed)]=json.load(open(f))
print("\n=== MODERATE TRAFFIC (prob 1/15, warmup 240s) ===")
flat={}
for m in methods+["STATICTRAFFIC"]:
    xs=[]
    for sd in sorted(mod.get(m,{})):
        xs+= [x for x in mod[m][sd] if x]
    flat[m]=xs
    if not xs: continue
    t,ci=mean_ci(xs)
    print(f"  {m:12s} n={len(xs):2d}  time {t:7.1f}+-{ci:5.1f}")
def paired(a,b):
    seeds=sorted(set(mod[a])&set(mod[b]))
    d=[]
    for sd in seeds:
        d+= [x-y for x,y in zip(mod[a][sd],mod[b][sd]) if x and y]
    m_,ci=mean_ci(d); wins=sum(1 for x in d if x<0)
    return m_,ci,wins,len(d)
for a,b in (("LA1","STATIC"),("LA1","TDOPT_replan"),("TDOPT_replan","TDOPT_open"),
            ("STATIC","TDOPT_open"),("GREEDY","STATIC")):
    m_,ci,w,n=paired(a,b)
    print(f"  paired {a} - {b}: {m_:+7.1f}+-{ci:5.1f}s  ({a} faster in {w}/{n})")

print("\n=== DEMAND SWEEP (prob 1/den, n=16 per cell) ===")
for den in (25,15,10):
    src={}
    if den==15:
        for m in ("STATIC","LA1","TDOPT_replan"):
            src[m]={sd:mod[m][sd] for sd in (11,12,13,14)}
    else:
        for f in glob.glob(f"sumo_results/dem{den}_*.json"):
            parts=os.path.basename(f)[:-5].split("_")
            seed=parts[-1]; meth="_".join(parts[1:-1])
            src.setdefault(meth,{})[int(seed)]=json.load(open(f))
    line=f"  1/{den:2d}: "
    for m in ("STATIC","LA1","TDOPT_replan"):
        xs=[x for sd in sorted(src.get(m,{})) for x in src[m][sd] if x]
        t,ci=mean_ci(xs); miss=sum(1 for sd in src.get(m,{}) for x in src[m][sd] if not x)
        line+=f"{m} {t:6.1f}+-{ci:4.1f} (miss {miss})  "
    def pr2(a,b):
        seeds=sorted(set(src[a])&set(src[b])); d=[]
        for sd in seeds:
            d+=[x-y for x,y in zip(src[a][sd],src[b][sd]) if x and y]
        m_,ci=mean_ci(d); return m_,ci,sum(1 for x in d if x<0),len(d)
    m1,c1,w1,n1=pr2("LA1","STATIC"); m2,c2,w2,n2=pr2("LA1","TDOPT_replan")
    print(line)
    print(f"        LA1-STATIC {m1:+6.1f}+-{c1:4.1f} ({w1}/{n1})   LA1-replan {m2:+6.1f}+-{c2:4.1f} ({w2}/{n2})")

# --- traffic-aware baseline: measured link times, no signal term ------------
print("\n=== STATIC-TRAFFIC (measured link times) vs the phase-aware methods ===")
def cell(den):
    if den==15: return mod
    src={}
    for f in glob.glob(f"sumo_results/dem{den}_*.json"):
        parts=os.path.basename(f)[:-5].split("_")
        src.setdefault("_".join(parts[1:-1]),{})[int(parts[-1])]=json.load(open(f))
    return src
def pairdiff(src,a,b):
    seeds=sorted(set(src.get(a,{}))&set(src.get(b,{}))); d=[]
    for sd in seeds:
        d+=[x-y for x,y in zip(src[a][sd],src[b][sd]) if x and y]
    if not d: return None
    m_,ci=mean_ci(d); return m_,ci,sum(1 for x in d if x<0),len(d)
for den,lab in ((25,"light"),(15,"moderate"),(10,"heavy")):
    src=cell(den); st=src.get("STATICTRAFFIC",{})
    n=sum(1 for sd in st for x in st[sd] if x)
    print(f"  {lab} (1/{den}): n={n} over seeds {sorted(st)}")
    for m in ("STATIC","LA1","TDOPT_replan"):
        miss=set(st)-set(src.get(m,{}))
        if miss: print(f"    WARNING unpaired seeds vs {m}: {sorted(miss)}")
    for a,b in (("STATICTRAFFIC","STATIC"),("LA1","STATICTRAFFIC"),
                ("TDOPT_replan","STATICTRAFFIC")):
        r=pairdiff(src,a,b)
        if r is None: print(f"    {a} - {b}: missing"); continue
        m_,ci,w,nn=r
        print(f"    {a:13s} - {b:13s} {m_:+7.1f}+-{ci:5.1f}s  ({a} faster in {w}/{nn})")
print("done")
