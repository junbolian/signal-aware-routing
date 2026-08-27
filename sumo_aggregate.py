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

# --- measured-link-time baselines --------------------------------------------
# Column 1 pairs against the shipped results. Column 2 pairs against the same
# methods re-run on the build in SAMEBUILD_DIR, which removes SUMO-version drift;
# set that env var to a directory of <prefix>_<METHOD>_<seed>.json to enable it.
SAMEBUILD=os.environ.get("SAMEBUILD_DIR","")
def cell(den,d="sumo_results"):
    if den==15 and d=="sumo_results": return mod
    src={}
    for f in glob.glob(os.path.join(d,("mod" if den==15 else f"dem{den}")+"_*.json")):
        p=os.path.basename(f)[:-5].split("_")
        src.setdefault("_".join(p[1:-1]),{})[int(p[-1])]=json.load(open(f))
    return src
def pairdiff(A,B,a,b):
    seeds=sorted(set(A.get(a,{}))&set(B.get(b,{}))); d=[]
    for sd in seeds:
        d+=[x-y for x,y in zip(A[a][sd],B[b][sd]) if x and y]
    if not d: return None
    m_,ci=mean_ci(d); return m_,ci,sum(1 for x in d if x<0),len(d)
def fmt(r,a):
    if r is None: return "        n/a        "
    m_,ci,w,n=r; return f"{m_:+7.1f}+-{ci:5.1f} ({w:2d}/{n:2d})"
NEW=[("STATICTRAFFIC","STATIC-TRAFFIC"),("STATICTRAFFICRT","STATIC-TRAFFIC-RT")]
print("\n=== measured-link-time baselines vs the phase-aware methods ===")
print("   (col 1: paired against shipped results; col 2: against SAMEBUILD_DIR rerun)")
for key,lab in NEW:
    print(f"  --- {lab} ---")
    for den,dlab in ((25,"light"),(15,"moderate"),(10,"heavy")):
        cur=cell(den); sb=cell(den,SAMEBUILD) if SAMEBUILD else {}
        st=cur.get(key,{})
        n=sum(1 for sd in st for x in st[sd] if x)
        print(f"    {dlab:8s} (1/{den:2d})  n={n:2d}  seeds={sorted(st)}")
        for m in ("STATIC","LA1","TDOPT_replan"):
            miss=set(st)-set(cur.get(m,{}))
            if miss: print(f"      WARNING unpaired seeds vs {m}: {sorted(miss)}")
        for a,b in ((key,"STATIC"),("LA1",key),("TDOPT_replan",key)):
            A1,B1=(cur,cur)
            A2,B2=((cur,sb) if a==key else (sb,cur)) if sb else ({},{})
            an=lab if a==key else a; bn=lab if b==key else b
            print(f"      {an:17s} - {bn:17s} {fmt(pairdiff(A1,B1,a,b),a)}   "
                  f"{fmt(pairdiff(A2,B2,a,b),a)}")
# the two measured baselines against each other, same instances, same build
print("  --- STATIC-TRAFFIC-RT vs STATIC-TRAFFIC (same instances) ---")
for den,dlab in ((25,"light"),(15,"moderate"),(10,"heavy")):
    cur=cell(den)
    print(f"    {dlab:8s} (1/{den:2d})  "
          f"{fmt(pairdiff(cur,cur,'STATICTRAFFICRT','STATICTRAFFIC'),'RT')}")
print("done")
