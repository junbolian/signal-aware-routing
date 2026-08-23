"""Chunked SUMO runner. Usage:
  python3 sumo_runner.py empty
  python3 sumo_runner.py mod <METHOD> <seed>
Writes JSON results to sumo_results/."""
import os, sys, math, json, random, heapq
import sumo, sumolib, traci
import signal_routing as sr
from lookahead import static_V, la_route

SH=sumo.SUMO_HOME; SUMO_BIN=os.path.join(SH,"bin","sumo")
NETF="grid8.net.xml"
C,GS,GL=120.0,40.0,20.0
TM=sr.Timing(C=C,gs=GS,gl=GL,straight_first=True,rtor=True)
N=8; ORIG,DEST=(7,0),(0,7); V_FREE=8.33
T0S=[0.0,30.0,60.0,90.0]; DECIDE=45.0
os.makedirs("sumo_results",exist_ok=True)

def nid(p): return f"{chr(65+p[0])}{p[1]}"
net=sumolib.net.readNet(NETF)
ELEN={e.getID():e.getLength() for e in net.getEdges()}
TAU0=ELEN[nid((0,0))+nid((1,0))]/V_FREE
rng=random.Random(1000)
ANET=sr.Net(N,rng,C=C,link_lo=TAU0,link_hi=TAU0,offset_mode="random")
VSTAT=static_V(ANET,TM,DEST)
def sedge(u,v): return nid(u)+nid(v)
def e2l(eid):
    if len(eid)==4 and eid[0] in "ABCDEFGH" and eid[2] in "ABCDEFGH":
        return ((ord(eid[0])-65,int(eid[1])),(ord(eid[2])-65,int(eid[3])))
    return None
def udir(a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    return (0 if abs(dx)<1e-6 else (1 if dx>0 else -1),
            0 if abs(dy)<1e-6 else (1 if dy>0 else -1))

def tls_tables():
    T={}
    for tid in traci.trafficlight.getIDList():
        try: links=traci.trafficlight.getControlledLinks(tid)
        except traci.TraCIException: continue
        cls=[]
        for g in links:
            if not g: cls.append(None); continue
            inL,outL,_=g[0]
            ei=net.getEdge(inL.rsplit("_",1)[0]); eo=net.getEdge(outL.rsplit("_",1)[0])
            a=ei.getFromNode().getCoord(); b=ei.getToNode().getCoord(); c2=eo.getToNode().getCoord()
            mt=sr.move_type(udir(a,b),udir(b,c2))
            cls.append((sr.axis_of(udir(a,b)),mt) if mt else None)
        wins=[("NS","S"),("NS","L"),("EW","S"),("EW","L")]
        states=[]
        for w in wins:
            states.append("".join("r" if c is None else
                                  ("g" if c[1]=="R" else ("G" if (c[0],c[1])==w else "r"))
                                  for c in cls))
        node=(ord(tid[0])-65,int(tid[1])) if len(tid)==2 and tid[0] in "ABCDEFGH" else None
        T[tid]=(ANET.offset.get(node,0.0),states)
    return T
B=[0.0,GS,GS+GL,2*GS+GL,C]
def pidx(x):
    for i in range(4):
        if B[i]<=x<B[i+1]: return i
    return 3
def drive(T,cache,t):
    for tid,(off,st) in T.items():
        i=pidx((t-off)%C)
        if cache.get(tid)!=i:
            traci.trafficlight.setRedYellowGreenState(tid,st[i]); cache[tid]=i

def td_from_link(e,t):
    best={e:t}; pred={e:None}; pq=[(t,e)]; goal=None
    while pq:
        tt,ee=heapq.heappop(pq)
        if tt>best.get(ee,1e18)+1e-9: continue
        u,v=ee
        if v==DEST: goal=ee; break
        h=(v[0]-u[0],v[1]-u[1])
        for w,d in ANET.nbrs(v):
            if w==u: continue
            m=sr.move_type(h,d)
            if m is None: continue
            wt=TM.wait(tt,ANET.offset[v],sr.axis_of(h),m)
            t2=tt+wt+ANET.t_cross+ANET.link_t(v,w); e2=(v,w)
            if t2<best.get(e2,1e18)-1e-9:
                best[e2]=t2; pred[e2]=ee; heapq.heappush(pq,(t2,e2))
    path=[]; cur=goal
    while cur is not None: path.append(cur); cur=pred[cur]
    path.reverse(); return path

def nxt_greedy(e,t):
    u,v=e; h=(v[0]-u[0],v[1]-u[1]); opts={}
    for w,d in ANET.nbrs(v):
        if sr.l1(w,DEST)>=sr.l1(v,DEST): continue
        m=sr.move_type(h,d)
        if m: opts[m]=(v,w)
    for m in ("S","L"):
        if m in opts and TM.wait(t,ANET.offset[v],sr.axis_of(h),m)<=1e-9: return opts[m]
    if "R" in opts: return opts["R"]
    for m in ("S","L"):
        if m in opts: return opts[m]
    raise RuntimeError("dead end")

def nxt_la1(e,t):
    u,v=e; h=(v[0]-u[0],v[1]-u[1]); bm=(1e18,None)
    for w,d in ANET.nbrs(v):
        if w==u: continue
        m=sr.move_type(h,d)
        if m is None: continue
        wt=TM.wait(t,ANET.offset[v],sr.axis_of(h),m)
        val=wt+ANET.t_cross+ANET.link_t(v,w)+VSTAT[(v,w)]
        if val<bm[0]-1e-9: bm=(val,(v,w))
    return bm[1]

def first_link(method,t0):
    if method=="STATIC":
        p=sr.static_route(ANET,TM,ORIG,DEST,use_ewait=True); return p[0],p
    if method in ("TDOPT_open","TDOPT_replan"):
        p,_=sr.td_route(ANET,TM,ORIG,DEST,t0); return p[0],p
    if method=="GREEDY":
        return sr.greedy_route(ANET,TM,ORIG,DEST,t0)[0],None
    if method=="LA1":
        return la_route(ANET,TM,ORIG,DEST,t0,1,VSTAT)[0],None
    raise ValueError(method)

def routes_file(path,prob):
    rows=[]
    for y in range(8):
        rows.append(("we%d"%y,[f"left{y}"+nid((0,y))]+[sedge((x,y),(x+1,y)) for x in range(7)]+[nid((7,y))+f"right{y}"]))
        rows.append(("ew%d"%y,[f"right{y}"+nid((7,y))]+[sedge((x,y),(x-1,y)) for x in range(7,0,-1)]+[nid((0,y))+f"left{y}"]))
    for x in range(8):
        rows.append(("sn%d"%x,[f"bottom{x}"+nid((x,0))]+[sedge((x,y),(x,y+1)) for y in range(7)]+[nid((x,7))+f"top{x}"]))
        rows.append(("ns%d"%x,[f"top{x}"+nid((x,7))]+[sedge((x,y),(x,y-1)) for y in range(7,0,-1)]+[nid((x,0))+f"bottom{x}"]))
    with open(path,"w") as f:
        f.write("<routes>\n<vType id=\"bg\" accel=\"2.6\" decel=\"4.5\" length=\"5\" maxSpeed=\"8.33\"/>\n")
        f.write("<vType id=\"egoT\" accel=\"2.6\" decel=\"4.5\" length=\"5\" maxSpeed=\"8.33\" speedFactor=\"1\" speedDev=\"0\"/>\n")
        for n2,eds in rows:
            f.write(f"<route id=\"r_{n2}\" edges=\"{' '.join(eds)}\"/>\n")
            if prob>0:
                f.write(f"<flow id=\"f_{n2}\" type=\"bg\" route=\"r_{n2}\" begin=\"0\" end=\"2600\" probability=\"{prob}\" departLane=\"best\" departSpeed=\"max\"/>\n")
        f.write("</routes>\n")

def run_one(method,t0,prob,seed,rfile,timeout):
    traci.start([SUMO_BIN,"-n",NETF,"-r",rfile,"--seed",str(seed),
                 "--step-length","1","--no-warnings","--no-step-log",
                 "--time-to-teleport","600"])
    try:
        T=tls_tables(); cache={}
        warm=240.0 if prob>0 else 0.0
        depart=warm+t0
        e0,preset=first_link(method,depart)
        if method=="TDOPT_replan":
            redges=[sedge(*e0)]
        else:
            redges=[sedge(*l) for l in preset] if preset else [sedge(*e0)]
        traci.route.add("egoR",redges)
        added=False; adep=None; arr=None; decided=set()
        t=0.0
        while t<timeout:
            drive(T,cache,t)
            traci.simulationStep(); t=traci.simulation.getTime()
            if not added and t>=depart:
                traci.vehicle.add("ego","egoR",typeID="egoT",departPos="0",
                    departLane="best",arrivalPos=str(ELEN[redges[-1]]-10.0))
                added=True
            if added and adep is None and "ego" in traci.vehicle.getIDList():
                adep=t
            if added and adep is not None and "ego" in traci.vehicle.getIDList() \
               and method in ("GREEDY","LA1","TDOPT_replan"):
                eid=traci.vehicle.getRoadID("ego")
                if eid==redges[-1] and eid not in decided and e2l(eid):
                    lk=e2l(eid)
                    if lk[1]!=DEST:
                        pos=traci.vehicle.getLanePosition("ego")
                        if pos>ELEN[eid]-DECIDE:
                            decided.add(eid)
                            tp=t+(ELEN[eid]-pos)/V_FREE
                            if method=="GREEDY": nx=nxt_greedy(lk,tp)
                            elif method=="LA1": nx=nxt_la1(lk,tp)
                            else:
                                p=td_from_link(lk,tp); nx=p[1]
                            ne=sedge(*nx); redges.append(ne)
                            traci.vehicle.setRoute("ego",[eid,ne])
            if added and "ego" in traci.simulation.getArrivedIDList():
                arr=t; break
        return (arr-adep) if arr and adep else None
    finally:
        traci.close()

def save(tag,rec):
    with open(f"sumo_results/{tag}.json","w") as f: json.dump(rec,f)

if __name__=="__main__":
    mode=sys.argv[1]
    if mode=="empty_one":
        method=sys.argv[2]
        routes_file("empty.rou.xml",0.0)
        rec=[]
        for t0 in T0S:
            r=run_one(method,t0,0.0,1,"empty.rou.xml",1800)
            rec.append(r); print(method,"empty",t0,r,flush=True)
        save(f"empty_one_{method}",rec)
    elif mode=="empty":
        routes_file("empty.rou.xml",0.0)
        rec={}
        for m in ["STATIC","GREEDY","LA1","TDOPT_open","TDOPT_replan"]:
            rec[m]=[]
            for t0 in T0S:
                r=run_one(m,t0,0.0,1,"empty.rou.xml",1800)
                rec[m].append(r); print(m,t0,r,flush=True)
        save("empty",rec)
    elif mode=="dem":
        method=sys.argv[2]; seed=int(sys.argv[3]); den=int(sys.argv[4])
        prob=1.0/den; rf=f"dem{den}.rou.xml"
        routes_file(rf,prob)
        rec=[]
        for t0 in T0S:
            r=run_one(method,t0,prob,seed,rf,3600)
            rec.append(r); print("dem",den,method,seed,t0,r,flush=True)
        save(f"dem{den}_{method}_{seed}",rec)
    else:
        method=sys.argv[2]; seed=int(sys.argv[3])
        routes_file("mod.rou.xml",1.0/15.0)
        rec=[]
        for t0 in T0S:
            r=run_one(method,t0,1.0/15.0,seed,"mod.rou.xml",2600)
            rec.append(r); print(method,seed,t0,r,flush=True)
        save(f"mod_{method}_{seed}",rec)
