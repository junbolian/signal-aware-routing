"""Chunked SUMO runner. Usage:
  python3 sumo_runner.py empty
  python3 sumo_runner.py mod <METHOD> <seed>
  python3 sumo_runner.py dem <METHOD> <seed> <denom>
METHOD is one of STATIC, GREEDY, LA1, MDP, TDOPT_open, TDOPT_replan,
STATIC-TRAFFIC, STATIC-TRAFFIC-RT, STATIC-Q, MDP-Q, TDOPT-Q.
MDP is the paper's unknown-offset local-information optimum (Proposition 7)
executed in SUMO. The -Q methods are the queue-aware planners of
queue_model.py: they estimate the arrival rate of every approach lane from the
standing queues at its recent green onsets, observe the platoon ahead of the
ego at every decision point, and carry the platoon it rides as state; MDP-Q
solves Proposition 7's Bellman equation with the discharge-aware wait,
STATIC-Q its phase-blind counterpart, TDOPT-Q the full-timing plan.
Writes JSON results to sumo_results/ and, for every replanning method, a
per-crossing prediction diagnostic to sumo_results/diag_<tag>.json."""
import os, sys, math, json, random, heapq
import sumo, sumolib, traci
import signal_routing as sr
from lookahead import static_V, la_route
import queue_model as qm
import queue_kernel as qk
qm.load_calibration()
KERNEL=None   # set per scenario in main (kernel_<prefix>.json)

SH=sumo.SUMO_HOME; SUMO_BIN=os.path.join(SH,"bin","sumo"+(".exe" if os.name=="nt" else ""))
NETF="grid8.net.xml"
C,GS,GL=120.0,40.0,20.0
TM=sr.Timing(C=C,gs=GS,gl=GL,straight_first=True,rtor=True)
N=8; ORIG,DEST=(7,0),(0,7); V_FREE=8.33
T0S=[0.0,30.0,60.0,90.0]; DECIDE=45.0
MEAS_WIN=240.0; MEAS_EVERY=20.0
ALIAS={"STATIC-TRAFFIC":"STATICTRAFFIC","STATIC-TRAFFIC-RT":"STATICTRAFFICRT",
       "STATIC-Q":"STATICQ","MDP-Q":"MDPQ","TDOPT-Q":"TDOPTQ"}
QMETH=("STATICQ","MDPQ","TDOPTQ")
ROLLING=("GREEDY","LA1","MDP","TDOPT_replan")+QMETH
RT_VMIN=2.0
LASTRUN={}
os.makedirs("sumo_results",exist_ok=True)

def nid(p): return f"{chr(65+p[0])}{p[1]}"
net=sumolib.net.readNet(NETF)
ELEN={e.getID():e.getLength() for e in net.getEdges()}
TAU0=ELEN[nid((0,0))+nid((1,0))]/V_FREE
rng=random.Random(1000)
ANET=sr.Net(N,rng,C=C,link_lo=TAU0,link_hi=TAU0,offset_mode="random")
VSTAT=static_V(ANET,TM,DEST)
def sedge(u,v): return nid(u)+nid(v)
VMDP0,_=qm.solve_values(ANET,TM,DEST,None,sedge)   # Proposition 7 value, no queues
VMDP={e:float(v[0]) for e,v in VMDP0.items()}
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

def measure_links(prob,seed,rfile,queue_free=False):
    """One traffic snapshot per (seed,demand): sample every MEAS_EVERY s over the
    warmup window, which ends before any probe departs. Returns the measured link
    times and the diagnostics of the measurement.

    queue_free=False (STATIC-TRAFFIC): every occupied edge is sampled, so a vehicle
    stopped at a red light enters the link cost as signal delay.
    queue_free=True (STATIC-TRAFFIC-RT): only edges with no halting vehicle are
    sampled, and the speed floor is RT_VMIN, so the result is a running-time
    surface with the signal delay left out of the link cost."""
    traci.start([SUMO_BIN,"-n",NETF,"-r",rfile,"--seed",str(seed),
                 "--step-length","1","--no-warnings","--no-step-log",
                 "--time-to-teleport","600"])
    try:
        T=tls_tables(); cache={}
        eids=[e.getID() for e in net.getEdges()]
        ff={e:ELEN[e]/net.getEdge(e).getSpeed() for e in eids}
        acc={e:[] for e in eids}; nsamp=0
        t=0.0
        while t<MEAS_WIN-1e-9:
            drive(T,cache,t)
            traci.simulationStep(); t=traci.simulation.getTime()
            if abs(t%MEAS_EVERY)<1e-9:
                nsamp+=1
                for e in eids:
                    if traci.edge.getLastStepVehicleNumber(e)<=0: continue
                    if queue_free:
                        if traci.edge.getLastStepHaltingNumber(e)!=0: continue
                        acc[e].append(ELEN[e]/max(traci.edge.getLastStepMeanSpeed(e),RT_VMIN))
                    else:
                        acc[e].append(ELEN[e]/max(traci.edge.getLastStepMeanSpeed(e),0.1))
        raw={e:(sum(acc[e])/len(acc[e]) if acc[e] else ff[e]) for e in eids}
        meas={e:max(raw[e],ff[e]) for e in eids}
        diag={"nsamp":nsamp,"n_edges":len(eids),
              "frac_sampled":sum(1 for e in eids if acc[e])/len(eids),
              "frac_raw_ge_ff":sum(1 for e in eids if raw[e]>=ff[e]-1e-9)/len(eids),
              "ratio":sorted(meas[e]/ff[e] for e in eids)}
        return meas,diag
    finally:
        traci.close()

def traffic_route(meas,orig,dest):
    """Plain Dijkstra on measured link times. No signal term is added: the
    measurement already contains the queueing and signal delay the background
    traffic experienced, so an E-wait term would double-count it."""
    def lt(u,v): return meas[sedge(u,v)]
    best={}; pred={}; pq=[]
    for w,d in ANET.nbrs(orig):
        e=(orig,w); c=lt(orig,w)
        if c<best.get(e,1e18): best[e]=c; pred[e]=None; heapq.heappush(pq,(c,e))
    goal=None
    while pq:
        c,e=heapq.heappop(pq)
        if c>best.get(e,1e18)+1e-9: continue
        u,v=e
        if v==dest: goal=e; break
        h=(v[0]-u[0],v[1]-u[1])
        for w,d in ANET.nbrs(v):
            if w==u: continue
            m=sr.move_type(h,d)
            if m is None: continue
            c2=c+ANET.t_cross+lt(v,w); e2=(v,w)
            if c2<best.get(e2,1e18)-1e-9:
                best[e2]=c2; pred[e2]=e; heapq.heappush(pq,(c2,e2))
    path=[]; cur=goal
    while cur is not None: path.append(cur); cur=pred[cur]
    path.reverse(); return path

class MeasNet:
    """ANET with the link-time vector replaced by the measurement. Only link_t
    changes; nbrs, t_cross and the offsets are ANET's, so sr.static_route runs
    unmodified on it and keeps SP-STATIC's expected-signal-delay term."""
    def __init__(self,meas): self.meas=meas; self.t_cross=ANET.t_cross; self.N=ANET.N
    def nbrs(self,v): return ANET.nbrs(v)
    def link_t(self,u,v): return self.meas[sedge(u,v)]
    # the topology queries are ANET's; only the link-time vector is replaced
    def axis(self,v,h): return ANET.axis(v,h)
    def move_type(self,v,h,d): return ANET.move_type(v,h,d)
    def dist(self,v,dest): return ANET.dist(v,dest)

APPROACH_LANES=[(e.getID(),l.getIndex(),l.getID()) for e in net.getEdges()
                if e2l(e.getID()) for l in e.getLanes()]
JLANES={}   # junction id -> [(edge id, lane index, lane id, through phase index)]
for eid,li,lid in APPROACH_LANES:
    u,v=e2l(eid); h=(v[0]-u[0],v[1]-u[1])
    JLANES.setdefault(nid(v),[]).append((eid,li,lid,0 if sr.axis_of(h)=="NS" else 2))
HALT=traci.constants.LAST_STEP_VEHICLE_HALTING_NUMBER

class RateEstimator:
    """Arrival-rate estimate per approach lane, what a traffic-aware router
    learns from probe data: the standing queue counted at each through-green
    onset divided by the through red, averaged over the last ONSET_WINDOW
    onsets of that lane. Fed from lane subscriptions each step."""
    def __init__(self):
        self.onsets={k:[] for k in ((eid,li) for eid,li,_ in APPROACH_LANES)}
        for _,_,lid in APPROACH_LANES: traci.lane.subscribe(lid,[HALT])
        self.rS=C-GS
    def observe(self,T,t):
        """Call right after simulationStep at time t: the results describe the
        state at t, before drive() applies the phase of time t."""
        res=traci.lane.getAllSubscriptionResults()
        for tid,(off,st) in T.items():
            i_now=pidx((t-off)%C); i_prev=pidx((t-1-off)%C)
            if i_now==i_prev or tid not in JLANES: continue
            for eid,li,lid,tidx in JLANES[tid]:
                if tidx==i_now:
                    self.onsets[(eid,li)].append(res[lid][HALT])
    def rates(self):
        out={}
        for k,q in self.onsets.items():
            w=q[-qm.ONSET_WINDOW:]
            out[k]=(sum(w)/len(w))/self.rS if w else 0.0
        return out

def platoons(eid,ego_pos,t):
    """Vehicles ahead of the ego on each lane of its current edge, as unimpeded
    stopline arrival times ordered nearest-first; a standing vehicle (speed
    below 0.1 m/s) is already at the queue and is given arrival time t."""
    out={}
    for li in range(net.getEdge(eid).getLaneNumber()):
        lid=f"{eid}_{li}"; rows=[]
        for vid in traci.lane.getLastStepVehicleIDs(lid):
            if vid=="ego": continue
            pos=traci.vehicle.getLanePosition(vid)
            if pos<=ego_pos: continue
            arr=t if traci.vehicle.getSpeed(vid)<0.1 else t+(ELEN[eid]-pos)/V_FREE
            rows.append((pos,arr))
        rows.sort(reverse=True)
        out[li]=[a for _,a in rows]
    return out

def move_of(e,e2):
    (u,v),(v2,w)=e,e2
    return sr.move_type((v[0]-u[0],v[1]-u[1]),(w[0]-v2[0],w[1]-v2[1]))

def predicted_pass(method,e,e2,tp,qs,pl):
    """Planned stopline passage time of the chosen movement, for the
    prediction diagnostic: TM.wait for the queue-ignorant planners, the
    discharge model (or its phase expectation for STATIC-Q) for the -Q ones."""
    (u,v)=e; m=move_of(e,e2); ax=sr.axis_of((v[0]-u[0],v[1]-u[1]))
    if method=="STATICQ": return tp+qm.head_ewait(tp,ax,m,TM,pl)
    if method in ("MDPQ","TDOPTQ"): return qm.head_pass(tp,ANET.offset[v],ax,m,TM,pl)
    return tp+TM.wait(tp,ANET.offset[v],ax,m)

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

def mdp_first_link(V):
    best=(1e18,None)
    for w,d in ANET.nbrs(ORIG):
        c=ANET.link_t(ORIG,w)+V[(ORIG,w)]
        if c<best[0]-1e-9: best=(c,(ORIG,w))
    return best[1]

def nxt_mdp(e,t):
    """Proposition 7's threshold rule: observed local waits plus continuation
    constants, over the L1-monotone movements."""
    u,v=e; ax=sr.axis_of((v[0]-u[0],v[1]-u[1])); bm=(1e18,None)
    for m,e2,_ in qm.monotone_moves(ANET,e,DEST):
        val=TM.wait(t,ANET.offset[v],ax,m)+ANET.t_cross+ANET.link_t(*e2)+VMDP[e2]
        if val<bm[0]-1e-9: bm=(val,e2)
    return bm[1]

def first_link(method,t0,meas=None,qs=None,pbe=None):
    """qs: arrival-rate estimate per approach lane; pbe: observed platoons on
    each candidate first edge (the ego sees its first link at departure)."""
    if method=="MDP":
        return mdp_first_link(VMDP),None
    if method in ("STATICQ","MDPQ"):
        V,_=qm.solve_values(ANET,TM,DEST,KERNEL,sedge,static=(method=="STATICQ"))
        return qm.first_link(ANET,ORIG,V,pbe,sedge),None
    if method=="TDOPTQ":
        return qm.tdoptq_first_link(ANET,TM,ORIG,t0,DEST,qs,pbe,sedge),None
    if method=="STATICTRAFFIC":
        p=traffic_route(meas,ORIG,DEST); return p[0],p
    if method=="STATICTRAFFICRT":
        p=sr.static_route(MeasNet(meas),TM,ORIG,DEST,use_ewait=True); return p[0],p
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

def run_one(method,t0,prob,seed,rfile,timeout,meas=None):
    traci.start([SUMO_BIN,"-n",NETF,"-r",rfile,"--seed",str(seed),
                 "--step-length","1","--no-warnings","--no-step-log",
                 "--time-to-teleport","600"])
    try:
        T=tls_tables(); cache={}
        warm=240.0 if prob>0 else 0.0
        depart=warm+t0
        redges=None
        if method not in QMETH:
            e0,preset=first_link(method,depart,meas)
            if method=="TDOPT_replan":
                redges=[sedge(*e0)]
            else:
                redges=[sedge(*l) for l in preset] if preset else [sedge(*e0)]
            traci.route.add("egoR",redges)
        added=False; adep=None; arr=None; decided=set(); tpl=[]
        pending={}; diag=[]; last_edge=None
        est=RateEstimator() if method in QMETH else None
        t=0.0
        while t<timeout:
            drive(T,cache,t)
            traci.simulationStep(); t=traci.simulation.getTime()
            if est is not None: est.observe(T,t)
            tpl+= list(traci.simulation.getStartingTeleportIDList())
            if not added and t>=depart:
                if method in QMETH:
                    pbe={sedge(ORIG,w):platoons(sedge(ORIG,w),-1.0,t) for w,_ in ANET.nbrs(ORIG)}
                    e0,_=first_link(method,depart,qs=est.rates(),pbe=pbe)
                    redges=[sedge(*e0)]; traci.route.add("egoR",redges)
                traci.vehicle.add("ego","egoR",typeID="egoT",departPos="0",
                    departLane="best",arrivalPos=str(ELEN[redges[-1]]-10.0))
                added=True
            if added and adep is None and "ego" in traci.vehicle.getIDList():
                adep=t
            if added and adep is not None and "ego" in traci.vehicle.getIDList():
                eid=traci.vehicle.getRoadID("ego")
                if last_edge is not None and eid!=last_edge and last_edge in pending:
                    rec=pending.pop(last_edge); rec["cross"]=t; diag.append(rec)
                last_edge=eid
                if method in ROLLING and eid==redges[-1] and eid not in decided and e2l(eid):
                    lk=e2l(eid)
                    if lk[1]!=DEST:
                        pos=traci.vehicle.getLanePosition("ego")
                        if pos>ELEN[eid]-DECIDE:
                            decided.add(eid)
                            tp=t+(ELEN[eid]-pos)/V_FREE
                            qs=pl=None
                            off=ANET.offset[lk[1]]
                            if method=="GREEDY": nx=nxt_greedy(lk,tp)
                            elif method=="LA1": nx=nxt_la1(lk,tp)
                            elif method=="MDP": nx=nxt_mdp(lk,tp)
                            elif method=="TDOPT_replan":
                                p=td_from_link(lk,tp); nx=p[1]
                            else:
                                qs=est.rates(); pl=platoons(eid,pos,t)
                                if method in ("STATICQ","MDPQ"):
                                    VQ,_=qm.solve_values(ANET,TM,DEST,KERNEL,sedge,static=(method=="STATICQ"))
                                    nx=qm.decide(ANET,TM,lk,tp,DEST,VQ,pl,KERNEL,off=(off if method=="MDPQ" else None))
                                else: nx=qm.tdoptq_path(ANET,TM,lk,tp,DEST,qs,pl,sedge)[0][1]
                            if pl is None: pl=platoons(eid,pos,t)
                            m=move_of(lk,nx)
                            if os.environ.get("SR_VERBOSE"):
                                opts=[]
                                for mm,e2,ax in qm._moves(ANET,lk):
                                    hp=(qm.head_pass(tp,off,ax,mm,TM,pl)-tp) if qs is not None else TM.wait(tp,off,ax,mm)
                                    qmin=min(len(pl.get(l,[])) for l in qm.lanes_for(mm))
                                    if method=="MDP": tv=VMDP[e2]
                                    elif method=="LA1": tv=VSTAT[e2]
                                    elif method in ("STATICQ","MDPQ"): tv=float(VQ[e2][min(qmin,qm.QMAX)])
                                    else: tv=float("nan")
                                    opts.append(f"{mm}:head={hp:5.1f} q={[len(pl.get(l,[])) for l in qm.lanes_for(mm)]} tail={tv:7.1f}")
                                print(f"    t={t:.0f} {eid} phase={(tp-off)%C:5.1f} choose {m} | "+" | ".join(opts),flush=True)
                            qa=min(len(pl.get(l,[])) for l in qm.lanes_for(m))
                            pending[eid]={"edge":eid,"move":m,"q_ahead":qa,"t_decide":t,
                                          "t_free":tp,"pred":predicted_pass(method,lk,nx,tp,qs,pl)}
                            ne=sedge(*nx); redges.append(ne)
                            traci.vehicle.setRoute("ego",[eid,ne])
            if added and "ego" in traci.simulation.getArrivedIDList():
                arr=t; break
        LASTRUN.update({"teleports":tpl,"route":list(redges),"arr":arr,"dep":adep,"diag":diag})
        return (arr-adep) if arr and adep else None
    finally:
        traci.close()

def save(tag,rec,diag=None):
    with open(f"sumo_results/{tag}.json","w") as f: json.dump(rec,f)
    if diag:
        with open(f"sumo_results/diag_{tag}.json","w") as f: json.dump(diag,f)

def measure_one(method,prob,seed,rfile):
    """Traffic snapshot, taken once per seed and reused across the four
    departures; only STATIC-TRAFFIC consumes it."""
    if method not in ("STATICTRAFFIC","STATICTRAFFICRT"): return None
    meas,d=measure_links(prob,seed,rfile,queue_free=(method=="STATICTRAFFICRT"))
    q=d["ratio"]; n=len(q)
    print(f"measure {method} seed={seed} samples={d['nsamp']} edges={d['n_edges']} "
          f"sampled={100*d['frac_sampled']:.1f}% raw>=ff={100*d['frac_raw_ge_ff']:.1f}% "
          f"ratio med={q[n//2]:.3f} p90={q[int(.9*n)]:.3f} max={q[-1]:.3f}",flush=True)
    return meas

def report(method,row):
    print(*row,flush=True)
    if method in ("STATICTRAFFIC","STATICTRAFFICRT")+QMETH:
        tp=LASTRUN.get("teleports",[])
        print("   route",">".join(LASTRUN.get("route",[])),"teleports",len(tp),
              (tp if tp else ""),flush=True)

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
        method=ALIAS.get(sys.argv[2],sys.argv[2]); seed=int(sys.argv[3]); den=int(sys.argv[4])
        prob=1.0/den; rf=f"dem{den}.rou.xml"
        routes_file(rf,prob)
        meas=measure_one(method,prob,seed,rf)
        rec=[]; dg=[]
        for t0 in T0S:
            r=run_one(method,t0,prob,seed,rf,3600,meas)
            rec.append(r); report(method,("dem",den,method,seed,t0,r)); dg.append(LASTRUN.get("diag",[]))
        save(f"dem{den}_{method}_{seed}",rec,dg)
    else:
        method=ALIAS.get(sys.argv[2],sys.argv[2]); seed=int(sys.argv[3])
        routes_file("mod.rou.xml",1.0/15.0)
        meas=measure_one(method,1.0/15.0,seed,"mod.rou.xml")
        rec=[]; dg=[]
        for t0 in T0S:
            r=run_one(method,t0,1.0/15.0,seed,"mod.rou.xml",2600,meas)
            rec.append(r); report(method,(method,seed,t0,r)); dg.append(LASTRUN.get("diag",[]))
        save(f"mod_{method}_{seed}",rec,dg)
