"""Consolidated publication figures: 5 files into paper/."""
import csv, math, random, glob, json
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal_routing as sr

plt.rcParams.update({
 "font.family":"serif","mathtext.fontset":"cm","font.size":9,
 "axes.labelsize":9,"legend.fontsize":7.5,"xtick.labelsize":8,
 "ytick.labelsize":8,"axes.linewidth":0.8,"figure.dpi":300,
 "savefig.bbox":"tight","axes.grid":True,"grid.alpha":0.25,
 "grid.linewidth":0.5,"legend.frameon":False})
COL={"ff":"#BBBBBB","st":"#4477AA","gr":"#EE6677","la":"#228833",
     "opt":"#000000","mdp":"#AA3377","s5":"#CCBB44","s10":"#66CCEE"}

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    if n<2: return m,0.0
    v=sum((x-m)**2 for x in xs)/(n-1)
    return m,1.96*math.sqrt(v/n)

rows=list(csv.DictReader(open("raw_results.csv")))
def sel(scn,meth): return [r for r in rows if r["scenario"]==scn and r["method"]==meth]
def paired_gap(scn,meth):
    b={(r["net"],r["t0"]):float(r["time"]) for r in sel(scn,"TD-OPT")}
    g=[(float(r["time"])-b[(r["net"],r["t0"])])/b[(r["net"],r["t0"])]*100 for r in sel(scn,meth)]
    return mean_ci(g)

# SP-FF gap on the 40-net subset used by LA/MDP series (seeds 1000..1039)
C,gs,gl=120.0,40.0,20.0
tm=sr.Timing(C=C,gs=gs,gl=gl,straight_first=True,rtor=True)
N=8; orig=(N-1,0); dest=(0,N-1); deps=[k*C/8 for k in range(8)]
ffg=[]
for i in range(40):
    rng=random.Random(1000+i)
    net=sr.Net(N,rng,C=C,link_lo=40.0,link_hi=90.0,offset_mode="random")
    pff=sr.static_route(net,tm,orig,dest,use_ewait=False)
    for t0 in deps:
        a=sr.evaluate(net,tm,pff,t0)["time"]
        op,_=sr.td_route(net,tm,orig,dest,t0)
        b=sr.evaluate(net,tm,op,t0)["time"]
        ffg.append((a-b)/b*100)
ffm,ffci=mean_ci(ffg)
print(f"SP-FF subset gap {ffm:.2f}+-{ffci:.2f}")

# Fig 1: gap ladder, 8 methods (subset-consistent, n=320)
order=[("SP-FF",ffm,ffci,"ff"),("SP-STATIC",16.22,1.07,"st"),
       ("GREEDY",22.12,1.11,"gr"),("LA-1",9.65,.93,"la"),
       ("MDP",10.35,.89,"mdp"),("LA-2",5.33,.69,"la"),
       ("LA-3",3.55,.57,"la"),("TD-OPT",0,0,"opt")]
fig,ax=plt.subplots(figsize=(5.2,3.0))
for i,(n2,g,ci,c) in enumerate(order):
    ax.bar(i,g,yerr=ci,capsize=3,color=COL[c],width=.62)
ax.set_xticks(range(8)); ax.set_xticklabels([o[0] for o in order],rotation=25,fontsize=7.5)
ax.set_ylabel("Gap vs TD-OPT (\\%)")
fig.savefig("figures/fig1_ladder.png"); plt.close(fig)

# Fig 2: VOI (legend instead of on-line text)
sig=sorted({int(r["sigma"]) for r in rows if r["scenario"]=="VOI"})
fig,ax=plt.subplots(figsize=(4.6,3.0))
ys=[];es=[]
for s in sig:
    t,ci=mean_ci([float(r["time"]) for r in rows if r["scenario"]=="VOI" and int(r["sigma"])==s])
    ys.append(t); es.append(ci)
ax.errorbar(sig,ys,yerr=es,marker="o",ms=3.5,color=COL["opt"],lw=1,capsize=2.5,
            label="TD plan under offset error $\\sigma$")
for m,c,ls in (("SP-STATIC","st","--"),("GREEDY","gr",":"),("TD-OPT","opt","-.")):
    t,_=mean_ci([float(r["time"]) for r in sel("base_N8",m)])
    ax.axhline(t,color=COL[c],ls=ls,lw=.9,label=m)
ax.set_xlabel("Offset knowledge error $\\sigma$ (s), $C=120$ s")
ax.set_ylabel("Mean realized travel time (s)")
ax.set_ylim(940,1180)
ax.legend(loc="lower right", bbox_to_anchor=(1.0, 0.25))
fig.savefig("figures/fig2_voi.png"); plt.close(fig)

# Fig 3: scaling, two panels (a heterogeneity, b grid size)
rows2=[]
import csv as _csv
rows2=list(_csv.DictReader(open("gini_grid_results.csv")))
fig,(a1,a2)=plt.subplots(1,2,figsize=(6.9,2.9))
sty={"uniform":("#4477AA","o"),"triangular":("#228833","s"),"lognormal":("#CCBB44","^")}
for fam,(c,mk) in sty.items():
    fx=[float(r["Delta"]) for r in rows2 if r["family"]==fam]
    fy=[float(r["adv"]) for r in rows2 if r["family"]==fam]
    fe=[float(r["ci"]) for r in rows2 if r["family"]==fam]
    a1.errorbar(fx,fy,yerr=fe,fmt=mk,ms=4,elinewidth=1,capsize=2.5,color=c,label=fam)
ax_=[float(r["Delta"]) for r in rows2]; ay_=[float(r["adv"]) for r in rows2]
cx=sum(ax_)/len(ax_); cy=sum(ay_)/len(ay_)
a1.plot([2,23],[cy-7*(x-cx) for x in (2,23)],ls=":",lw=1,color="#000000",label="prediction slope $-J/2$")
a1.axhline(0,color="k",lw=.7)
a1.set_xlabel("Gini mean difference $\\Delta$ (s)")
a1.set_ylabel("GREEDY $-$ SP-STATIC (s)")
a1.legend(loc="lower left")
grids=[5,8,12]
for m,c,mk in (("SP-STATIC","st","s"),("GREEDY","gr","^")):
    ys=[];es=[]
    for Ng in grids:
        scn="base_N8" if Ng==8 else f"grid_N{Ng}"
        g,ci=paired_gap(scn,m); ys.append(g); es.append(ci)
    a2.errorbar(grids,ys,yerr=es,marker=mk,ms=4,lw=1,capsize=2.5,color=COL[c],label=m)
a2.set_xlabel("Grid size $N$")
a2.set_ylabel("Gap vs TD-OPT (\\%)")
a2.set_xticks(grids); a2.set_ylim(8,30); a2.legend(loc="upper left")
for ax_,lab in ((a1,"a"),(a2,"b")):
    ax_.text(0.02,1.03,lab,transform=ax_.transAxes,fontweight="bold",fontsize=10)
fig.tight_layout()
fig.savefig("figures/fig3_scaling.png"); plt.close(fig)

# Fig 4: lookahead, two panels (a depth sigma=0; b depth x accuracy)
het=[16.22,9.65,5.33,3.55,2.76,1.38]; hetci=[1.07,.93,.69,.57,.50,.40]
uni=[19.48,9.06,5.48,3.07]; unici=[1.01,.69,.56,.43]
ks=[1,2,3,4,5]
s5=[10.49,7.38,6.12,4.47,3.38]; s5c=[.98,.92,.76,.67,.62]
s10=[10.58,9.93,6.37,5.47,5.45]; s10c=[.95,1.11,.81,.71,.80]
fig,(a1,a2)=plt.subplots(1,2,figsize=(6.9,2.9),sharey=True)
a1.errorbar(range(6),het,yerr=hetci,marker="o",ms=4,lw=1,capsize=2.5,
            color=COL["la"],label="heterogeneous links")
a1.errorbar(range(4),uni,yerr=unici,marker="s",ms=4,lw=1,capsize=2.5,
            color=COL["st"],label="uniform links")
a1.axhline(22.12,color=COL["gr"],ls=":",lw=.9,label="GREEDY (het.)")
a1.set_xlabel("Lookahead depth $k$ ($k{=}0$: SP-STATIC)")
a1.set_ylabel("Gap vs TD-OPT (\\%)")
a1.set_xticks(range(6)); a1.legend()
a2.axhline(16.22,color=COL["st"],ls="--",lw=.9,label="SP-STATIC")
for s,c,mk,ser,serc in ((0,"opt","o",het[1:],hetci[1:]),(5,"s5","s",s5,s5c),(10,"s10","^",s10,s10c)):
    a2.errorbar(ks,ser,yerr=serc,marker=mk,ms=4,lw=1,capsize=2.5,color=COL[c],
                label=f"$\\sigma={s}$ s")
a2.set_xlabel("Lookahead depth $k$")
a2.set_xticks(ks); a2.legend()
for ax_,lab in ((a1,"a"),(a2,"b")):
    ax_.text(0.02,1.03,lab,transform=ax_.transAxes,fontweight="bold",fontsize=10)
fig.tight_layout()
fig.savefig("figures/fig4_lookahead.png"); plt.close(fig)

# Fig 5 (sumo): restyle only, same content, keep filename fig8_sumo.png
mod={}
for f in glob.glob("sumo_results/mod_*.json"):
    p=f.split("/")[-1][:-5].split("_"); meth="_".join(p[1:-1])
    mod.setdefault(meth,[]).extend(x for x in json.load(open(f)) if x)
names=[("SP-STATIC","STATIC","st"),("GREEDY","GREEDY","gr"),("LA-1","LA1","la"),
       ("TD-OPT open","TDOPT_open","ff"),("TD-OPT replan","TDOPT_replan","mdp")]
fig,ax=plt.subplots(figsize=(4.6,3.0))
for i,(lab,key,c) in enumerate(names):
    t,ci=mean_ci(mod[key])
    ax.bar(i,t,yerr=ci,capsize=3,color=COL[c],width=.62)
ax.set_xticks(range(5)); ax.set_xticklabels([n[0] for n in names],rotation=25,fontsize=7.5)
ax.set_ylabel("Mean travel time (s)"); ax.set_ylim(1000,1440)
fig.savefig("figures/fig8_sumo.png"); plt.close(fig)
print("figures v2 done")
