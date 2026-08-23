"""Grid-level test of the Gini law: greedy-vs-static advantage against Delta
for three link-time families matched in mean (65 s). Prediction: one line,
slope about -J/2 per unit Delta (J=14 on the 8x8 corner trip)."""
import random, math, zlib
import signal_routing as sr

C, gs, gl, MU = 120.0, 40.0, 20.0, 65.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8; orig = (N-1, 0); dest = (0, N-1); deps = [k*C/8 for k in range(8)]

def lognormal_params(mean, sd):
    s2 = math.log(1 + (sd/mean)**2)
    return math.log(mean) - s2/2, math.sqrt(s2)

def make_sampler(fam, par, rng):
    if fam == "uniform":    return lambda: MU + rng.uniform(-par, par)
    if fam == "triangular": return lambda: rng.triangular(MU-par, MU+par)
    if fam == "lognormal":
        m, s = lognormal_params(MU, par)
        return lambda: max(5.0, rng.lognormvariate(m, s))
    raise ValueError(fam)

def delta_of(fam, par, rng, n=400000):
    f = make_sampler(fam, par, rng)
    return sum(abs(f()-f()) for _ in range(n)) / n

def mean_ci(xs):
    n=len(xs); m=sum(xs)/n
    v=sum((x-m)**2 for x in xs)/(n-1)
    return m, 1.96*math.sqrt(v/n)

def net_with_sampler(seed, fam, par):
    rng = random.Random(seed)
    net = sr.Net(N, rng, C=C, link_lo=MU, link_hi=MU, offset_mode="random")
    srng = random.Random(seed + 777)
    f = make_sampler(fam, par, srng)
    for k in net.tau: net.tau[k] = f()
    return net

cells = [("uniform",[5,10,15,20,25]), ("triangular",[8,15,22,30,37]),
         ("lognormal",[4,8,12,16,20])]
print("family        par    Delta    adv(s)   ci")
rows=[]
for fam, pars in cells:
    for par in pars:
        drng = random.Random(zlib.crc32(f"{fam}-{par}".encode()))
        Delta = delta_of(fam, par, drng)
        adv=[]
        for i in range(80):
            net = net_with_sampler(1000+i, fam, par)
            p = sr.static_route(net, tm, orig, dest, use_ewait=True)
            for t0 in deps:
                a = sr.evaluate(net, tm, p, t0)["time"]
                b = sr.evaluate(net, tm, sr.greedy_route(net, tm, orig, dest, t0), t0)["time"]
                adv.append(a-b)
        m, ci = mean_ci(adv)
        rows.append((fam, Delta, m, ci))
        print(f"{fam:12s} {par:5.0f}  {Delta:6.2f}  {m:8.2f}  {ci:5.2f}")

import csv
w=csv.writer(open("gini_grid_results.csv","w")); w.writerow(["family","Delta","adv","ci"])
for r in rows: w.writerow(r)
# pooled OLS of advantage on Delta across all families
xs=[r[1] for r in rows]; ys=[r[2] for r in rows]
mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
sl=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs)
ic=my-sl*mx
# per-family slopes
for fam,_ in cells:
    fx=[r[1] for r in rows if r[0]==fam]; fy=[r[2] for r in rows if r[0]==fam]
    fmx=sum(fx)/len(fx); fmy=sum(fy)/len(fy)
    sxx=sum((x-fmx)**2 for x in fx)
    fsl=sum((x-fmx)*(y-fmy) for x,y in zip(fx,fy))/sxx
    fic=fmy-fsl*fmx
    res=[y-(fic+fsl*x) for x,y in zip(fx,fy)]
    se=math.sqrt(sum(e*e for e in res)/(len(fx)-2)/sxx)
    print(f"slope[{fam}] = {fsl:.3f} +- {1.96*se:.3f} per unit Delta")
print(f"pooled slope = {sl:.3f} per unit Delta (prediction -J/2 = -7.0), intercept {ic:.2f}")
