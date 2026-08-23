"""Per-network slope statistics reported in the paper.
Part A: uniform family, original protocol, 160 networks, slope in half-width h.
Part B: uniform/triangular/lognormal families, 80 networks, slope in the Gini
mean difference Delta. Each network contributes one fitted slope; the paper
reports the mean and 95% CI across networks. Outputs two JSON files."""
import random, math, json, zlib
import signal_routing as sr

C, gs, gl, MU = 120.0, 40.0, 20.0, 65.0
tm = sr.Timing(C=C, gs=gs, gl=gl, straight_first=True, rtor=True)
N = 8; orig = (N-1, 0); dest = (0, N-1); deps = [k*C/8 for k in range(8)]

def slopes_stats(slopes):
    n = len(slopes); m = sum(slopes)/n
    sd = math.sqrt(sum((s-m)**2 for s in slopes)/(n-1))
    return m, 1.96*sd/math.sqrt(n)

def adv(net):
    p = sr.static_route(net, tm, orig, dest, use_ewait=True)
    out = []
    for t0 in deps:
        a = sr.evaluate(net, tm, p, t0)["time"]
        b = sr.evaluate(net, tm, sr.greedy_route(net, tm, orig, dest, t0), t0)["time"]
        out.append(a-b)
    return sum(out)/len(out)

# Part A: uniform h-sweep, original protocol
spreads = [0, 5, 10, 15, 20, 25]; NN_A = 160
pernet = {i: {} for i in range(NN_A)}
for h in spreads:
    for i in range(NN_A):
        rng = random.Random(1000+i)
        net = sr.Net(N, rng, C=C, link_lo=MU-h, link_hi=MU+h, offset_mode="random")
        pernet[i][h] = adv(net)
mx = sum(spreads)/len(spreads); sxx = sum((x-mx)**2 for x in spreads)
sl_A = []
for i in range(NN_A):
    ys = [pernet[i][h] for h in spreads]; my = sum(ys)/len(ys)
    sl_A.append(sum((x-mx)*(y-my) for x, y in zip(spreads, ys))/sxx)
mA, ciA = slopes_stats(sl_A)
print(f"[A] uniform h-slope: {mA:.3f} +- {ciA:.3f} (95% CI, {NN_A} networks; prediction -14/3 = {-14/3:.3f})")
json.dump({"uniform_slope_h": mA, "ci": ciA, "n_networks": NN_A}, open("uniform_slope_pernet.json", "w"))

# Part B: three families against Delta
def lognormal_params(mean, sd):
    s2 = math.log(1+(sd/mean)**2); return math.log(mean)-s2/2, math.sqrt(s2)
def make_sampler(fam, par, rng):
    if fam == "uniform":    return lambda: MU + rng.uniform(-par, par)
    if fam == "triangular": return lambda: rng.triangular(MU-par, MU+par)
    m, s = lognormal_params(MU, par); return lambda: max(5.0, rng.lognormvariate(m, s))
cells = [("uniform", [5,10,15,20,25]), ("triangular", [8,15,22,30,37]), ("lognormal", [4,8,12,16,20])]
NN_B = 80; out = {}
for fam, pars in cells:
    deltas = {}
    for par in pars:
        drng = random.Random(zlib.crc32(f"{fam}-{par}".encode()))
        f = make_sampler(fam, par, drng)
        deltas[par] = sum(abs(f()-f()) for _ in range(400000))/400000
    pernet = {i: {} for i in range(NN_B)}
    for par in pars:
        for i in range(NN_B):
            rng = random.Random(1000+i)
            net = sr.Net(N, rng, C=C, link_lo=MU, link_hi=MU, offset_mode="random")
            srng = random.Random(1000+i+777)
            f = make_sampler(fam, par, srng)
            for k in net.tau: net.tau[k] = f()
            pernet[i][par] = adv(net)
    xs = [deltas[p] for p in pars]; fmx = sum(xs)/len(xs); fsxx = sum((x-fmx)**2 for x in xs)
    sl = []
    for i in range(NN_B):
        ys = [pernet[i][p] for p in pars]; fmy = sum(ys)/len(ys)
        sl.append(sum((x-fmx)*(y-fmy) for x, y in zip(xs, ys))/fsxx)
    m, ci = slopes_stats(sl)
    out[fam] = {"slope": m, "ci": ci}
    print(f"[B] {fam:11s} Delta-slope: {m:.3f} +- {ci:.3f} (95% CI, {NN_B} networks; prediction -7.0)")
json.dump(out, open("gini_family_slopes.json", "w"))
