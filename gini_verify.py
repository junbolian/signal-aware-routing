"""Verify the general ladder law A = rho(1-rho)r/2 - Delta/2 (Delta = Gini mean
difference E|X-Y|) beyond the uniform family. Distributions matched in mean."""
import random, math
C, r, mu = 120.0, 80.0, 65.0
rho = r / C
N = 2_000_000
rng = random.Random(7)

def lognormal_params(mean, sd):
    s2 = math.log(1 + (sd / mean) ** 2)
    return math.log(mean) - s2 / 2, math.sqrt(s2)

MLN, SLN = lognormal_params(mu, 15.0)
dists = {
    "uniform h=10":    lambda: mu + rng.uniform(-10, 10),
    "uniform h=25":    lambda: mu + rng.uniform(-25, 25),
    "triangular h=15": lambda: rng.triangular(mu - 15, mu + 15),
    "lognormal sd=15": lambda: rng.lognormvariate(MLN, SLN),
}
print(f"rho={rho:.4f}  rho(1-rho)r/2={rho*(1-rho)*r/2:.4f}")
for name, F in dists.items():
    sA = sD = sA2 = 0.0
    for _ in range(N):
        x, y = F(), F()
        red1, red2 = rng.random() < rho, rng.random() < rho
        res1, res2 = rng.uniform(0, r), rng.uniform(0, r)
        cmin, csec = (x, y) if x <= y else (y, x)
        red_c, res_c = (red1, res1) if x <= y else (red2, res2)
        static = cmin + (res_c if red_c else 0.0)
        if not red1:   greedy = x
        elif not red2: greedy = y
        else:          greedy = res1 + x
        a = static - greedy
        d = abs(x - y)
        sA += a; sA2 += a * a; sD += d
    A_sim = sA / N
    se = math.sqrt((sA2 / N - A_sim ** 2) / N) * 1.96
    Delta = sD / N
    A_pred = rho * (1 - rho) * r / 2 - Delta / 2
    print(f"{name:18s} Delta={Delta:7.3f}  A_sim={A_sim:7.4f}+-{se:.4f}  A_pred={A_pred:7.4f}  |diff|={abs(A_sim-A_pred):.4f}")
