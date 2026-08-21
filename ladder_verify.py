"""P2 ladder model: closed form vs Monte Carlo, and the universal-slope check
against the full-grid sweep (fig4).

Block model: two parallel signalized streets, times X,Y iid U[tb-h,tb+h],
independent phases, P(red)=rho, residual|red ~ U(0,r).
STATIC knows X,Y, commits to min(X,Y), pays expected wait rho*r/2.
GREEDY: straight street (X) if green; else Y if green; else waits X residual.
Closed form per block:
  E[STATIC] = tb - h/3 + rho*r/2
  E[GREEDY] = tb + rho^2 * r/2
  Adv(h) = E[STATIC]-E[GREEDY] = rho*(1-rho)*r/2 - h/3
  dAdv/dh = -1/3 per decision block, independent of rho, r  (universal slope)"""
import random, math

tb, r, C = 65.0, 80.0, 120.0
rho = r / C
rng = random.Random(7)
M = 2_000_000
for h in (0.0, 10.0, 25.0):
    s_sum = g_sum = 0.0
    for _ in range(M):
        X = rng.uniform(tb-h, tb+h); Y = rng.uniform(tb-h, tb+h)
        redX = rng.random() < rho; redY = rng.random() < rho
        resX = rng.uniform(0, r) if redX else 0.0
        resY = rng.uniform(0, r) if redY else 0.0
        # static
        if X <= Y: s_sum += X + resX
        else:      s_sum += Y + resY
        # greedy
        if not redX:   g_sum += X
        elif not redY: g_sum += Y
        else:          g_sum += resX + X
    Es, Eg = s_sum/M, g_sum/M
    cf_s = tb - h/3 + rho*r/2
    cf_g = tb + rho*rho*r/2
    print(f"h={h:4.0f}  MC static {Es:7.3f} (cf {cf_s:7.3f})  "
          f"MC greedy {Eg:7.3f} (cf {cf_g:7.3f})  "
          f"MC adv {Es-Eg:+7.3f} (cf {cf_s-cf_g:+7.3f})")
hstar = 3*rho*(1-rho)*r/2
print(f"closed-form crossover h* = {hstar:.2f} s per block")

# grid sweep slope (fig4 data) vs universal prediction -J/3, J = L1 = 14
pts = [(0,59.2),(5,36.5),(10,14.4),(15,-0.9),(20,-27.5),(25,-56.0)]
n=len(pts); mh=sum(p[0] for p in pts)/n; my=sum(p[1] for p in pts)/n
num=sum((p[0]-mh)*(p[1]-my) for p in pts); den=sum((p[0]-mh)**2 for p in pts)
slope=num/den
print(f"grid sweep OLS slope = {slope:.3f} s per s of h; "
      f"universal prediction -J/3 with J=14: {-14/3:.3f}")
