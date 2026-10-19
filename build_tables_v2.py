"""LaTeX rows of Tables 4 to 7 and C.3, and the per-network sign counts quoted in
Section 4.2, from results_v2/summary_v2.json and the per-run files.
Usage: python build_tables_v2.py <results_v2 dir>"""
import glob, json, os, sys, statistics as st
RES = sys.argv[1]
S = json.load(open(os.path.join(RES, "summary_v2.json")))
M = {(m["layout"], m["decide"], m["den"], m["method"]): m for m in S["means"]}
P = {(p["layout"], p["decide"], p["den"], p["a"], p["b"]): p for p in S["paired"]}
N = {"STATIC":"SP-STATIC","GREEDY":"GREEDY","LA1":"LA-$1$","LA2":"LA-$2$","LA3":"LA-$3$","MDP":"MDP",
     "TDOPT_open":"TD-OPT open-loop","TDOPT_replan":"TD-OPT replan","STATICQ":"SP-STATIC-Q","MDPQ":"MDP-Q",
     "MDPQOP":"MDP-Q-OP","TDOPTQ":"TD-OPT-Q"}
DENS = [0, 100, 50, 25, 15, 10]
def cell(x): return "--" if x is None else f"${x['mean']:.1f} \\pm {x['ci_net']:.1f}$"
def pcell(x): return "--" if x is None else f"${x['mean']:+.1f} \\pm {x['ci_net']:.1f}$"
def pair(lay, dec, den, a, b):
    x = P.get((lay, dec, den, a, b))
    if x is None:
        y = P.get((lay, dec, den, b, a))
        if y is not None: x = dict(y, mean=-y["mean"])
    return x
print("% Table 4 (means, shared, 45 m)")
for m in ["STATIC","GREEDY","LA1","LA2","LA3","MDP","TDOPT_open","TDOPT_replan","STATICQ","MDPQ","MDPQOP","TDOPTQ"]:
    print(N[m] + " & " + " & ".join(cell(M.get(("shared",45,d,m))) for d in DENS) + " \\\\")
ROWS = [("LA1","STATIC"),("TDOPT_replan","STATIC"),("TDOPTQ","STATIC"),("LA2","LA1"),("LA3","LA1"),("TDOPT_replan","LA1"),
        ("TDOPTQ","LA1"),("TDOPTQ","TDOPT_replan"),("MDP","LA1"),("MDPQ","MDP"),("STATICQ","STATIC")]
print("% Table 5 (paired, shared, 45 m)")
for a, b in ROWS:
    print(f"{N[a]} $-$ {N[b]} & " + " & ".join(pcell(pair("shared",45,d,a,b)) for d in DENS) + " \\\\")
COLS = [("channelized",45,25),("channelized",45,15),("channelized",45,10),("shared",100,15),("channelized",100,15)]
print("% Table 6a (means)")
for m in ["STATIC","LA1","MDP","TDOPT_replan","STATICQ","MDPQ","TDOPTQ"]:
    print(N[m] + " & " + " & ".join(cell(M.get((l,dc,d,m))) for l,dc,d in COLS) + " \\\\")
print("% Table 6b (paired)")
for a, b in [("LA1","STATIC"),("TDOPTQ","STATIC"),("TDOPT_replan","LA1"),("TDOPTQ","LA1"),("TDOPTQ","TDOPT_replan"),("MDPQ","MDP"),("STATICQ","STATIC")]:
    print(f"{N[a]} $-$ {N[b]} & " + " & ".join(pcell(pair(l,dc,d,a,b)) for l,dc,d in COLS) + " \\\\")
D = {(c["layout"], c["decide"], c["den"], c["method"], c["move"]): c for c in S["crossing_diag"]}
print("% Table 7 (crossing errors at 1/15)")
for m in ["GREEDY","LA1","MDP","TDOPT_replan","MDPQ","TDOPTQ"]:
    out = []
    for lay, dec, mv in [("shared",45,"R"),("shared",45,"L"),("channelized",45,"R"),("channelized",45,"L"),("channelized",100,"L")]:
        c = D.get((lay, dec, 15, m, mv))
        out.append("--" if c is None else f"${c['err_mean']:+.1f}$ (${c['err_sd']:.1f}$)")
    print(N[m] + " & " + " & ".join(out) + " \\\\")
print("% Table C.3 (empty validation)")
E = S["empty_validation"]
print("kappa", round(E["kappa_A"],2), round(E["kappa_B"],2))
for m in ["TDOPT_replan","TDOPTQ","LA3","LA2","MDP","STATIC","GREEDY","LA1","TDOPT_open"]:
    v = E["methods"][m]; print(f"{N[m]} & ${v['mean']:+.1f} \\pm {v['ci_net']:.1f}$ \\\\")
print("% shortfall of LA-1")
for s in S["shortfall"]:
    print(s["layout"], s["decide"], s["den"], s["best_method"], round(s["la1_shortfall_pct"],1) if s["la1_shortfall_pct"] is not None else None,
          s["la1_minus_best"])
# per-network sign counts
def netdiff(d, den, a, b):
    def load(m):
        o = {}
        for f in glob.glob(os.path.join(RES, d, f"dem{den}_{m}_n*_*.json")):
            bb = os.path.basename(f)[:-5].split("_"); o[(int(bb[-2][1:]), int(bb[-1]))] = json.load(open(f))
        return o
    A, B = load(a), load(b); r = {}
    for k in A:
        if k in B: r.setdefault(k[0], []).extend(x - y for x, y in zip(A[k], B[k]))
    return [st.mean(v) for _, v in sorted(r.items())]
print("% per-network counts (networks where a is faster than b, of 10)")
for d, den, a, b in [("shared_d45",d_,"LA1","STATIC") for d_ in DENS] + [("shared_d45",d_,"TDOPTQ","STATIC") for d_ in DENS] + \
        [("shared_d45",d_,"TDOPTQ","LA1") for d_ in DENS] + [("shared_d45",d_,"LA2","LA1") for d_ in DENS] + \
        [("channelized_d45",d_,"LA1","STATIC") for d_ in (25,15,10)] + [("channelized_d45",d_,"TDOPTQ","LA1") for d_ in (25,15,10)]:
    v = netdiff(d, den, a, b); print(d, den, a, b, sum(1 for x in v if x < 0), "of", len(v), "range", round(min(v),1), round(max(v),1))
for t in S["traffic_routers"]:
    print("traffic", t["den"], t["a"], round(t["mean"],1), round(t["ci_net"],1), t.get("route_equals_static"), t.get("route_compared"))
