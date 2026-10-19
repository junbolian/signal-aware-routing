"""Print the rows of Tables C.1 and C.2 of the manuscript from the result files.
Table C.3 (empty-network validation) comes from the v2 sample of record:
  python analysis/build_tables_v2.py results_v2
Run from the repository root after the corrected p4_diagnostic.py and p4_hazard.py:
  python make_appendix_tables.py"""
import json
C = 120.0
g = json.load(open("gini_family_slopes.json")); u = json.load(open("uniform_slope_pernet.json"))
print("% Table C.1")
for fam, lab in (("uniform", "Uniform"), ("triangular", "Triangular"), ("lognormal", "Lognormal")):
    s, ci = g[fam]["slope"], g[fam]["ci"]; z = (s + 7.0) / (ci / 1.96)
    print(f"{lab} & $\\Delta$ & $-7.00$ & ${s:.1f} \\pm {ci:.1f}$ & $80$ & ${z:+.1f}$ \\\\")
s, ci = u["uniform_slope_h"], u["ci"]; z = (s + 14 / 3) / (ci / 1.96)
print(f"Uniform, original sweep & $h$ & $-4.67$ & ${s:.1f} \\pm {ci:.1f}$ & ${u['n_networks']}$ & ${z:+.1f}$ \\\\")
p = json.load(open("p4_diag.json")); h = json.load(open("p4_hazard.json"))
print("% Table C.2", p["total"], p["zero_mm"], p["violations"])
for sg in ("5", "10", "20", "30", "45", "60"):
    d = p["per_sigma"][sg]; hz = h[sg]["hazard"]
    md = f"${d['zero_mm_maxdev']:.1f}$" if d["zero_mm"] else "--"
    print(f"${sg}$ & ${d['zero_mm']}$ & {md} & ${d['tight_median']:.2f}$ & ${d['mmrate']:.3f}$ & ${hz:.3f}$ & ${2*int(sg)/C:.3f}$ \\\\")
