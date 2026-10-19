"""Figures 6 and 7 of the TR-C manuscript from the v2 sample of record.
Usage: python make_figures_v3.py <results_v2 dir> <output dir>
Paired differences are matched on (network, seed, departure); the interval is the
network-clustered 95 percent t interval over the K network means (K-1 df)."""
import glob, json, os, sys, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

RES, OUT = sys.argv[1], sys.argv[2]
plt.rcParams.update({
 "font.family":"serif","mathtext.fontset":"cm","font.size":9,
 "axes.labelsize":9,"legend.fontsize":7.5,"xtick.labelsize":8,
 "ytick.labelsize":8,"axes.linewidth":0.8,"figure.dpi":300,
 "savefig.bbox":"tight","axes.grid":True,"grid.alpha":0.25,
 "grid.linewidth":0.5,"legend.frameon":False})
NAME = {"STATIC":"SP-STATIC","GREEDY":"GREEDY","LA1":"LA-1","LA2":"LA-2","LA3":"LA-3","MDP":"MDP",
        "TDOPT_open":"TD-OPT open-loop","TDOPT_replan":"TD-OPT replan","STATICQ":"SP-STATIC-Q",
        "MDPQ":"MDP-Q","MDPQOP":"MDP-Q-OP","TDOPTQ":"TD-OPT-Q"}
COL = {"GREEDY":"#EE6677","LA1":"#228833","LA2":"#66AA55","LA3":"#B5C93A","MDP":"#AA3377",
       "TDOPT_open":"#999999","TDOPT_replan":"#000000","STATICQ":"#4477AA","MDPQ":"#AA3377",
       "MDPQOP":"#CC88AA","TDOPTQ":"#4477AA"}

def load(d, den, m):
    out = {}
    for f in glob.glob(os.path.join(RES, d, f"dem{den}_{m}_n*_*.json")):
        b = os.path.basename(f)[:-5].split("_")
        out[(int(b[-2][1:]), int(b[-1]))] = json.load(open(f))
    return out

def paired(d, den, a, b="STATIC"):
    A, B = load(d, den, a), load(d, den, b)
    per = {}
    for k in A:
        if k in B:
            per.setdefault(k[0], []).extend(x - y for x, y in zip(A[k], B[k]) if x is not None and y is not None)
    nm = [st.mean(v) for _, v in sorted(per.items())]
    if len(nm) < 2: return None
    m = st.mean(nm); ci = stats.t.ppf(0.975, len(nm) - 1) * st.stdev(nm) / len(nm) ** 0.5
    return m, ci, nm

# Figure 6: moderate demand, both layouts, per-network means and the clustered interval
panels = [("shared_d45", ["GREEDY","LA1","MDP","TDOPT_open","TDOPT_replan","STATICQ","MDPQ","MDPQOP","TDOPTQ"], "(a) shared turning lanes"),
          ("channelized_d45", ["LA1","MDP","TDOPT_replan","STATICQ","MDPQ","MDPQOP","TDOPTQ"], "(b) channelized approaches")]
fig, axes = plt.subplots(1, 2, figsize=(6.4, 2.9), sharex=True, gridspec_kw={"width_ratios":[9, 7]})
check = {}
for ax, (d, meths, title) in zip(axes, panels):
    for i, m in enumerate(meths):
        r = paired(d, 15, m); y = len(meths) - 1 - i
        check[(d, m)] = r[:2]
        ax.scatter(r[2], [y] * len(r[2]), s=7, color=COL[m], alpha=0.45, linewidths=0, zorder=2)
        ax.errorbar(r[0], y, xerr=r[1], fmt="o", ms=4, color=COL[m], ecolor=COL[m], elinewidth=1.1, capsize=2.5, zorder=3,
                    mfc=("white" if m in ("STATICQ","MDPQ","MDPQOP","TDOPTQ") else COL[m]))
    ax.axvline(0, color="k", lw=0.7)
    ax.set_yticks(range(len(meths))); ax.set_yticklabels([NAME[m] for m in meths][::-1])
    ax.set_title(title, fontsize=8.5, loc="left"); ax.grid(axis="y", alpha=0)
axes[0].set_xlabel("paired difference to SP-STATIC (s)"); axes[1].set_xlabel("paired difference to SP-STATIC (s)")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig6_sumo_v2.pdf")); plt.close(fig)

# Figure 7: paired difference to SP-STATIC against demand, shared lanes, two families
dens = [0, 100, 50, 25, 15, 10]; lab = ["empty", "1/100", "1/50", "1/25", "1/15", "1/10"]
fams = [(["LA1","LA2","LA3","TDOPT_replan"], "(a) lookahead depth without a queue model"),
        (["LA1","MDP","MDPQ","TDOPTQ"], "(b) local information and queue models")]
fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.7), sharey=True)
for ax, (meths, title) in zip(axes, fams):
    for j, m in enumerate(meths):
        xs, ys, es = [], [], []
        for k, den in enumerate(dens):
            r = paired("shared_d45", den, m)
            if r is None: continue
            xs.append(k + (j - (len(meths) - 1) / 2) * 0.11); ys.append(r[0]); es.append(r[1])
            check[("shared_d45", m, den)] = r[:2]
        ax.errorbar(xs, ys, yerr=es, marker="o", ms=3.5, lw=1.1, capsize=2.2, color=COL[m], label=NAME[m],
                    mfc=("white" if m in ("MDPQ","TDOPTQ") else COL[m]))
    ax.axhline(0, color="k", lw=0.7); ax.set_xticks(range(6)); ax.set_xticklabels(lab)
    ax.set_title(title, fontsize=8.5, loc="left"); ax.legend(loc="upper left", ncol=2, handlelength=1.6)
    ax.set_xlabel("background demand")
axes[0].set_ylabel("paired difference to SP-STATIC (s)")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig7_depth_v2.pdf")); plt.close(fig)

# cross-check against summary_v2.json
S = json.load(open(os.path.join(RES, "summary_v2.json")))
idx = {(p["layout"], p["decide"], p["den"], p["a"], p["b"]): p for p in S["paired"]}
bad = 0
for k, (m, ci) in check.items():
    if len(k) == 2: d, a = k; den = 15
    else: d, a, den = k
    lay = d.split("_")[0]; p = idx[(lay, 45, den, a, "STATIC")]
    if abs(p["mean"] - m) > 1e-6 or abs(p["ci_net"] - ci) > 1e-6: bad += 1; print("MISMATCH", k, m, ci, p["mean"], p["ci_net"])
print(f"figures written; {len(check)} plotted intervals checked against summary_v2.json, {bad} mismatches")
