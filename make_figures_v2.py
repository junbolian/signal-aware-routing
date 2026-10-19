"""Figures of the v2 sample of record (Task 6), from results_v2/summary_v2.json.

  results_v2/fig5_sumo.pdf   mean travel time per method at demand 1/15, decision 45 m,
                             (a) shared turning lanes, (b) channelized approaches;
                             network-clustered 95 percent intervals; queue-aware planners hatched
  results_v2/fig6_depth.pdf  paired difference to STATIC against demand, network-clustered intervals

Style follows make_figures.py and the submitted fig5_sumo.pdf: matplotlib serif
(DejaVu Serif, Computer Modern mathtext), 9 pt, the same palette and hatching.
Usage: python make_figures_v2.py [--res results_v2]"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "mathtext.fontset": "cm", "font.size": 9,
    "axes.labelsize": 9, "legend.fontsize": 7.5, "xtick.labelsize": 8,
    "ytick.labelsize": 8, "axes.linewidth": 0.8, "figure.dpi": 300,
    "savefig.bbox": "tight", "axes.grid": True, "grid.alpha": 0.25,
    "grid.linewidth": 0.5, "legend.frameon": False})
COL = {"ff": "#BBBBBB", "st": "#4477AA", "gr": "#EE6677", "la": "#228833",
       "opt": "#000000", "mdp": "#AA3377", "la2": "#66AA55", "la3": "#BBCC33"}
LABEL = {"STATIC": "SP-STATIC", "GREEDY": "GREEDY", "LA1": "LA-1", "LA2": "LA-2", "LA3": "LA-3", "MDP": "MDP",
         "TDOPT_open": "TD-OPT open", "TDOPT_replan": "TD-OPT replan", "STATICQ": "SP-STATIC-Q",
         "MDPQ": "MDP-Q", "MDPQOP": "MDP-Q-OP", "TDOPTQ": "TD-OPT-Q"}
# (method, color key, hatch): colors by information regime, queue-aware planners hatched
BARS = [("STATIC", "st", None), ("GREEDY", "gr", None), ("LA1", "la", None), ("MDP", "mdp", None),
        ("TDOPT_open", "ff", None), ("TDOPT_replan", "opt", None), ("STATICQ", "st", "///"),
        ("MDPQ", "mdp", "///"), ("MDPQOP", "mdp", "xxx"), ("TDOPTQ", "opt", "///")]
LINES = [("LA1", "la"), ("LA2", "la2"), ("LA3", "la3"), ("MDP", "mdp"), ("TDOPT_replan", "opt"), ("TDOPTQ", "st")]
LEVELS = [(0, "empty"), (100, "1/100"), (50, "1/50"), (25, "1/25"), (15, "1/15"), (10, "1/10")]

ap = argparse.ArgumentParser()
ap.add_argument("--res", default="results_v2")
RES = ap.parse_args().res
S = json.load(open(os.path.join(RES, "summary_v2.json")))
MEANS = {(r["layout"], r["decide"], r["den"], r["method"]): r for r in S["means"]}
PAIRED = {(r["layout"], r["decide"], r["den"], r["a"], r["b"]): r for r in S["paired"]}

# ---------------------------------------------------------------- fig5_sumo
fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True,
                         gridspec_kw={"width_ratios": [10, 8], "wspace": 0.06})
lo, hi = 1e9, 0
for ax, lay, tag in zip(axes, ("shared", "channelized"), ("(a) shared turning lanes", "(b) channelized approaches")):
    bars = [b for b in BARS if (lay, 45, 15, b[0]) in MEANS]
    for i, (m, c, h) in enumerate(bars):
        r = MEANS[(lay, 45, 15, m)]
        ax.bar(i, r["mean"], yerr=r["ci_net"], capsize=3, width=.62, color=COL[c],
               hatch=h, edgecolor="white" if h else COL[c], linewidth=0 if not h else 0.0)
        lo, hi = min(lo, r["mean"] - r["ci_net"]), max(hi, r["mean"] + r["ci_net"])
    ax.set_xticks(range(len(bars)))
    ax.set_xticklabels([LABEL[b[0]] for b in bars], rotation=35, ha="right", fontsize=7.5)
    ax.set_title(tag, loc="left", fontsize=9)
    ax.grid(axis="x", visible=False)
axes[0].set_ylabel("Mean travel time (s)")
step = 50
axes[0].set_ylim(step * int((lo - 20) // step), step * int((hi + 20) // step + 1))
fig.savefig(os.path.join(RES, "fig5_sumo.pdf"))
plt.close(fig)

# ---------------------------------------------------------------- fig6_depth
fig, ax = plt.subplots(figsize=(4.6, 3.0))
xs = list(range(len(LEVELS)))
dodge = 0.06
for k, (m, c) in enumerate(LINES):
    pts = [(x, PAIRED.get(("shared", 45, den, m, "STATIC"))) for x, (den, _) in zip(xs, LEVELS)]
    pts = [(x + (k - (len(LINES) - 1) / 2) * dodge, r) for x, r in pts if r]
    if not pts:
        continue
    ax.errorbar([p[0] for p in pts], [p[1]["mean"] for p in pts], yerr=[p[1]["ci_net"] for p in pts],
                color=COL[c], marker="o", ms=3, lw=1.5, capsize=2, label=LABEL[m])
ax.axhline(0, color="black", lw=0.7)
ax.set_xticks(xs)
ax.set_xticklabels([lab for _, lab in LEVELS])
ax.set_xlabel("background demand (insertion probability)")
ax.set_ylabel("paired difference to SP-STATIC (s)")
ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))
fig.savefig(os.path.join(RES, "fig6_depth.pdf"))
plt.close(fig)
print(f"wrote {RES}/fig5_sumo.pdf and {RES}/fig6_depth.pdf")
