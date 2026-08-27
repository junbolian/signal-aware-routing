# Signal-Aware Routing

Companion code for the paper:

> **Routing Through Traffic Signals: The Value, Depth, and Accuracy of Phase Information**
> Junbo Jacob Lian, Zheng Chen, Xing Liu, and Zeshui Xu, working paper, 2026.

Navigation systems route with traffic signals summarized as static average turn
penalties; real-time phase information is used only for display and speed advisory.
This repository contains the full simulation and analysis suite behind the paper,
which treats route choice itself as a function of signal-phase information and
quantifies what each layer of information is worth.

## Key findings

1. **Speed separates from routing.** Cruising at the speed cap and waiting at
   stoplines is time-optimal on every route, so speed advisory (GLOSA) never
   changes arrival time.
2. **Take-the-green is a trap.** The myopic rule has worst-case ratio
   Theta(C/tau_min) and beats static routing only when link times are nearly
   homogeneous, its advantage decaying at a policy-free rate per unit of
   link-time dispersion.
3. **Timing accuracy has a linear price.** Offsets known to within sigma degrade
   the plan linearly; benefits break even with static routing near a quarter
   cycle, so production countdown accuracy sits deep inside the useful region.
4. **Depth has geometric returns, capped by accuracy.** Rolling lookahead over k
   true crossings closes the optimality gap by a factor of about 0.6 per
   crossing, with a floor set by countdown error.
5. **Under unknown offsets the optimum is a threshold rule.** Link-stationary,
   often a single number per link. Its value shows the light directly ahead is
   worth only about a third of the full-information gap.
6. **Congestion is the boundary.** In SUMO with queue-ignorant planners, deep
   timing plans lose their entire advantage while the shallow one-step rule
   still beats static routing from light to heavy demand.

## Repository layout

```
signal_routing.py      Core model: grid network, four-phase signals, RTOR,
                       movement-expanded time-dependent Dijkstra (TD-OPT),
                       SP-FF / SP-STATIC / GREEDY baselines, scenario suite.
lookahead.py           Rolling LA-k policies and the static value function.
extend_lookahead.py    LA-k for k = 1..5 under countdown error sigma = 0/5/10 s.
lak_monotonicity.py    Pointwise monotonicity audit of rolling LA-k.
sweep_hetero.py        Greedy-vs-static advantage across link heterogeneity.
gini_verify.py         Monte Carlo check of the general ladder law (Prop 3).
gini_grid.py           Full-grid Delta-collapse behind Figure 3(a).
pernet_slopes.py       Per-network slope statistics for the dispersion law.
ladder_verify.py       Monte Carlo check of the closed-form ladder model (Prop 3).
p4_diagnostic.py       Instance-level check of the offset-error bound (Prop 4).
check_uniform.py       Uniform-arrival-phase sanity check (Lemma 3).
mdp_policy.py          Unknown-offset MDP: value iteration, threshold policy,
                       simulation validation (Prop 6).
sumo_runner.py         SUMO microscopic experiments (empty and congested).
sumo_aggregate.py      Paired statistics and tables from SUMO outputs.
make_figures.py        Regenerates all paper figures into figures/.
raw_results.csv        Per-instance results of the grid study.
summary.csv            Aggregated grid-study results.
sumo_results/          Raw SUMO run outputs (json).
figures/               Paper figures (regenerable).
```

## Installation

Python 3.10 or later.

```bash
pip install -r requirements.txt
```

The core model and all analytical experiments use the standard library only;
`matplotlib` is needed for figures. The SUMO experiments additionally require:

```bash
pip install eclipse-sumo sumolib traci
```

## Reproducing the paper

| Paper artifact | Command |
|---|---|
| Tables 1-2, `raw_results.csv` | `python signal_routing.py` |
| Table 3 (LA-k x sigma) | `python extend_lookahead.py` |
| Prop 3 closed form vs Monte Carlo | `python ladder_verify.py`, `python gini_verify.py` |
| Prop 3 grid slopes, Figure 3(a) | `python gini_grid.py`, `python pernet_slopes.py` |
| Prop 4 offset-error bound | `python p4_diagnostic.py` |
| Prop 6 MDP value and policy | `python mdp_policy.py` |
| Lemma 3 sanity check | `python check_uniform.py` |
| SUMO validation and congestion | `python sumo_runner.py empty`, `python sumo_runner.py mod METHOD SEED`, `python sumo_runner.py dem METHOD SEED DENOM`, then `python sumo_aggregate.py` |
| STATIC-TRAFFIC baseline | `python sumo_runner.py mod STATIC-TRAFFIC SEED`, `python sumo_runner.py dem STATIC-TRAFFIC SEED DENOM` |
| STATIC-TRAFFIC-RT baseline | `python sumo_runner.py mod STATIC-TRAFFIC-RT SEED`, `python sumo_runner.py dem STATIC-TRAFFIC-RT SEED DENOM` |
| All figures | `python make_figures.py` |

`grid8.net.xml` is tracked. It was built with:

```bash
netgenerate --grid --grid.number 8 --grid.length 500 --grid.attach-length 500 --default.lanenumber 2 --default.speed 8.33 --default-junction-type traffic_light --no-turnarounds -o grid8.net.xml
```

Shipped congested results were generated with Eclipse SUMO 1.27.1 on Linux; multi-vehicle runs are not bit-identical across platforms/builds (the empty-net calibration is); rerun all methods on one build for paired comparisons.

Model defaults: 8x8 grid, cycle C = 120 s, symmetric splits (40 s through, 20 s
protected left), through-before-left, right turn on red enabled, crossing time
2 s, link times U[40, 90] s, i.i.d. uniform offsets, corner-to-corner trips,
paired evaluation across 60 networks x 8 departures with 95% confidence
intervals. All scenario ablations (RTOR off, reversed phase order, synchronized
and progressive offsets, grid sizes 5 and 12) are switches inside
`signal_routing.py`.

## Results snapshot

Gap to the full-information optimum (TD-OPT), base grid, paired instances:

| Method | Gap |
|---|---|
| GREEDY (take the green) | 22.1% |
| SP-FF (ignore signals) | 17.2% |
| SP-STATIC (expected delays) | 16.2% |
| LA-1 (one crossing of truth) | 9.7% |
| MDP optimum, unknown offsets | 10.4% |
| LA-3 | 3.6% |
| TD-OPT | 0 |

![Information ladder](figures/fig1_ladder.png)

## License

MIT. See `LICENSE`.

## Citation

```bibtex
@unpublished{lian2026signals,
  author = {Lian, Junbo Jacob and Chen, Zheng and Liu, Xing and Zhang, Chaoyu and Xu, Zeshui},
  title  = {Routing Through Traffic Signals: The Value, Depth, and Accuracy of Phase Information},
  note   = {Working paper},
  year   = {2026}
}
```
