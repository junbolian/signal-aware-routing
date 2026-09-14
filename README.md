# Signal-Aware Routing

Companion code for the paper:

> **Routing Through Traffic Signals: The Value, Depth, and Accuracy of Phase Information**
> Author names withheld for double-anonymous review, working paper, 2026.

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
queue_model.py         Queue-aware planners (STATIC-Q, MDP-Q, TDOPT-Q) and the MDP in SUMO.
queue_kernel.py        Empirical transition kernel of the count ahead (cross-fitted folds).
make_channelized_net.py Channelized four-lane grid (grid8_rt.net.xml) for the lane-layout ablation.
calibrate_discharge.py Discharge-law calibration from background traffic.
run_congestion.sh      Full congestion sweep on one build (resumable).
sumo_aggregate.py      Paired statistics and tables from SUMO outputs (original).
sumo_aggregate_q.py    Tables and diagnostics of the queue-aware benchmark.
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
| Queue-aware congestion benchmark | `./run_congestion.sh`, then `python sumo_aggregate_q.py` |
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
@unpublished{signals2026routing,
  author = {Author names withheld for review},
  title  = {Routing Through Traffic Signals: The Value, Depth, and Accuracy of Phase Information},
  note   = {Working paper},
  year   = {2026}
}
```

## Queue-aware congestion benchmark (added Sep 2026)

Files: `queue_model.py` (queue-aware planners STATIC-Q, MDP-Q, TDOPT-Q and the
paper's MDP executed in SUMO), `calibrate_discharge.py` (fits the release law
of standing queues, writes `discharge_calibration.json`: HS = 2.37 s,
LS = -0.5 s), `run_congestion.sh` (the full sweep, resumable),
`sumo_aggregate_q.py` (tables, paired differences, prediction diagnostic).

```bash
pip install eclipse-sumo==1.27.1 sumolib==1.27.1 traci==1.27.1 numpy
python3 calibrate_discharge.py 15,10 11,12 900     # optional: re-fit HS and LS
./run_congestion.sh                                 # ~4 h on one core, resumable
python3 queue_kernel.py                             # transition kernels from the queue-ignorant diagnostics (cross-fitted folds)
python3 run_congestion.py                           # fills the MDP-Q and STATIC-Q cells, which need the kernels
python3 sumo_aggregate_q.py                         # prints tables, writes sumo_results/congestion_summary.json
```
The driver runs the queue-ignorant methods first; MDP-Q and STATIC-Q require
`kernel_<prefix>_fold{A,B}.json`, so run `queue_kernel.py` once the diagnostics
of LA1, MDP, TDOPT_replan and GREEDY exist for both seed folds, then the driver again.

Multi-vehicle SUMO runs are not reproducible across builds, so every method
of the congestion tables must come from the same sweep; `sumo_results/` holds
one build's results only. The per-crossing prediction records in
`sumo_results/diag_*.json` compare each rolling planner's planned stopline
passage with the realised one.

### Lane-layout ablation (channelized approaches)

`python3 make_channelized_net.py` builds `grid8_rt.net.xml`: four lanes per edge,
kerb lane right-turn only, two through lanes, median lane left-turn only, so
turning vehicles never queue behind through vehicles. Every script reads the
network from `SR_NET` and writes to `SR_RESULTS` (defaults `grid8.net.xml`,
`sumo_results`), so the ablation runs with

```bash
export SR_NET=grid8_rt.net.xml SR_RESULTS=sumo_results_rt      # PowerShell: $env:SR_NET="grid8_rt.net.xml"; $env:SR_RESULTS="sumo_results_rt"
python3 calibrate_discharge.py 15,10 11,12 900
python3 run_congestion.py --moderate-only --core-only
python3 queue_kernel.py sumo_results_rt
python3 run_congestion.py --moderate-only --core-only
python3 sumo_aggregate_q.py sumo_results_rt
```
