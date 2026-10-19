# v2 sample of record: SUMO congestion results across ten networks

Everything in this directory comes from one SUMO build on one machine. Every number in
`summary_v2.json` and `tables_v2.txt` is read from the per-run files here by `aggregate_v2.py`.

## Environment

- Machine: Intel Core Ultra 9 275HX, 24 cores (8 P + 16 E), Windows 11 (10.0.26300).
- Simulator: Eclipse SUMO 1.27.1. `sumo --version` matches `BUILD.txt` (copied from
  `sumo_results/BUILD.txt`) line for line, build-features line included.
- Python: no interpreter on the machine still had `traci`/`sumolib`/`eclipse-sumo`
  (the `__pycache__` shows the earlier runs used CPython 3.11, whose site-packages is now empty).
  A fresh venv `E:\Optimization\sr-venv311` was created from the system CPython 3.11 with
  `eclipse-sumo==1.27.1 traci==1.27.1 sumolib==1.27.1 numpy matplotlib scipy`
  (resolved: numpy 2.4.6, matplotlib 3.11.2, scipy 1.17.1, sumo-data 1.28.0, a data-only dependency).
  The regression gate (below) reproduces the existing runs bit for bit on this environment.

## Design as run

- K = 10 networks, j = 0..9: `sr.Net(8, random.Random(1000 + j), C=120, link_lo=TAU0, link_hi=TAU0, offset_mode="random")`.
  Network 0 is the network of every earlier run.
- Background seeds 11, 12, 13, 14; departures `T0S = [0, 30, 60, 90]`; at demand 0 shifted by `12*(seed-11)` s.
- Corner-to-corner trip (7,0) -> (0,7).
- Unit of inference: the network (16 trips per network and cell, t interval with 9 df).

## Commands run (repository root, Git Bash, venv Python)

```
PY=/e/Optimization/sr-venv311/Scripts/python.exe
git checkout -b v2-sample
# Task 0.4 regression gate (results_v2/regression/run_gate.sh): five runs in parallel
SR_NETSEED=0 SR_RESULTS=results_v2/regression SR_PORT=<p> $PY sumo_runner.py dem <METHOD> 11 15
$PY results_v2/regression/compare_gate.py                 # -> regression/REGRESSION.txt
# Task 0.5 timing (results_v2/timing/run_timing.sh), then
$PY results_v2/timing/project.py                          # -> TIMING.txt
# Tasks 1-3: results_v2/launch_all.sh, four drivers concurrently (22 slots)
$PY run_v2.py --layout shared      --decide 45  --jobs 13
$PY run_v2.py --layout channelized --decide 45  --jobs 5
$PY run_v2.py --layout shared      --decide 100 --jobs 2
$PY run_v2.py --layout channelized --decide 100 --jobs 2
#   each driver calls, when the sources of a level are complete for all ten networks,
#   $PY queue_kernel.py <dir> --levels <den> --nets 0 .. 9 --sources LA1 MDP TDOPT_replan [GREEDY]
#   and, after MDP-Q, $PY queue_kernel.py <dir> --levels <den> --nets 0 .. 9 --sources MDPQ --tag op
# Tasks 4-6
$PY aggregate_v2.py                                       # -> summary_v2.json, tables_v2.txt
$PY make_figures_v2.py                                    # -> fig5_sumo.pdf, fig6_depth.pdf
```

`results_v2/relaunch_channelized_d45.sh` restarted channelized_d45 at 23:02 as two processes with
disjoint networks (`--nets 0 1 2 3 4 --jobs 4 --port-base 9100` and `--nets 5 6 7 8 9 --jobs 5 --port-base 9150`)
once the decision-100 drivers had freed their slots. See "Failed and retried runs" below.

## K, cores, wall time

- K = 10 (`TIMING.txt`: projected 7.0 h for K = 10, far below the 72 h threshold).
- 24 cores; 22 concurrent SUMO runs (13 + 5 + 2 + 2 slots, then 13 + 9 after the restart).
- Sum of per-run wall time (`run_health.wall_s_sum`): 42.0 + 23.9 + 4.6 + 5.0 = 75.5 core-hours
  (projection: 76.5).

| Task | Wall clock (2026-10-18/19, local) | Duration |
|---|---|---|
| 0 (branch, code, gate, timing) | gate runs finished 20:35:34; TIMING.txt written 20:38:36; commit e21849a 20:39:13 | about 1 h including setup |
| 1 (shared_d45, 2522 runs incl. Task 3 cells) | 20:39:22 to 00:01:58 | 3 h 23 min |
| 2.1 (channelized_d45, 880 runs incl. Task 3 cells) | 20:39:22 to 00:28:34 | 3 h 49 min |
| 2.2 (shared_d100 / channelized_d100, 200 runs each) | 20:39:22 to 23:01:17 / 23:13:22 | 2 h 22 min / 2 h 34 min |
| 3 (MDPQOP; on-policy kernels as soon as MDP-Q of a level was complete) | shared 23:48:32 to 00:01:58; channelized 00:20:04 to 00:28:34 | overlapped with Tasks 1 and 2.1 |
| 4 to 6 (aggregate_v2.py, make_figures_v2.py) | after 00:28:34 | a few minutes |

All 3,802 cells ran: 3,800 sample cells plus the two demand-0 check cells.

## Failed and retried runs, teleports, gridlocks

- No run failed on its own. The only entries in `channelized_d45/failures_v2.log` and the four `FAILED` lines
  at 23:01:48 in `channelized_d45/driver_log.txt` (n7 dem15: TDOPT_replan 13, TDOPT-Q 13, LA1 14, MDP 14)
  come from the deliberate restart: killing the driver's process tree stopped those runs before they wrote
  a result. The restarted drivers reran them and the other interrupted cell, and every cell finished on its first attempt after that.
  No result file was left partly written (all JSON files modified in the 5 minutes before the kill parse).
- Gridlock: none. Every one of the 15,208 trips arrived within the 3600 s horizon (`run_health.no_arrival` = 0).
- Teleports (SUMO `--time-to-teleport 600`): some background vehicle teleported during 450 trips
  (shared_d45 264, channelized_d45 128, shared_d100 29, channelized_d100 29). The ego vehicle itself
  teleported in 2 trips, both channelized_d45 LA1: dem10 network 6 seed 11 departure index 1 (997 s) and
  dem15 network 0 seed 14 departure index 0 (1028 s). Both trips are kept in the sample, as in the earlier runs.
- Routes: every demand-0 trip has 14 links (13 crossings), so Task 4's division by 13 is exact. Across the
  whole sample, 9 trips took a non-monotone 16-link route (TDOPT-Q 7, TDOPT_replan 2). For that reason the
  channelized dem25 fold-A kernel has 2884 transitions rather than 2880.

## Kernel transitions (Task 1, Task 2.1, Task 3.3)

| Directory | Kernel | Transitions per fold (A / B) | Sources |
|---|---|---|---|
| shared_d45 | dem100, dem50, dem25, dem10 | 2880 / 2880 | LA1, MDP, TDOPT_replan |
| shared_d45 | dem15 | 3840 / 3840 | LA1, MDP, TDOPT_replan, GREEDY |
| shared_d45 | dem25_op, dem15_op, dem10_op | 960 / 960 | MDPQ |
| channelized_d45 | dem15, dem10 | 2880 / 2880 | LA1, MDP, TDOPT_replan |
| channelized_d45 | dem25 | 2884 / 2880 | LA1, MDP, TDOPT_replan |
| channelized_d45 | dem15_op | 960 / 960 | MDPQ |

Each fold's kernel pools the 5 networks x 4 seeds x 4 departures of the other fold. The on-policy kernel has
one source planner where the off-policy kernel has three (four at 1/15), so it rests on a third (a quarter)
of the transitions. MDPQOP minus MDPQ therefore compares on- with off-policy estimation and, at the same time,
a smaller with a larger sample.

## Code changes (branch v2-sample)

- `sumo_runner.py`
  - `SR_NETSEED=j` builds the network with `random.Random(1000 + j)` (`build_anet`). With it set, tags are
    `dem{den}_{METHOD}_n{j}_{seed}` and `diag_dem{den}_{METHOD}_n{j}_{seed}`. Without it, names and behavior are as before.
  - With `SR_NETSEED` set, STATIC-Q, MDP-Q and MDPQOP load `kernel_dem{den}[_op]_fold{A|B}.json` of their network fold.
    At demand 0 no kernel exists and they run kernel-free (only the demand-0 check uses this).
  - New method `MDPQOP` (alias `MDP-Q-OP`): MDP-Q with the `_op` kernel, identical in every other respect.
  - Each run writes `meta_<tag>.json`: per trip departure/arrival time, route, teleport count in the network,
    whether the ego teleported, the kernel used (path, transitions), and wall time.
  - Robustness for parallel runs without changing any simulation input: one route file per process
    (in the system temp directory, same content as `dem{den}.rou.xml`), and a fixed TraCI port per slot (`SR_PORT`).
  - STATIC-Q/MDP-Q kernel values are solved once per process (`q_values`) instead of at every decision.
    It is the same deterministic computation and gives the same values.
- `queue_kernel.py`: network folds (A = even, B = odd), each crossing binned with its own network's offsets, pooling over
  all seeds of every network of the estimating fold; `--levels`, `--nets`, `--sources`, `--tag`. Legacy seed-fold mode unchanged.
- `sumo_aggregate_q.py`: `model_pred(m, t0, ctx)` works on any network (`analytic_context(j)`) and covers LA2, LA3, TDOPTQ.
  The script body moved under `main()`. Output on `sumo_results` is byte-identical to the previous version.
- New: `run_v2.py`, `aggregate_v2.py`, `make_figures_v2.py`.

## Deviations from the work order, and findings

1. **The earlier STATIC-Q and MDP-Q runs never used the empirical kernel.** In every commit, `sumo_runner.py`
   declares `KERNEL=None` and never assigns it, so `qm.solve_values` and `qm.decide` ran their
   kernel-free branch (platoon persistence: q kept on S, reset to 0 after a turn). Check: rerunning legacy
   `dem MDP-Q 11 25` (no SR_NETSEED) reproduces `sumo_results/dem25_MDPQ_11.json` exactly
   ([1244, 1275, 1245, 1210]). The kernel files in `sumo_results*` were estimated but not used.
   In v2, STATIC-Q, MDP-Q and MDPQOP load the cross-fitted kernel as this order specifies. So v2 MDP-Q and STATIC-Q
   are not the planners behind the current Tables 4 to 6.
2. Python environment re-created (see Environment). The regression gate passed bit for bit, including every
   per-crossing diag record (`regression/REGRESSION.txt`).
3. Parallelism: the order suggests several processes with disjoint `--nets`. The driver also accepts `--jobs N`
   (N concurrent runs in one process, each on its own TraCI port). One process per results directory ran,
   except channelized_d45 after 23:02 (two processes, disjoint `--nets`). The cells and their content are the same either way.
4. `run_v2.py` sets `SR_NET` from `--layout` (shared = grid8.net.xml, channelized = grid8_rt.net.xml), so the
   `set SR_NET=...` lines of the order are not needed. A conflicting `SR_NET` is overridden and noted.
5. Kernel file naming: `kernel_dem{den}_foldA.json` is the kernel **used by** fold A (even networks) and
   **estimated on** fold B (odd networks). This follows the convention of the earlier seed-fold files. Each file
   records `planning_nets` and `estimation_nets`. A kernel is built once every source cell of all ten networks
   exists, not only the other fold's. The kernel itself is identical: each file reads only the other fold.
6. Demand-0 check: no kernel exists at demand 0, so the check ran STATIC-Q and MDP-Q kernel-free (with no traffic,
   every platoon is empty). Result on network 0, seed 11: STATIC-Q [1248, 1219, 1189, 1159] = STATIC, and
   MDP-Q [1017, 1099, 1121, 1030] = MDP, with identical routes (`summary_v2.json` `dem0_check`). These two cells are
   excluded from the 5.1/5.2 tables.
7. Robustness additions without changing simulation input (verified by the gate): a per-process route file,
   a fixed TraCI port per slot, `meta_*.json` sidecars, and kernel values solved once per process.
8. Timing (Task 0.5) was measured with seven runs in parallel, not one at a time. The projection applies a x2
   contention allowance for 22 concurrent runs. The realized total (75.5 core-hours) matched the projection.
9. Task 4: the analytical prediction is evaluated at the nominal departure time `t0` (warm-up 0 at demand 0),
   as in `sumo_aggregate_q.py`. kappa = mean(SUMO - model) / 13 on TDOPT_replan per fold; deviation
   = 100 (SUMO - P) / P with P = model + 13 kappa of the other fold. TDOPT_open, TDOPT_replan and TDOPTQ share the
   model prediction `td_route` (no queues at demand 0).
10. Task 5.5 uses the cross-fitted kappa (other network fold) of Task 4. r counts the crossings from the decision
    on, including the decided one.
11. Task 5.3: the shortfall is LA1 minus the other method on matched trips, in percent of the other method's mean
    on those trips. The lowest-mean method is chosen among every method of the cell, the traffic-aware routers included.
12. Figures: `figures/fig3_scaling.pdf` is not in the repository (`figures/` holds PNGs), and `pdffonts` is not
    installed. The fonts of `fig3_scaling.pdf` and `fig5_sumo.pdf` in `ignore/trc_submission.zip` were read from their
    `/BaseFont` entries: DejaVu Serif, plus Computer Modern for mathtext, embedded as Type 3 by Matplotlib. The v2
    figures use the same rcParams (`make_figures.py`) and embed DejaVu Serif as Type 3.
    The figures label methods with the paper's display names (SP-STATIC, LA-1, TD-OPT replan, ...), as the existing
    figures do; `make_figures_v2.py` maps them to the code identifiers. fig5 adds MDP-Q-OP (cross-hatched). The panel
    tags "(a) shared turning lanes" and "(b) channelized approaches" stay as in the submitted figure. fig6 adds MDP
    and puts the legend outside the axes. Points are dodged horizontally so the intervals do not overlap.
13. The timing runs (network 1, seed 11, demand 1/10) are in `timing/` and are not part of the sample. The MDP-Q
    timing run used the legacy network-0 kernel `sumo_results/kernel_dem10_foldA.json`, as the order allows.
14. Nothing was cut: all of Tasks 1 to 3 ran at K = 10.

## Keys of summary_v2.json

All intervals are 95 percent half-widths (mean +- value). `ci_net`: t interval over network means,
`n_nets - 1` df (primary). `ci_netseed`: t interval over (network, seed) means. `ci_trip`: t interval over trips.
Layout is `shared` (grid8.net.xml) or `channelized` (grid8_rt.net.xml); `decide` is the decision distance in m;
`den` is the demand denominator (insertion probability 1/den per corridor entry; 0 = empty network).
Method identifiers are those of the code: STATIC, GREEDY, LA1, LA2, LA3, MDP, TDOPT_open, TDOPT_replan, TDOPTQ (TDOPT-Q),
STATICQ (STATIC-Q), MDPQ (MDP-Q), MDPQOP (MDP-Q-OP), STATICTRAFFIC, STATICTRAFFICRT.

- `meta`: `generated`, `results_dir`, `interval` (definition), `nets`, `seeds`, `K`.
- `means[]` (5.1), one per (layout, decide, den, method): `n` trips with an arrival, `n_missing` trips without one,
  `mean` (over trips), `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`, `net_means` {network: mean}.
- `paired[]` (5.2), a minus b matched on (network, seed, departure): `layout`, `decide`, `den`, `a`, `b`, `n`, `mean`,
  `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`, `a_faster` (trips with a - b < 0), `ties` (a - b = 0).
  Pairs: every method minus STATIC, LA1, TDOPTQ and TDOPT_replan, plus TDOPTQ-TDOPT_replan, STATICQ-STATIC, MDPQ-MDP,
  MDPQOP-MDP, MDPQOP-MDPQ, LA1-MDP, LA2-LA1, LA3-LA1 where both exist.
- `shortfall[]` (5.3), one per cell: `best_method` (lowest mean), `best_mean`, `la1_mean`, `la1_is_best`,
  `la1_shortfall_pct` (LA1 minus best, percent of the best method's mean on the matched trips), `la1_minus_best`
  {`mean`, `ci_net`} (null when LA1 is lowest), `vs` {method: `la1_minus`, `ci_net`, `n`, `shortfall_pct`,
  `other_mean_matched`} for every other method of the cell.
- `crossing_diag[]` (5.4), per (layout, decide, den in 25/15/10, rolling planner, move S/L/R): `n` decided crossings,
  `q_ahead_mean` (vehicles ahead on the lanes of the chosen movement at the decision), `err_mean` and `err_sd` of
  realised minus planned stopline passage time (`cross - pred` of the diag records).
- `value_calibration[]` (5.5), shared layout, decision 45 m, per (den, method in MDP/MDPQ/MDPQOP/TDOPTQ, move, ahead "0"/">=1"):
  `n` decisions, `mean` of planned remaining time - realised remaining time + kappa x remaining crossings
  (`plan_arr - arr + kappa * r`, r = crossings from this decision on, including it; kappa of the other network fold
  from Task 4), `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`.
- `traffic_routers[]` (5.6): STATICTRAFFIC - STATIC and STATICTRAFFICRT - STATIC with the keys of `paired`, plus
  `route_equals_static` and `route_compared` (trips whose STATICTRAFFICRT route equals the STATIC route of the same
  network, seed and departure; null for STATICTRAFFIC).
- `empty_validation` (Task 4, 5.7): `kappa_A` (fit on TDOPT_replan, fold A = even networks), `kappa_B` (odd networks),
  `kappa_pooled`, `n_fit_A`, `n_fit_B`, `n_crossings` (13); `methods` {method: keys of `means` for the deviation
  100 x (SUMO - P) / P with P = analytical time + 13 x kappa of the other fold, plus `mean_fold_A`, `mean_fold_B`,
  `n_missing`}; `trips[]` {method, net, seed, dep, t0, sumo, model} with the raw analytical prediction (no kappa).
- `cross_check` (5.8): `n_trips_compared`, `n_differences`, `differences[]` {den, method, seed, dep, v2, depth}.
- `dem0_check`: {"STATICQ-STATIC", "MDPQ-MDP"}: `a`, `b` (four travel times each, network 0, seed 11),
  `times_identical`, `routes_identical`.
- `run_health[]` per directory: `runs`, `trips`, `no_arrival`, `no_arrival_cells` [den, method, net, seed, dep],
  `trips_with_teleports` (any vehicle teleported in the network during the trip's simulation), `ego_teleported`,
  `wall_s_sum` (sum of per-run wall seconds).
- `kernels[]`: `layout`, `decide`, `file`, `n_transitions`, `estimation_nets`, `planning_nets`, `sources`,
  `n_transitions_by_net`.

## Files

- `<layout>_d<decide>/dem{den}_{METHOD}_n{j}_{seed}.json`: four travel times (s), null if no arrival within 3600 s.
- `diag_...`: per departure, the per-crossing decision records (edge, move, q_ahead, t_decide, t_free, pred, cross,
  q_lanes, plan_arr, arr). `meta_...`: see above.
- `kernel_dem{den}_fold{A|B}.json`: used by the networks of that fold and estimated on the other fold.
  `kernel_dem{den}_op_fold{A|B}.json`: the same from MDP-Q's own crossings.
- `driver_log.txt`, `failures_v2.log` (only if a run failed): driver logs.
- `regression/`: gate runs and `REGRESSION.txt`. `timing/`: timing runs (network 1, not part of the sample).
