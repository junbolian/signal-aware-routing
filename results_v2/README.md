# results_v2: SUMO sample of record

Every SUMO number in the paper is read from the per-run files in this directory by `aggregate_v2.py`, which writes `summary_v2.json` and `tables_v2.txt`. All runs used one build, Eclipse SUMO 1.27.1 (`BUILD.txt`), on one machine (Windows 11, 24 cores), with CPython 3.11 and the packages of `requirements.txt`.

Before the sample was run, a regression gate reran five planners on network 0, seed 11, demand 1/15 and reproduced the earlier runs of the same cells (tag `pre-v2`) bit for bit, including every per-crossing record (`regression/REGRESSION.txt`).

## Design

- K = 10 networks, j = 0..9: `sr.Net(8, random.Random(1000 + j), C=120, link_lo=TAU0, link_hi=TAU0, offset_mode="random")`.
  Network 0 is the network of every earlier run.
- Background seeds 11, 12, 13, 14; departures `T0S = [0, 30, 60, 90]`; at demand 0 shifted by `12*(seed-11)` s.
- Corner-to-corner trip (7,0) -> (0,7).
- Unit of inference: the network (16 trips per network and cell, t interval with 9 df).

## Commands

```
python run_v2.py --layout shared      --decide 45  --jobs 13
python run_v2.py --layout channelized --decide 45  --jobs 5
python run_v2.py --layout shared      --decide 100 --jobs 2
python run_v2.py --layout channelized --decide 100 --jobs 2
python aggregate_v2.py
```

Each driver calls `queue_kernel.py <dir> --levels <den> --nets 0 1 2 3 4 5 6 7 8 9 --sources LA1 MDP TDOPT_replan [GREEDY]` once the source planners of a demand level are complete on all ten networks, and after MDP-Q `queue_kernel.py <dir> --levels <den> --nets 0 1 2 3 4 5 6 7 8 9 --sources MDPQ --tag op`. All 3,802 cells ran: 3,800 sample cells and two demand-0 check cells, about 75.5 core-hours in total.

## Failed and retried runs, teleports, gridlocks

- No run failed. In channelized_d45, a deliberate restart of the driver stopped five runs before they wrote a result, and the restarted driver reran them. No result file was left partly written.
- Gridlock: none. Every one of the 15,208 trips arrived within the 3600 s horizon (`run_health.no_arrival` = 0).
- Teleports (SUMO `--time-to-teleport 600`): some background vehicle teleported during 450 trips
  (shared_d45 264, channelized_d45 128, shared_d100 29, channelized_d100 29). The ego vehicle itself
  teleported in 2 trips, both channelized_d45 LA1: dem10 network 6 seed 11 departure index 1 (997 s) and
  dem15 network 0 seed 14 departure index 0 (1028 s). Both trips are kept in the sample, as in the earlier runs.
- Routes: every demand-0 trip has 14 links (13 crossings), so the division by 13 in the empty-network validation is exact. Across the
  whole sample, 9 trips took a non-monotone 16-link route (TDOPT-Q 7, TDOPT_replan 2). For that reason the
  channelized dem25 fold-A kernel has 2884 transitions rather than 2880.

## Kernel transitions

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

## Conventions

- Kernel file naming: `kernel_dem{den}_foldA.json` is the kernel used by fold A (even networks) and estimated on fold B (odd networks). Each file records `planning_nets` and `estimation_nets`.
- Demand-0 check: no kernel exists at demand 0, so STATIC-Q and MDP-Q ran kernel-free on network 0, seed 11. STATIC-Q gave [1248, 1219, 1189, 1159] = STATIC and MDP-Q [1017, 1099, 1121, 1030] = MDP, with identical routes (`summary_v2.json`, key `dem0_check`). These two cells are not part of the tables.
- Empty-network validation: the analytical prediction is evaluated at the nominal departure time. kappa = mean(SUMO - model) / 13 on TDOPT_replan per network fold, and the deviation is 100 (SUMO - P) / P with P = model + 13 kappa of the other fold.
- The LA-1 shortfall is LA-1 minus the other planner on matched trips, in percent of the other planner's mean on those trips.

## Keys of summary_v2.json

All intervals are 95 percent half-widths (mean +- value). `ci_net`: t interval over network means,
`n_nets - 1` df (primary). `ci_netseed`: t interval over (network, seed) means. `ci_trip`: t interval over trips.
Layout is `shared` (grid8.net.xml) or `channelized` (grid8_rt.net.xml); `decide` is the decision distance in m;
`den` is the demand denominator (insertion probability 1/den per corridor entry; 0 = empty network).
Method identifiers are those of the code: STATIC, GREEDY, LA1, LA2, LA3, MDP, TDOPT_open, TDOPT_replan, TDOPTQ (TDOPT-Q),
STATICQ (STATIC-Q), MDPQ (MDP-Q), MDPQOP (MDP-Q-OP), STATICTRAFFIC, STATICTRAFFICRT.

- `meta`: `generated`, `results_dir`, `interval` (definition), `nets`, `seeds`, `K`.
- `means[]`, one per (layout, decide, den, method): `n` trips with an arrival, `n_missing` trips without one,
  `mean` (over trips), `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`, `net_means` {network: mean}.
- `paired[]`, a minus b matched on (network, seed, departure): `layout`, `decide`, `den`, `a`, `b`, `n`, `mean`,
  `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`, `a_faster` (trips with a - b < 0), `ties` (a - b = 0).
  Pairs: every method minus STATIC, LA1, TDOPTQ and TDOPT_replan, plus TDOPTQ-TDOPT_replan, STATICQ-STATIC, MDPQ-MDP,
  MDPQOP-MDP, MDPQOP-MDPQ, LA1-MDP, LA2-LA1, LA3-LA1 where both exist.
- `shortfall[]`, one per cell: `best_method` (lowest mean), `best_mean`, `la1_mean`, `la1_is_best`,
  `la1_shortfall_pct` (LA1 minus best, percent of the best method's mean on the matched trips), `la1_minus_best`
  {`mean`, `ci_net`} (null when LA1 is lowest), `vs` {method: `la1_minus`, `ci_net`, `n`, `shortfall_pct`,
  `other_mean_matched`} for every other method of the cell.
- `crossing_diag[]`, per (layout, decide, den in 25/15/10, rolling planner, move S/L/R): `n` decided crossings,
  `q_ahead_mean` (vehicles ahead on the lanes of the chosen movement at the decision), `err_mean` and `err_sd` of
  realised minus planned stopline passage time (`cross - pred` of the diag records).
- `value_calibration[]`, shared layout, decision 45 m, per (den, method in MDP/MDPQ/MDPQOP/TDOPTQ, move, ahead "0"/">=1"):
  `n` decisions, `mean` of planned remaining time - realised remaining time + kappa x remaining crossings
  (`plan_arr - arr + kappa * r`, r = crossings from this decision on, including it; kappa of the other network fold), `ci_net`, `n_nets`, `ci_netseed`, `n_netseed`, `ci_trip`.
- `traffic_routers[]`: STATICTRAFFIC - STATIC and STATICTRAFFICRT - STATIC with the keys of `paired`, plus
  `route_equals_static` and `route_compared` (trips whose STATICTRAFFICRT route equals the STATIC route of the same
  network, seed and departure; null for STATICTRAFFIC).
- `empty_validation`: `kappa_A` (fit on TDOPT_replan, fold A = even networks), `kappa_B` (odd networks),
  `kappa_pooled`, `n_fit_A`, `n_fit_B`, `n_crossings` (13); `methods` {method: keys of `means` for the deviation
  100 x (SUMO - P) / P with P = analytical time + 13 x kappa of the other fold, plus `mean_fold_A`, `mean_fold_B`,
  `n_missing`}; `trips[]` {method, net, seed, dep, t0, sumo, model} with the raw analytical prediction (no kappa).
- `cross_check`: `n_trips_compared`, `n_differences`, `differences[]` {den, method, seed, dep, v2, depth}.
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
  q_lanes, plan_arr, arr). `meta_...`: per trip departure and arrival time, route, teleport counts, whether the ego vehicle teleported, the kernel used (file and transitions), and wall time.
- `kernel_dem{den}_fold{A|B}.json`: used by the networks of that fold and estimated on the other fold.
  `kernel_dem{den}_op_fold{A|B}.json`: the same from MDP-Q's own crossings.
- `regression/`: gate runs and `REGRESSION.txt`.
