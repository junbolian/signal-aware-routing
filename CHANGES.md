# Queue-aware congestion benchmark: files and what changed

New files
- queue_model.py: queue-aware planners STATIC-Q, MDP-Q, TDOPT-Q; the paper's MDP
  value (Proposition 7) on the SUMO grid; observed-platoon head model; fluid queue
  for just-turned crossings; platoon persistence along corridors as a state
  (e, q); vectorised value iteration (two Gauss-Seidel sweeps, about 0.1 s).
- calibrate_discharge.py: fits cross_k - onset = LS + HS*k on standing queues
  released at through-green onsets, from background traffic only. Result on
  SUMO 1.27.1 Linux, demand 1/15 and 1/10, seeds 11 and 12, 900 s each:
  HS = 2.374 s, LS = -0.500 s, rmse 0.52 s, 17627 crossings, 4193 onsets.
- discharge_calibration.json: the numbers above, read by queue_model.py.
- run_congestion.py: the full sweep on one build, resumable, cross-platform
  (seed subsets as arguments for parallel processes); run_congestion.sh is the
  same sweep for Git Bash or WSL.
- sumo_aggregate_q.py: means, paired differences (per-instance and
  seed-clustered CIs), per-crossing prediction diagnostic, empty-net calibration.

Modified files
- sumo_runner.py: methods MDP, STATIC-Q, MDP-Q, TDOPT-Q; per-lane subscriptions
  feeding the arrival-rate estimator (standing queue at each through-green onset
  over the last five onsets, divided by the through red); observed platoon ahead of
  the ego per lane at each decision point (45 m before the stopline) and on the
  candidate first links at departure; per-crossing prediction records written to
  sumo_results/diag_<tag>.json for every rolling method; decisions of the
  original methods are unchanged (STATIC seed 11 reproduces bit for bit); the
  SUMO binary path gets the .exe suffix on Windows.
- README.md: the new section and table rows.

Consistency checks passed (all in the container)
- queue_model with rates = {} and empty platoons reproduces the paper exactly:
  fluid and platoon waits equal w_m(xi); STATIC-Q value equals lookahead.static_V;
  MDP-Q value equals the Proposition 7 value (mean origin value over the 40 paper
  networks 1040.7 s against the paper's 1041.4 s, the difference being midpoint
  sampling of the phase grid); TDOPT-Q path and arrival equal td_route.
- Every arc map of TDOPT-Q is nondecreasing in time (FIFO) in all states.
- Head-model accuracy in SUMO, seed 11 moderate: realised minus planned delay
  +1.3 s (S), +1.2 s (L), +2.5 s (R) for MDP-Q, standard deviations below 2.5 s;
  the queue-ignorant MDP underpredicts right turns by 20.6 s (sd 22.6) because
  the kerb-lane through queue blocks the right turn on red.

Build note
- The shipped congestion results (sumo_results_prev_build) do not reproduce on
  SUMO 1.27.1 Linux in a fresh environment (seed 11 STATIC: shipped
  1250/1219/1309/1278, here 1308/1225/1309/1279). All congestion numbers in the
  revised paper come from one sweep on one build.
- The uploaded repo copy's mdp_policy.py builds the MDP over all non-U-turn
  movements, not the L1-monotone set of Proposition 7; queue_model.solve_values
  uses the monotone set. Check which version is on GitHub.
