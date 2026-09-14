"""Calibrate the vertical-queue discharge law used by queue_model.py from
background traffic in SUMO, independently of any ego vehicle.

At every through-green onset of every approach lane, the vehicles standing on
the lane at the onset are recorded in stopline order; their crossing times give
the release law  cross_k - onset = LS + HS * k  (k = 1, 2, ...), fitted by
pooled least squares over all onsets with at least two standing vehicles.
Usage:
  python3 calibrate_discharge.py [denoms] [seeds] [horizon_s]
defaults: denoms 15, seeds 11 12, horizon 900 s. Writes discharge_calibration.json
(LS, HS, fit diagnostics, per-demand breakdown)."""
import os, sys, json
import traci
import sumo_runner as R
import signal_routing as sr


def collect(prob, seed, horizon):
    rfile = "calib.rou.xml"
    R.routes_file(rfile, prob)
    traci.start([R.SUMO_BIN, "-n", R.NETF, "-r", rfile, "--seed", str(seed),
                 "--step-length", "1", "--no-warnings", "--no-step-log",
                 "--time-to-teleport", "600"])
    pairs = []          # (k, cross - onset)
    per_green = []      # vehicles released per green among those standing at onset
    try:
        T = R.tls_tables(); cache = {}
        lanes = {}
        for e in R.net.getEdges():
            lk = R.e2l(e.getID())
            if lk is None:
                continue
            u, v = lk
            h = (v[0] - u[0], v[1] - u[1])
            through_idx = 0 if sr.axis_of(h) == "NS" else 2
            for l in e.getLanes():
                lanes[l.getID()] = (e.getID(), v, through_idx, l.getLength())
        pending = {}    # lane -> (onset, [vids in stopline order], next k)
        prev_idx = {}
        t = 0.0
        while t < horizon:
            R.drive(T, cache, t)
            traci.simulationStep(); t = traci.simulation.getTime()
            for lid, (eid, v, tidx, L) in lanes.items():
                idx = R.pidx((t - R.ANET.offset[v]) % R.C)
                onset = (prev_idx.get(lid) != tidx and idx == tidx)
                prev_idx[lid] = idx
                if lid in pending:
                    onset_t, order, k, gone = pending[lid]
                    present = set(traci.lane.getLastStepVehicleIDs(lid))
                    for vid in order:
                        if vid in gone:
                            continue
                        if vid not in present:
                            gone.add(vid)
                            k += 1
                            pairs.append((k, t - onset_t, prob, vid))
                    if len(gone) == len(order) or idx != tidx:
                        per_green.append((len(order), len(gone), prob))
                        del pending[lid]
                    else:
                        pending[lid] = (onset_t, order, k, gone)
                if onset:
                    vids = traci.lane.getLastStepVehicleIDs(lid)
                    standing = [(traci.vehicle.getLanePosition(x), x) for x in vids
                                if traci.vehicle.getSpeed(x) < 0.1]
                    standing.sort(reverse=True)
                    order = [x for _, x in standing]
                    if len(order) >= 2:
                        pending[lid] = (t, order, 0, set())
    finally:
        traci.close()
    return pairs, per_green


def ols(pairs):
    n = len(pairs)
    sx = sum(k for k, _, _, _ in pairs); sy = sum(y for _, y, _, _ in pairs)
    sxx = sum(k * k for k, _, _, _ in pairs); sxy = sum(k * y for k, y, _, _ in pairs)
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    a = (sy - b * sx) / n
    res = [y - (a + b * k) for k, y, _, _ in pairs]
    rmse = (sum(r * r for r in res) / n) ** 0.5
    return a, b, rmse


if __name__ == "__main__":
    denoms = [int(x) for x in (sys.argv[1].split(",") if len(sys.argv) > 1 else ["15"])]
    seeds = [int(x) for x in (sys.argv[2].split(",") if len(sys.argv) > 2 else ["11", "12"])]
    horizon = float(sys.argv[3]) if len(sys.argv) > 3 else 900.0
    allp, allg = [], []
    for den in denoms:
        for sd in seeds:
            p, g = collect(1.0 / den, sd, horizon)
            allp += p; allg += g
            print(f"denom {den} seed {sd}: {len(p)} crossings from {len(g)} onsets", flush=True)
    # The k = 1 crossing is detected when the front of the first standing vehicle,
    # already at the stopline, enters the junction, about one step after the
    # onset; it carries no headway information, so the law is fitted on k >= 2
    # and the all-k fit is reported for reference.
    a, b, rmse = ols([p for p in allp if p[0] >= 2])
    a_all, b_all, rmse_all = ols(allp)
    out = {"LS": round(a, 3), "HS": round(b, 3), "rmse": round(rmse, 3), "fit_on": "k>=2",
           "all_k_fit": {"LS": round(a_all, 3), "HS": round(b_all, 3), "rmse": round(rmse_all, 3)},
           "n_crossings": len(allp),
           "n_onsets": len(allg), "by_k": {}, "by_demand": {}}
    for k in range(1, 26):
        ys = [y for kk, y, _, _ in allp if kk == k]
        if ys:
            out["by_k"][k] = {"n": len(ys), "mean": round(sum(ys) / len(ys), 2)}
    for den in denoms:
        sub = [p for p in allp if abs(p[2] - 1.0 / den) < 1e-12 and p[0] >= 2]
        if len(sub) > 10:
            aa, bb, rr = ols(sub)
            out["by_demand"][den] = {"LS": round(aa, 3), "HS": round(bb, 3), "rmse": round(rr, 3), "n": len(sub)}
    # release capacity check: standing vehicles released within one green
    full = [(q, r) for q, r, _ in allg]
    out["release"] = {"mean_standing_at_onset": round(sum(q for q, _ in full) / len(full), 2),
                      "frac_fully_released": round(sum(1 for q, r in full if r == q) / len(full), 3),
                      "max_released_in_one_green": max(r for _, r in full)}
    outpath = os.path.join(R.RESDIR, "discharge_calibration.json") if os.environ.get("SR_RESULTS") else "discharge_calibration.json"
    json.dump(out, open(outpath, "w"), indent=1)
    print("wrote", outpath)
    print(json.dumps(out, indent=1))
