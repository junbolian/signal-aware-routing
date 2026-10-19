"""Resumable driver for the v2 sample of record (results_v2/<layout>_d<decide>).

Usage:
  python run_v2.py --layout shared --decide 45 --nets 0 1 2 3 4 --seeds 11 12 13 14
  python run_v2.py --layout channelized --decide 45 --nets 5 6 7 8 9 --jobs 8

A cell is one (demand level, method, network, seed) run of sumo_runner.py (four
trips); it is skipped when its result file exists. Order: the kernel-free
methods, then, per demand level, the cross-fitted kernels of queue_kernel.py
once the queue-ignorant source cells (LA1, MDP, TDOPT_replan, GREEDY where in
the plan) exist for every network 0..K-1 and every seed, then STATIC-Q and
MDP-Q, then the on-policy kernel from MDP-Q's crossings (--tag op) and MDPQOP.
Several processes with disjoint --nets (or --seeds) lists coordinate through
the result files: a process whose cells wait for a kernel polls until the other
processes have produced the source cells, and kernel estimation is guarded by
a lock directory. --jobs N runs N cells at once inside one process, each slot
on its own TraCI port (SR_PORT), so concurrent SUMO instances never share a port.

Options:
  --layout shared|channelized   grid8.net.xml or grid8_rt.net.xml (sets SR_NET)
  --decide 45|100               decision point in metres (sets SR_DECIDE)
  --nets / --seeds              cells this process owns (default 0..K-1, 11..14)
  --K                           networks in the sample (kernel completeness), default 10
  --jobs                        concurrent cells in this process, default 1
  --port-base                   first TraCI port of this process (default per layout)
  --no-wait                     do not poll for other processes' cells; skip blocked cells
  --plan                        print the cell counts and exit"""
import argparse
import concurrent.futures as cf
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
PY = sys.executable
TAG = {"STATIC-TRAFFIC": "STATICTRAFFIC", "STATIC-TRAFFIC-RT": "STATICTRAFFICRT",
       "STATIC-Q": "STATICQ", "MDP-Q": "MDPQ", "TDOPT-Q": "TDOPTQ", "MDP-Q-OP": "MDPQOP"}
NETFILE = {"shared": "grid8.net.xml", "channelized": "grid8_rt.net.xml"}
KSOURCES = ["LA1", "MDP", "TDOPT_replan", "GREEDY"]


def plan(layout, decide):
    """(free, q, op) cells of one results directory as {den: [methods]} maps."""
    if (layout, decide) == ("shared", 45):
        lv = [0, 100, 50, 25, 15, 10]
        free = {d: ["STATIC", "LA1", "LA2", "LA3", "MDP", "TDOPT_replan", "TDOPT-Q"] for d in lv}
        for d in (0, 15):
            free[d] += ["GREEDY", "TDOPT_open"]
        free[15] += ["STATIC-TRAFFIC"]
        for d in (25, 15, 10):
            free[d] += ["STATIC-TRAFFIC-RT"]
        q = {d: ["STATIC-Q", "MDP-Q"] for d in (100, 50, 25, 15, 10)}
        op = {d: ["MDP-Q-OP"] for d in (25, 15, 10)}
    elif (layout, decide) == ("channelized", 45):
        free = {d: ["STATIC", "LA1", "MDP", "TDOPT_replan", "TDOPT-Q"] for d in (25, 15, 10)}
        q = {d: ["STATIC-Q", "MDP-Q"] for d in (25, 15, 10)}
        op = {15: ["MDP-Q-OP"]}
    elif decide == 100:
        free = {15: ["STATIC", "LA1", "MDP", "TDOPT_replan", "TDOPT-Q"]}
        q, op = {}, {}
    else:
        raise SystemExit(f"no plan for layout {layout}, decide {decide}")
    return free, q, op


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True, choices=sorted(NETFILE))
    ap.add_argument("--decide", type=int, required=True)
    ap.add_argument("--K", type=int, default=10)
    ap.add_argument("--nets", type=int, nargs="*")
    ap.add_argument("--seeds", type=int, nargs="*", default=[11, 12, 13, 14])
    ap.add_argument("--all-seeds", type=int, nargs="*", default=[11, 12, 13, 14],
                    help="seeds of the whole sample: a kernel waits for the source cells of every one")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--port-base", type=int)
    ap.add_argument("--no-wait", action="store_true")
    ap.add_argument("--plan", action="store_true")
    a = ap.parse_args()
    nets = a.nets if a.nets is not None else list(range(a.K))
    allnets = list(range(a.K))
    res = f"results_v2/{a.layout}_d{a.decide}"
    os.makedirs(res, exist_ok=True)
    if os.environ.get("SR_NET") and os.environ["SR_NET"] != NETFILE[a.layout]:
        print(f"note: SR_NET={os.environ['SR_NET']} overridden by --layout {a.layout} ({NETFILE[a.layout]})", flush=True)
    env0 = dict(os.environ, SR_NET=NETFILE[a.layout], SR_DECIDE=str(a.decide), SR_RESULTS=res)
    env0.pop("SR_VERBOSE", None)
    base = a.port_base or {("shared", 45): 9000, ("channelized", 45): 9100,
                           ("shared", 100): 9200, ("channelized", 100): 9300}[(a.layout, a.decide)]
    free, q, op = plan(a.layout, a.decide)

    def out(den, m, j, s):
        return f"{res}/dem{den}_{TAG.get(m, m)}_n{j}_{s}.json"

    def kfile(den, tag, j):
        return f"{res}/kernel_dem{den}{'_op' if tag else ''}_fold{'A' if j % 2 == 0 else 'B'}.json"

    cells = []   # (priority, den, method, net, seed, dependency file or None)
    for den, ms in free.items():
        for m in ms:
            cells += [(0, den, m, j, s, None) for j in nets for s in a.seeds]
    for den, ms in q.items():
        for m in ms:
            cells += [(1, den, m, j, s, kfile(den, "", j)) for j in nets for s in a.seeds]
    if (a.layout, a.decide) == ("shared", 45) and 0 in nets and 11 in a.seeds:
        # demand 0: STATIC-Q and MDP-Q equal STATIC and MDP by construction; checked on one cell
        cells += [(1, 0, m, 0, 11, None) for m in ("STATIC-Q", "MDP-Q")]
    for den, ms in op.items():
        for m in ms:
            cells += [(2, den, m, j, s, kfile(den, "op", j)) for j in nets for s in a.seeds]
    kernels = []
    for den in q:
        srcs = [m for m in KSOURCES if m in free.get(den, [])]
        kernels.append({"den": den, "tag": "", "sources": srcs,
                        "files": [kfile(den, "", 0), kfile(den, "", 1)],
                        "needs": [out(den, m, j, s) for m in srcs for j in allnets for s in a.all_seeds]})
    for den in op:
        kernels.append({"den": den, "tag": "op", "sources": ["MDPQ"],
                        "files": [kfile(den, "op", 0), kfile(den, "op", 1)],
                        "needs": [out(den, "MDP-Q", j, s) for j in allnets for s in a.all_seeds]})
    if a.plan:
        for st, name in enumerate(("kernel-free", "kernel (STATIC-Q, MDP-Q)", "on-policy kernel (MDPQOP)")):
            cs = [c for c in cells if c[0] == st]
            print(f"{name}: {len(cs)} cells, {sum(os.path.exists(out(*c[1:5])) for c in cs)} done")
        return

    log = open(f"{res}/driver_log.txt", "a")
    for k in kernels:     # a lock left by a killed driver would block its kernel forever
        lock = f"{res}/.lock_kernel_dem{k['den']}{k['tag']}"
        if os.path.isdir(lock) and time.time() - os.path.getmtime(lock) > 1800:
            os.rmdir(lock)

    def say(msg):
        line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
        print(line, flush=True)
        log.write(line + "\n"); log.flush()

    def run_cell(c, port):
        _, den, m, j, s, _ = c
        args = ["dem", m, str(s), str(den)]
        env = dict(env0, SR_NETSEED=str(j), SR_PORT=str(port))
        for attempt in (1, 2):
            t0 = time.time()
            p = subprocess.run([PY, "sumo_runner.py"] + args, capture_output=True, text=True, env=env)
            ok = p.returncode == 0 and os.path.exists(out(den, m, j, s))
            lines = [ln for ln in (p.stdout + p.stderr).splitlines()
                     if "Retrying" not in ln and "Could not connect" not in ln]
            say(f"{'done' if ok else 'FAILED'} n{j} {' '.join(args)} attempt {attempt} "
                f"({time.time() - t0:.0f} s)" + ("" if ok else f" exit {p.returncode}: " + " | ".join(lines[-6:])))
            if not ok:
                with open(f"{res}/failures_v2.log", "a") as f:
                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} n{j} {' '.join(args)} attempt {attempt} exit {p.returncode}\n"
                            + "\n".join(lines[-30:]) + "\n\n")
            if ok:
                return True
        return False

    def try_kernels():
        for k in kernels:
            if all(os.path.exists(f) for f in k["files"]):
                continue
            if not all(os.path.exists(f) for f in k["needs"]):
                continue
            lock = f"{res}/.lock_kernel_dem{k['den']}{k['tag']}"
            try:
                os.mkdir(lock)
            except FileExistsError:
                continue
            try:
                if all(os.path.exists(f) for f in k["files"]):
                    continue
                cmd = [PY, "queue_kernel.py", res, "--levels", str(k["den"]), "--nets"] + [str(j) for j in allnets] \
                    + ["--sources"] + k["sources"] + (["--tag", k["tag"]] if k["tag"] else [])
                p = subprocess.run(cmd, capture_output=True, text=True, env=dict(env0, SR_NETSEED="0"))
                say(f"kernel dem{k['den']}{(' ' + k['tag']) if k['tag'] else ''} sources {k['sources']}: exit {p.returncode}\n"
                    + "\n".join("   " + ln for ln in (p.stdout + p.stderr).splitlines()))
            finally:
                os.rmdir(lock)

    pending = sorted((c for c in cells if not os.path.exists(out(*c[1:5]))),
                     key=lambda c: (c[0], {10: 0, 15: 1, 25: 2, 50: 3, 100: 4, 0: 5}[c[1]], c[3], c[4], c[2]))
    say(f"start {res} nets {nets} seeds {a.seeds} K {a.K} jobs {a.jobs}: {len(pending)} of {len(cells)} cells to run")
    ports = list(range(base, base + a.jobs))
    running, failed = {}, []
    last_wait_msg = 0.0
    with cf.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        while pending or running:
            try_kernels()
            ready = [c for c in pending if c[5] is None or os.path.exists(c[5])]
            while ready and ports:
                c = ready.pop(0); pending.remove(c)
                port = ports.pop(0)
                running[ex.submit(run_cell, c, port)] = (c, port)
            if not running:
                if a.no_wait:
                    say(f"{len(pending)} cells blocked on kernels; --no-wait set, stopping")
                    break
                lost = {out(*c[1:5]) for c in failed}
                dead = [k for k in kernels if not all(os.path.exists(f) for f in k["files"])
                        and any(f in lost for f in k["needs"])]
                if dead:
                    say(f"{len(pending)} cells blocked on kernels whose source cells failed in this process "
                        f"({', '.join('dem%d%s' % (k['den'], k['tag']) for k in dead)}); stopping, rerun to retry")
                    break
                if time.time() - last_wait_msg > 1800:
                    say(f"waiting for other processes: {len(pending)} cells blocked on kernels")
                    last_wait_msg = time.time()
                time.sleep(60)
                continue
            done, _ = cf.wait(list(running), timeout=60, return_when=cf.FIRST_COMPLETED)
            for fut in done:
                c, port = running.pop(fut)
                ports.append(port)
                if not fut.result():
                    failed.append(c)
    say(f"DONE {res} nets {nets}: {len(failed)} cells failed after retry" +
        ("".join(f"\n   n{c[3]} dem{c[1]} {c[2]} seed {c[4]}" for c in failed)))


if __name__ == "__main__":
    main()
