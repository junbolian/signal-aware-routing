"""Queue-aware routing layer for the SUMO congestion experiment.

Queue-aware planners that mirror the paper's information ladder:

  STATIC-Q  queue-aware static routing (counterpart of SP-STATIC): no phase
            information; movements priced by expected discharge-aware waits,
            the current crossing by the vehicles the ego sees ahead of it.
  MDP-Q     queue-aware local-information optimum (counterpart of the paper's
            unknown-offset MDP, Proposition 7): the Bellman equation with the
            discharge-aware wait, solved by value iteration; at each stopline
            the ego observes the true local phase and the platoon ahead.
  TDOPT-Q   full timing plus the queue forecast at the planned arrival phase of
            every crossing, rolling replanning (counterpart of TD-OPT replan).

What the queue-aware router knows. (i) For every approach lane, the arrival
rate lambda estimated from the standing queues counted at its recent
through-green onsets (rolling window of ONSET_WINDOW onsets, lambda = mean
onset queue / through red); this is what a traffic-aware router obtains from
probe data. (ii) On its own approach, the vehicles ahead of it with their
positions and whether they are standing. Offsets are never observed; phase
information enters only through the paper's information regimes.

Queue model. Background traffic consists of through-moving corridor flows, so
every vehicle ahead of the ego crosses in the through window of its approach
axis, and a vehicle released at a signal reaches the next signal of the same
corridor together with the vehicles released with it. Two states describe the
ego's position in this structure:
  (e, q)  riding a platoon: q vehicles are ahead of the ego in its lane on
          link e and stay ahead of it at the next stopline if it continues
          straight (platoon persistence);
  (e, f)  just turned onto link e: the ego joined the corridor of e at a point
          unrelated to that corridor's platoon, so the queue it meets at the
          next stopline follows the fluid model of a fixed-time signal with
          uniform arrivals lambda (standing queue growing at rate lambda in
          red, draining at 1/HS - lambda in green); after that crossing it
          rides the platoon it found there.
A standing vehicle ahead is released at the next through onset; a moving one
crosses at max(own arrival, predecessor + HS) if that instant is green and
otherwise stops. The ego needs HS after the vehicle ahead of it and then obeys
its own movement window: S needs the through window, L reaches the stopline
behind the through queue and then waits for the protected-left window, R
passes without a window (right turn on red) but only once the kerb-lane queue
ahead of it has moved.

Convention. Crossing times are ego-relative: with nobody ahead the ego passes
at the paper's P_m(t), so the queue-ignorant planners are the lambda = 0,
empty-platoon special case, and an ego behind q vehicles passes HS*q later.
The real release law onset + LS + HS*k of calibrate_discharge.py differs from
this by a constant that the ego pays on every crossing and that the paper's
per-crossing constant delta absorbs; LS enters only the green capacity.
"""
import heapq
import json
import math
import os

import numpy as np

import signal_routing as sr

CAL_FILE = "discharge_calibration.json"
LS, HS = -0.5, 2.374                    # overwritten by load_calibration()
DXI = 0.5                               # phase grid for head expectations (s)
DXI_MDP = 0.25                          # phase grid for value iteration (s)
ONSET_WINDOW = 5                        # onsets per lane in the rate estimate
QMAX = 20                               # largest platoon carried as state
FLUID = "f"                             # the just-turned state


def load_calibration(path=CAL_FILE):
    global LS, HS
    if os.path.exists(path):
        d = json.load(open(path))
        LS, HS = float(d["LS"]), float(d["HS"])
    return LS, HS


# ----------------------------------------------------------------- helpers

def _through(tm, ax):
    return tm._window(ax, "S")


def _next_onset(t, off, s, C):
    return t + ((s - (t - off)) % C)


def _finish(reach, off, ax, m, tm):
    """Apply the ego's own movement window once it has reached the stopline."""
    if m == "R" and tm.rtor:
        return reach
    return reach + tm.wait(reach, off, ax, "L" if m == "L" else "S")


def lanes_for(m):
    """Lane indices admitting movement m on the two-lane grid approaches:
    lane 0 (kerb) serves R and S, lane 1 serves S and L (grid8.net.xml)."""
    return {"R": (0,), "S": (0, 1), "L": (1,)}[m]


def rates_of(rates, eid, m):
    return [rates.get((eid, l), 0.0) for l in lanes_for(m)]


def _moves(net, e):
    u, v = e
    h = (v[0] - u[0], v[1] - u[1])
    for w, d in net.nbrs(v):
        if w == u:
            continue
        m = net.move_type(v, h, d)
        if m is not None:
            yield m, (v, w), net.axis(v, h)


def monotone_moves(net, e, dest):
    u, v = e
    for m, e2, ax in _moves(net, e):
        if net.dist(e2[1], dest) < net.dist(v, dest):
            yield m, e2, ax


# ------------------------------------------------- current approach: platoon

def platoon_cross(arrivals, off, ax, tm):
    """Model crossing time of the last vehicle of the platoon ahead of the ego.
    arrivals: unimpeded stopline arrival times, nearest-to-stopline first; a
    standing vehicle has arrival = observation time. Returns None if empty."""
    if not arrivals:
        return None
    C = tm.C
    s, g = _through(tm, ax)
    prev = None
    for a in arrivals:
        cand = a if prev is None else max(a, prev + HS)
        xi = (cand - off) % C
        prev = cand if s <= xi < s + g else _next_onset(cand, off, s, C)
    return prev


def lane_pass(t, off, ax, m, tm, arrivals):
    last = platoon_cross(arrivals, off, ax, tm)
    reach = t if last is None else max(t, last + HS)
    return _finish(reach, off, ax, m, tm)


def head_pass(t, off, ax, m, tm, platoons):
    """Ego passage at the current intersection from the observed platoons,
    platoons = {lane: [arrival times]}, over the best admissible lane."""
    return min(lane_pass(t, off, ax, m, tm, platoons.get(l, [])) for l in lanes_for(m))


def head_ewait_lane(t, ax, m, tm, arrivals):
    """Expected head delay over a uniform unknown offset, one lane (STATIC-Q)."""
    n = int(round(tm.C / DXI))
    tot = 0.0
    for i in range(n):
        off = (i + 0.5) * DXI
        tot += lane_pass(t, off, ax, m, tm, arrivals) - t
    return tot / n


def head_ewait(t, ax, m, tm, platoons):
    return min(head_ewait_lane(t, ax, m, tm, platoons.get(l, [])) for l in lanes_for(m))


# ------------------------------------- downstream waits on the phase grid

def _wait_vec(x, s, d, C):
    xi = np.mod(x, C)
    return np.where((xi >= s) & (xi < s + d), 0.0, np.mod(s - xi, C))


def _finish_vec(x, ax, m, tm):
    if m == "R" and tm.rtor:
        return x
    s, d = tm._window(ax, "L" if m == "L" else "S")
    return x + _wait_vec(x, s, d, tm.C)


def fluid_q_vec(xi, ax, lam, tm):
    """Standing through queue met by an ego arriving at local phase xi on a
    lane with arrival rate lam (fluid model)."""
    C = tm.C
    s, g = _through(tm, ax)
    r = C - g
    xi = np.asarray(xi, dtype=float)
    ingreen = (xi >= s) & (xi < s + g)
    qg = np.maximum(0.0, lam * r - (1.0 / HS - lam) * (xi - s))
    qr = lam * np.mod(xi - (s + g), C)
    return np.where(ingreen, qg, qr)


def fluid_wait_vec(xi, ax, m, tm, lam):
    """Expected ego delay at a just-turned crossing, arrival phase xi, rate lam.
    During red the number of vehicles ahead is Poisson with the fluid mean q:
    with probability 1 - exp(-q) at least one vehicle stands ahead and the ego
    cannot reach the stopline before the through onset. lam = 0 reproduces the
    paper's w_m(xi) exactly."""
    C = tm.C
    s, g = _through(tm, ax)
    xi = np.asarray(xi, dtype=float)
    ingreen = (xi >= s) & (xi < s + g)
    q = fluid_q_vec(xi, ax, lam, tm)
    rem = s + g - xi
    over = HS * q > rem
    onset_after = (xi + rem) + np.mod(s - (xi + rem), C)
    reach_g = np.where(over, onset_after + HS * (q - rem / HS), xi + HS * q)
    pass_g = _finish_vec(reach_g, ax, m, tm)
    onset = xi + np.mod(s - xi, C)
    if m == "S":
        pass_r = _finish_vec(onset + HS * q, ax, m, tm)
    else:
        p1 = 1.0 - np.exp(-q)
        pass_r = (1.0 - p1) * _finish_vec(xi, ax, m, tm) + p1 * _finish_vec(onset + HS * q, ax, m, tm)
    return np.where(ingreen, pass_g, pass_r) - xi


_PW = {}


def platoon_wait_vec(xi, ax, m, tm, q):
    """Ego delay at a crossing reached while riding a compact platoon of q
    vehicles ahead in its lane (arrivals xi - HS*q, ..., xi - HS)."""
    key = (tm.C, tm.gs, tm.gl, tm.rtor, HS, ax, m, q, len(xi), float(xi[0]))
    if key in _PW:
        return _PW[key]
    s, g = _through(tm, ax)
    out = np.empty(len(xi))
    for i, x in enumerate(xi):
        arr = [x - HS * (q - j) for j in range(q)]
        out[i] = lane_pass(x, 0.0, ax, m, tm, arr) - x
    _PW[key] = out
    return out


# ------------------------------------------------------- queue-aware values

import queue_kernel as qk


def _phase_bins(P, ax, tm):
    """Bin of the local passage phase P (array) relative to the cross-axis
    through window: 0 during it, 1 within 40 s after its end, 2 later."""
    cax = "EW" if ax == "NS" else "NS"
    s, g = tm._window(cax, "S")
    since = np.mod(P - s, tm.C)
    return np.where(since < g, 0, np.where(since < g + 40.0, 1, 2))


def solve_values(net, tm, dest, kernel, sedge, static=False, tol=1e-4, max_sweeps=300):
    """Value iteration over the states (e, q), q = 0..QMAX: the ego rides link
    e with q vehicles ahead in its lane. The head wait at the next stopline is
    the platoon wait W_m(xi, q); the count ahead after the crossing follows the
    empirical kernel P(q' | q, m, phase bin) (queue_kernel.py), the marginal
    kernel when static=True. static=False: V(e,q) = E_xi[min_m(...)], the
    local-information optimum (MDP-Q); static=True: V(e,q) = min_m E_xi[...]
    (STATIC-Q). L1-monotone action set. kernel=None reproduces the paper's
    values on q = 0 exactly (no vehicles ahead anywhere). Returns (V, sweeps)."""
    XI = np.arange(0.0, tm.C, DXI_MDP) + DXI_MDP / 2
    links = []
    for x in range(net.N):
        for y in range(net.N):
            u = (x, y)
            for v, d in net.nbrs(u):
                links.append((u, v))
    order = sorted(links, key=lambda e: net.dist(e[1], dest))
    Q = np.arange(QMAX + 1)
    QB = np.minimum(Q, qk.QB_MAX)
    PW, BIN = {}, {}
    for e in links:
        if e[1] == dest:
            continue
        for m, e2, ax in monotone_moves(net, e, dest):
            PW[(e, m)] = np.stack([platoon_wait_vec(XI, ax, m, tm, q) for q in Q])
            BIN[(e, m)] = _phase_bins(XI[None, :] + PW[(e, m)], ax, tm)     # (Q+1, XI)
    V = {e: np.zeros(QMAX + 1) for e in links}
    for sweep in range(1, max_sweeps + 1):
        delta = 0.0
        for e in order:
            if e[1] == dest:
                continue
            best = None
            for m, e2, ax in monotone_moves(net, e, dest):
                c = net.t_cross + net.link_t(*e2)
                if kernel is None:
                    cont = np.broadcast_to(V[e2][:, None], (QMAX + 1, len(XI))) if m == "S" \
                        else np.full((QMAX + 1, len(XI)), V[e2][0])
                else:
                    Vk = V[e2][:qk.QK + 1]
                    if static:
                        M = kernel.Km[kernel.MV[m]] @ Vk                    # (QB+1,)
                        cont = M[QB][:, None] + np.zeros((1, len(XI)))
                    else:
                        M = kernel.K[kernel.MV[m]] @ Vk                     # (NBIN, QB+1)
                        cont = M[BIN[(e, m)], QB[:, None]]
                cq = PW[(e, m)] + c + cont
                if static:
                    cq = np.zeros_like(cq) + cq.mean(axis=1, keepdims=True)
                best = cq if best is None else np.minimum(best, cq)
            if best is None:
                V[e] = np.full(QMAX + 1, 1e9)
                continue
            nq = best.mean(axis=1)
            delta = max(delta, float(np.abs(nq - V[e]).max()))
            V[e] = nq
        if delta < tol:
            break
    return V, sweep


def _cont_obs(m, q, P_local, ax, tm, kernel, Vnext, static):
    if kernel is None:
        return Vnext[min(q, QMAX)] if m == "S" else Vnext[0]
    pb = None if static else int(_phase_bins(np.array([P_local]), ax, tm)[0])
    return float(kernel.row(m, q, pb) @ Vnext[:qk.QK + 1])


def decide(net, tm, e, t, dest, V, platoons, kernel, off=None, with_value=False):
    """Decision at the stopline of link e with observed platoons. off given:
    true local phase observed (MDP-Q); off None: unknown offset, the head
    averaged over a uniform offset and the marginal kernel (STATIC-Q)."""
    C = tm.C
    static = off is None
    offs = [off] if off is not None else [(i + 0.5) * DXI for i in range(int(round(C / DXI)))]
    best = (float("inf"), None)
    for m, e2, ax in monotone_moves(net, e, dest):
        c = net.t_cross + net.link_t(*e2)
        for l in lanes_for(m):
            arr = platoons.get(l, [])
            tot = 0.0
            for o in offs:
                P = lane_pass(t, o, ax, m, tm, arr)
                tot += P + c + _cont_obs(m, len(arr), (P - o) % C, ax, tm, kernel, V[e2], static)
            val = tot / len(offs)
            if val < best[0] - 1e-9:
                best = (val, e2)
    return (best[1], best[0]) if with_value else best[1]


def first_link(net, orig, V, platoons_by_edge, sedge):
    """Departure decision: the ego enters link (orig, w) at its start and sees
    the vehicles ahead on it; value = link time + V((orig,w), q_ahead)."""
    best = (float("inf"), None)
    for w, d in net.nbrs(orig):
        e = (orig, w)
        pl = platoons_by_edge.get(sedge(orig, w), {})
        q = min(len(pl.get(l, [])) for l in (0, 1)) if pl else 0
        c = net.link_t(orig, w) + V[e][min(q, QMAX)]
        if c < best[0] - 1e-9:
            best = (c, e)
    return best[1]


# ----------------------------------------------- full timing: TDOPT-Q search

def _down_pass(t, off, ax, m, tm, state, lams):
    """Deterministic passage at a downstream crossing in state q or f; returns
    (passage time, next state for the S continuation)."""
    C = tm.C
    xi = np.array([(t - off) % C])
    if state == FLUID:
        k = int(np.argmin([fluid_wait_vec(xi, ax, m, tm, lam)[0] for lam in lams]))
        w = float(fluid_wait_vec(xi, ax, m, tm, lams[k])[0])
        qn = int(min(round(float(fluid_q_vec(xi, ax, lams[k], tm)[0])), QMAX))
        return t + w, qn
    q = state
    arr = [t - HS * (q - j) for j in range(q)]
    return lane_pass(t, off, ax, m, tm, arr), q


def tdoptq_path(net, tm, e, t, dest, rates, platoons, sedge):
    """Label setting over (link, state) under true offsets from link e at
    time t. platoons: observed platoons on e (head); None prices the first
    crossing from the fluid model (departure decision). Every arc map is
    nondecreasing in t, so label setting is exact. Returns the link path."""
    start = (e, FLUID if platoons is None else "head")
    best = {start: t}
    pred = {start: None}
    pq = [(t, 0, start)]
    goal = None
    cnt = 1
    while pq:
        tt, _, st = heapq.heappop(pq)
        if tt > best.get(st, float("inf")) + 1e-9:
            continue
        ee, state = st
        u, v = ee
        if v == dest:
            goal = st
            break
        for m, e2, ax in _moves(net, ee):
            lams = rates_of(rates, sedge(u, v), m)
            if state == "head":
                cands = []
                for l in lanes_for(m):
                    arr = platoons.get(l, [])
                    cands.append((lane_pass(tt, net.offset[v], ax, m, tm, arr), len(arr)))
                p, qn = min(cands)
            else:
                p, qn = _down_pass(tt, net.offset[v], ax, m, tm, state, lams)
            t2 = p + net.t_cross + net.link_t(*e2)
            st2 = (e2, qn if m == "S" else FLUID)
            if t2 < best.get(st2, float("inf")) - 1e-9:
                best[st2] = t2
                pred[st2] = st
                cnt += 1
                heapq.heappush(pq, (t2, cnt, st2))
    path = []
    cur = goal
    while cur is not None:
        path.append(cur[0])
        cur = pred[cur]
    path.reverse()
    return path, best[goal]


def tdoptq_first_link(net, tm, orig, t0, dest, rates, platoons_by_edge, sedge):
    best = (float("inf"), None)
    for w, d in net.nbrs(orig):
        e = (orig, w)
        ta = t0 + net.link_t(orig, w)
        pl = platoons_by_edge.get(sedge(orig, w), {})
        path, val = tdoptq_path(net, tm, e, ta, dest, rates, pl if pl else None, sedge)
        if val < best[0] - 1e-9:
            best = (val, e)
    return best[1]
