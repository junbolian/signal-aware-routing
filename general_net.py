"""General (non-grid) network for the signal-aware routing model.

GNet exposes the same three topology queries as signal_routing.Net --
axis(v, h), move_type(v, h, d) and dist(v, dest) -- so every router in
signal_routing.py, lookahead.py, extend_lookahead.py, mdp_policy.py and
queue_model.py runs unchanged on a real street network.

Two things differ from the grid and are handled here:

  * Axes are per node. A signalized node has two axes A and B obtained by
    clustering the headings of its approaches; axis(v, h) returns the key
    (v, "A") or (v, "B"). Because the key carries the node, GTiming can hold a
    different four-phase split per node behind the same wait()/ewait()
    signature that signal_routing.Timing uses.
  * The monotone class is "strictly decreases the free-flow time to the
    destination", computed by dist(), replacing the grid's L1 distance.

Geometry is in metres, headings are unit vectors in the (x, y) plane, and
angles follow the usual convention: a positive turn angle is counter-clockwise,
which under right-hand traffic is a left turn.
"""
import heapq
import math

# movement geometry thresholds (degrees)
STRAIGHT_MAX = 40.0     # |angle| < 40 is a through movement
TURN_MAX = 150.0        # beyond 150 the movement is a U-turn
AXIS_MIN_SEP = 45.0     # two axes closer than this cannot be separated
MAX_LEGS = 4            # five or more legs cannot be reduced to two axes


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    if n < 1e-9:
        raise ValueError("zero-length link")
    return (dx / n, dy / n)


def _bearing(d):
    """Heading of a unit vector in degrees, in [0, 360)."""
    return math.degrees(math.atan2(d[1], d[0])) % 360.0


def _turn_angle(h, d):
    """Signed turn from heading h to heading d, in (-180, 180].
    Positive is counter-clockwise, i.e. a left turn."""
    a = math.degrees(math.atan2(h[0] * d[1] - h[1] * d[0], h[0] * d[0] + h[1] * d[1]))
    return a if a > -180.0 else 180.0


def _axis_sep(a, b):
    """Separation of two undirected street directions, in [0, 90]."""
    s = abs((a - b) % 180.0)
    return min(s, 180.0 - s)


class GTiming:
    """Per-node four-phase fixed-time plan in the order (A,S), (A,L), (B,S),
    (B,L) -- or the left-first order when straight_first is False.

    The axis key is the (node, "A"|"B") pair produced by GNet.axis, so the
    signature of wait() and ewait() matches signal_routing.Timing exactly and
    the routers need no change; the difference is that the split may vary from
    node to node, which the grid's single global plan cannot express."""

    def __init__(self, C, splits, straight_first=True, rtor=True):
        # splits: node -> (g_A_S, g_A_L, g_B_S, g_B_L), summing to C
        self.C = C
        self.rtor = rtor
        self.win = {}
        for v, (gas, gal, gbs, gbl) in splits.items():
            if straight_first:
                seq = [((v, "A"), "S", gas), ((v, "A"), "L", gal),
                       ((v, "B"), "S", gbs), ((v, "B"), "L", gbl)]
            else:
                seq = [((v, "A"), "L", gal), ((v, "A"), "S", gas),
                       ((v, "B"), "L", gbl), ((v, "B"), "S", gbs)]
            t = 0.0
            for ax, m, dur in seq:
                self.win[(ax, m)] = (t, dur)
                t += dur
            if abs(t - C) > 1e-9:
                raise ValueError(f"phase durations at {v} sum to {t}, not {C}")

    def _window(self, ax, mtype):
        if mtype == "R":
            mtype = "S"     # only reached when rtor is False
        return self.win[(ax, mtype)]

    def wait(self, t, offset, ax, mtype):
        if mtype == "R" and self.rtor:
            return 0.0
        s, d = self._window(ax, mtype)
        tau = (t - offset) % self.C
        if s <= tau < s + d:
            return 0.0
        return (s - tau) % self.C

    def ewait(self, ax, mtype):
        if mtype == "R" and self.rtor:
            return 0.0
        _, d = self._window(ax, mtype)
        r = self.C - d
        return r * r / (2.0 * self.C)


class GNet:
    """A signalized street network of arbitrary topology.

    nodes: id -> (x, y) in metres.
    links: directed (u, v) with length_m and speed_mps; tau[(u, v)] is the
           free-flow time length / speed.
    Only nodes whose approaches cluster into two axes at least AXIS_MIN_SEP
    apart, with at most MAX_LEGS legs, are admissible; the rest are reported in
    self.excluded and carry no signal."""

    def __init__(self, nodes, links, t_cross=2.0):
        self.nodes = dict(nodes)
        self.tau = {}
        self.length = {}
        self.speed = {}
        for (u, v), (length_m, speed_mps) in links.items():
            self.tau[(u, v)] = length_m / speed_mps
            self.length[(u, v)] = length_m
            self.speed[(u, v)] = speed_mps
        self.t_cross = t_cross
        self._adj = {}
        for (u, v) in self.tau:
            self._adj.setdefault(u, []).append(v)
        for u in self.nodes:
            self._adj.setdefault(u, [])
        # the guards in the routers are 10 * N * N steps; N = node count is ample
        self.N = len(self.nodes)
        self.offset = {}
        self._dist_cache = {}
        self._heading = {}
        for (u, v) in self.tau:
            xu, yu = self.nodes[u]
            xv, yv = self.nodes[v]
            self._heading[(u, v)] = _unit(xv - xu, yv - yu)
        self.axes, self.excluded = self._build_axes()

    # ----------------------------------------------------------- topology
    def nbrs(self, v):
        for w in self._adj.get(v, []):
            yield w, self._heading[(v, w)]

    def link_t(self, u, v):
        return self.tau[(u, v)]

    def _build_axes(self):
        """Cluster each node's approach directions into two axes.

        Axis A and axis B are the pair of approach directions (taken modulo
        180 degrees, a street being a line) with the largest mutual
        separation; every other approach joins whichever of the two it aligns
        with better, by larger |cos|. A node with five or more legs, or whose
        two axes are closer than AXIS_MIN_SEP, cannot be reduced to the
        model's two-axis plan and is excluded."""
        axes, excluded = {}, []
        for v in self.nodes:
            legs = list(self._adj.get(v, []))
            if not legs:
                continue
            dirs = {w: _bearing(self._heading[(v, w)]) % 180.0 for w in legs}
            if len(legs) > MAX_LEGS:
                excluded.append((v, f"{len(legs)} legs"))
                continue
            if len(legs) == 1:
                # a stub has a single axis; it can never host a through/turn
                # decision, so it is excluded rather than given a degenerate plan
                excluded.append((v, "1 leg"))
                continue
            best = (-1.0, None, None)
            ws = sorted(dirs)
            for i in range(len(ws)):
                for j in range(i + 1, len(ws)):
                    sep = _axis_sep(dirs[ws[i]], dirs[ws[j]])
                    if sep > best[0] + 1e-9:
                        best = (sep, ws[i], ws[j])
            sep, wa, wb = best
            if sep < AXIS_MIN_SEP:
                excluded.append((v, f"axes {sep:.1f} deg apart"))
                continue
            ang_a, ang_b = dirs[wa], dirs[wb]
            assign = {}
            for w in legs:
                ca = abs(math.cos(math.radians(dirs[w] - ang_a)))
                cb = abs(math.cos(math.radians(dirs[w] - ang_b)))
                assign[w] = "A" if ca >= cb else "B"
            axes[v] = {"A_deg": ang_a, "B_deg": ang_b, "sep_deg": sep,
                       "leg": assign}
        return axes, excluded

    # -------------------------------------------- the three model queries
    def axis(self, v, h):
        """Axis key of the approach whose heading is h at node v.

        h is the heading of the link arriving at v, so the approach direction
        is the same line; the key carries the node so that GTiming can hold a
        per-node split."""
        a = self.axes.get(v)
        if a is None:
            raise KeyError(f"node {v} has no two-axis plan (excluded)")
        b = _bearing(h) % 180.0
        ca = abs(math.cos(math.radians(b - a["A_deg"])))
        cb = abs(math.cos(math.radians(b - a["B_deg"])))
        return (v, "A") if ca >= cb else (v, "B")

    def move_type(self, v, h, d):
        """S, L, R or None (U-turn) from the signed turn angle."""
        ang = _turn_angle(h, d)
        if abs(ang) < STRAIGHT_MAX:
            return "S"
        if STRAIGHT_MAX <= ang <= TURN_MAX:
            return "L"
        if -TURN_MAX <= ang <= -STRAIGHT_MAX:
            return "R"
        return None

    def dist(self, v, dest):
        """Free-flow travel time from v to dest, the general replacement for
        the grid's L1 distance. The monotone class is "strictly decreases
        dist"."""
        d = self._dist_cache.get(dest)
        if d is None:
            d = self._reverse_dijkstra(dest)
            self._dist_cache[dest] = d
        return d.get(v, float("inf"))

    def _reverse_dijkstra(self, dest):
        rev = {}
        for (u, v), t in self.tau.items():
            rev.setdefault(v, []).append((u, t))
        best = {dest: 0.0}
        pq = [(0.0, dest)]
        while pq:
            c, v = heapq.heappop(pq)
            if c > best.get(v, float("inf")) + 1e-12:
                continue
            for u, t in rev.get(v, []):
                nc = c + t
                if nc < best.get(u, float("inf")) - 1e-12:
                    best[u] = nc
                    heapq.heappush(pq, (nc, u))
        return best

    # ------------------------------------------------------------ plans
    def set_offsets(self, rng, C):
        """Offsets i.i.d. uniform on [0, C) at every admissible node."""
        self.offset = {v: rng.uniform(0.0, C) for v in self.axes}
        return self.offset

    def splits(self, C, arterial, g_hi=(45.0, 20.0), g_lo=(35.0, 20.0),
               g_eq=(40.0, 20.0)):
        """Per-node four-phase durations (A_S, A_L, B_S, B_L).

        arterial: node -> "A", "B" or None. The axis carrying the higher OSM
        road class gets the longer through phase; equal classes split evenly.
        Every plan sums to C."""
        out = {}
        for v in self.axes:
            side = arterial.get(v)
            if side is None:
                a_s, a_l = g_eq
                b_s, b_l = g_eq
            elif side == "A":
                a_s, a_l = g_hi
                b_s, b_l = g_lo
            else:
                a_s, a_l = g_lo
                b_s, b_l = g_hi
            total = a_s + a_l + b_s + b_l
            if abs(total - C) > 1e-9:
                raise ValueError(f"split at {v} sums to {total}, not {C}")
            out[v] = (a_s, a_l, b_s, b_l)
        return out

    # ------------------------------------------------------------ helpers
    def restrict_to_admissible(self):
        """Drop excluded nodes and every link touching them, then keep the
        largest strongly connected component. Returns a new GNet."""
        keep = set(self.axes)
        links = {}
        for (u, v), t in self.tau.items():
            if u in keep and v in keep:
                links[(u, v)] = (self.length[(u, v)], self.speed[(u, v)])
        sub = GNet({v: self.nodes[v] for v in keep}, links, self.t_cross)
        comp = sub.largest_scc()
        links = {(u, v): (sub.length[(u, v)], sub.speed[(u, v)])
                 for (u, v) in sub.tau if u in comp and v in comp}
        return GNet({v: sub.nodes[v] for v in comp}, links, self.t_cross)

    def largest_scc(self):
        """Tarjan, iterative."""
        index, low, onstack, stack, comps = {}, {}, set(), [], []
        counter = [0]
        for root in self.nodes:
            if root in index:
                continue
            work = [(root, iter(self._adj.get(root, [])))]
            index[root] = low[root] = counter[0]
            counter[0] += 1
            stack.append(root)
            onstack.add(root)
            while work:
                v, it = work[-1]
                advanced = False
                for w in it:
                    if w not in index:
                        index[w] = low[w] = counter[0]
                        counter[0] += 1
                        stack.append(w)
                        onstack.add(w)
                        work.append((w, iter(self._adj.get(w, []))))
                        advanced = True
                        break
                    if w in onstack:
                        low[v] = min(low[v], index[w])
                if advanced:
                    continue
                work.pop()
                if work:
                    low[work[-1][0]] = min(low[work[-1][0]], low[v])
                if low[v] == index[v]:
                    comp = set()
                    while True:
                        w = stack.pop()
                        onstack.discard(w)
                        comp.add(w)
                        if w == v:
                            break
                    comps.append(comp)
        return max(comps, key=len) if comps else set()
