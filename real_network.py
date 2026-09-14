"""Extract a real signalized street network for the signal-aware routing study.

The geometry, link lengths, intersection structure and speed limits are real
(OpenStreetMap); the signal plans are synthetic fixed-time plans of the paper's
model class (Section 2.2), assigned separately. No field timing data is used.

Pipeline:
  1. osmnx.graph_from_bbox(network_type="drive", simplify=True).
  2. Drop one-way links: an edge survives only if both directions exist.
  3. Keep the nodes tagged highway=traffic_signals as candidate intersections
     and contract every other node, summing lengths and taking the minimum
     speed limit and the most arterial road class along the contracted chain.
  4. Apply the two-axis test of general_net.GNet to a fixed point: a signalized
     node whose approaches do not cluster into two axes at least 45 degrees
     apart is not an intersection of the model, so it leaves the keep-set and
     is contracted through on the next pass.
  5. Keep the largest strongly connected component.
  6. Check the acceptance requirements; fall through to the next candidate if
     any fails.

Writes real_net.json and, unless --no-sumo, the matching real.net.xml.

Usage:  python real_network.py [--area sacramento|slc|evanston] [--no-sumo]
"""
import argparse
import json
import math
import os
import statistics
import sys
import urllib.request

import general_net as gn

# name, north, south, east, west
#
# The three bounding boxes of the study plan are listed first and are tried
# first. None of them meets the acceptance requirements once one-way links are
# dropped, because that step removes the crossing structure: in the prescribed
# Sacramento box the surviving two-way subgraph has 40 nodes of degree one and
# 36 of degree two against only 5 of degree three or more, so it is a union of
# paths rather than a grid, and the prescribed Salt Lake City and Evanston
# boxes are likewise too small (see "tried" in real_net.json for the counts).
# The enlarged Evanston boxes keep the same centre and street pattern and are
# used as documented fallbacks.
CANDIDATES = [
    ("sacramento", 38.585, 38.565, -121.465, -121.500),
    ("slc",        40.775, 40.755, -111.880, -111.905),
    ("evanston",   42.060, 42.040,  -87.670,  -87.700),
    ("evanston-2x", 42.070, 42.030, -87.655, -87.715),
    ("evanston-3x", 42.080, 42.020, -87.640, -87.730),
]
MIN_NODES = 40
MIN_TWOWAY_LINKS = 60
MIN_CV = 0.25
DEFAULT_SPEED_KPH = 40.0
HIGH_CLASS = {"motorway", "trunk", "primary", "secondary",
              "motorway_link", "trunk_link", "primary_link", "secondary_link"}


def _speed_mps(data):
    v = data.get("maxspeed")
    if isinstance(v, list):
        v = v[0]
    if isinstance(v, str):
        t = v.strip().lower()
        try:
            if t.endswith("mph"):
                return float(t.replace("mph", "").strip()) * 0.44704
            return float(t.split()[0]) / 3.6
        except ValueError:
            pass
    elif isinstance(v, (int, float)):
        return float(v) / 3.6
    return DEFAULT_SPEED_KPH / 3.6


def _names(data):
    v = data.get("name")
    if isinstance(v, list):
        return [x for x in v if isinstance(x, str)]
    return [v] if isinstance(v, str) else []


def _hw(data):
    h = data.get("highway")
    if isinstance(h, list):
        h = h[0]
    return h or "residential"


def fetch_graph(north, south, east, west):
    import osmnx as ox
    try:                                   # osmnx >= 2.0
        return ox.graph_from_bbox(bbox=(west, south, east, north),
                                  network_type="drive", simplify=True)
    except TypeError:                      # osmnx 1.x
        return ox.graph_from_bbox(north=north, south=south, east=east, west=west,
                                  network_type="drive", simplify=True)


def signalized(G):
    return {n for n, d in G.nodes(data=True)
            if "traffic_signals" in str(d.get("highway", ""))}


def two_way_adj(G):
    """Undirected adjacency of the links surviving the one-way filter."""
    best = {}
    for u, v, data in G.edges(data=True):
        L = float(data.get("length", 0.0))
        if (u, v) not in best or L < best[(u, v)][0]:
            best[(u, v)] = (L, _speed_mps(data), _hw(data), (_names(data) or [""])[0])
    adj = {}
    for (u, v), rec in best.items():
        if (v, u) in best:
            adj.setdefault(u, {})[v] = rec
    return adj


CONTINUE_MAX_DEG = 60.0    # a chain may bend by at most this much per node


def contract_to(adj, keep, xy):
    """Links between keep-nodes, contracting every node in between.

    An unsignalized node is usually a minor crossing of degree three or four,
    not a degree-two chain link, so the chain continues along the straightest
    branch: the one whose heading deviates least from the incoming heading, and
    only if that deviation is below CONTINUE_MAX_DEG. This keeps a corridor
    running through its minor intersections instead of ending at the first
    one."""
    out = {}
    for s in keep:
        for first in adj.get(s, {}):
            L, sp, cls, name = adj[s][first]
            prev, cur = s, first
            guard = 0
            while cur not in keep:
                guard += 1
                if guard > 400:
                    cur = None
                    break
                nbrs = [w for w in adj.get(cur, {}) if w != prev]
                if not nbrs:
                    cur = None
                    break
                hx, hy = xy[cur][0] - xy[prev][0], xy[cur][1] - xy[prev][1]
                hn = math.hypot(hx, hy) or 1.0
                best_dev, w = None, None
                for cand in nbrs:
                    dx, dy = xy[cand][0] - xy[cur][0], xy[cand][1] - xy[cur][1]
                    dn = math.hypot(dx, dy) or 1.0
                    dot = (hx * dx + hy * dy) / (hn * dn)
                    dev = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
                    if best_dev is None or dev < best_dev:
                        best_dev, w = dev, cand
                if best_dev > CONTINUE_MAX_DEG:
                    cur = None
                    break
                L2, sp2, cls2, nm2 = adj[cur][w]
                L += L2
                sp = min(sp, sp2)
                if cls2 in HIGH_CLASS:
                    cls = cls2
                if not name:
                    name = nm2
                prev, cur = cur, w
            if cur is None or cur == s:
                continue
            if (s, cur) not in out or L < out[(s, cur)][0]:
                out[(s, cur)] = (L, sp, cls, name)
    return {k: v for k, v in out.items() if (k[1], k[0]) in out}


def build_network(G, sig, xy, verbose=True, area=""):
    """Contract and apply the two-axis test to a fixed point."""
    adj = two_way_adj(G)
    keep = {n for n in sig if n in adj}
    if verbose:
        print(f"[{area}]   two-way subgraph: {sum(len(d) for d in adj.values())} "
              f"directed edges; {len(keep)} signalized nodes on it")
    exlog = []
    net, links = gn.GNet({}, {}), {}
    for it in range(15):
        links = contract_to(adj, keep, xy)
        used = {n for k in links for n in k}
        if not used:
            return gn.GNet({}, {}), {}, exlog
        net = gn.GNet({n: xy[n] for n in used},
                      {k: (v[0], v[1]) for k, v in links.items()})
        if it == 0 and verbose:
            print(f"[{area}]   first contraction: {len(links)} directed links, "
                  f"{len(used)} nodes; axis test rejects {len(net.excluded)}")
        bad = {v for v, _ in net.excluded}
        exlog += [(v, why) for v, why in net.excluded]
        stale = (keep - used) | bad
        if not stale:
            if verbose:
                print(f"[{area}]   axis fixed point after {it + 1} pass(es): "
                      f"{len(net.axes)} intersections, {len(net.tau)} directed links")
            return net, links, exlog
        keep -= stale
    return net, links, exlog


def build(area, north, south, east, west, verbose=True):
    G = fetch_graph(north, south, east, west)
    sig = signalized(G)
    if verbose:
        print(f"[{area}] raw graph: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges, {len(sig)} signalized")
    lat0 = (north + south) / 2.0
    mx = 111320.0 * math.cos(math.radians(lat0))
    my = 110540.0
    xy = {n: ((G.nodes[n]["x"] - west) * mx, (G.nodes[n]["y"] - south) * my)
          for n in G.nodes}
    net, links, exlog = build_network(G, sig, xy, verbose, area)
    net = net.restrict_to_admissible()
    meta = {k: links[k] for k in net.tau if k in links}
    taus = [net.tau[k] for k in net.tau]
    cv = statistics.pstdev(taus) / statistics.mean(taus) if taus else 0.0
    stats = {"area": area, "nodes": len(net.nodes), "links": len(net.tau),
             "twoway_pairs": len(net.tau) // 2, "cv": cv,
             "tau_mean": statistics.mean(taus) if taus else 0.0,
             "tau_min": min(taus) if taus else 0.0,
             "tau_max": max(taus) if taus else 0.0}
    if verbose:
        print(f"[{area}] final: {stats['nodes']} intersections, {stats['links']} "
              f"directed links ({stats['twoway_pairs']} two-way), CV {cv:.3f}")
    return net, meta, stats, (north, south, east, west), exlog


def arterial_side(net, meta):
    """Which axis at each node carries the higher OSM road class."""
    out = {}
    for v in net.axes:
        side = {"A": 0, "B": 0}
        for w, d in net.nbrs(v):
            cls = meta.get((v, w), (0, 0, "residential", ""))[2]
            side[net.axis(v, d)[1]] += 1 if cls in HIGH_CLASS else 0
        out[v] = "A" if side["A"] > side["B"] else ("B" if side["B"] > side["A"] else None)
    return out


def histogram(taus, bins=10):
    lo, hi = min(taus), max(taus)
    w = (hi - lo) / bins if hi > lo else 1.0
    rows = []
    for i in range(bins):
        a, b = lo + i * w, lo + (i + 1) * w
        n = sum(1 for t in taus if (a <= t < b or (i == bins - 1 and t == hi)))
        rows.append((a, b, n))
    return rows


OVERPASS = ["https://overpass-api.de/api/map?bbox=",
            "https://overpass.kumi.systems/api/map?bbox=",
            "https://overpass.private.coffee/api/map?bbox="]


def fetch_osm(bbox, path):
    """Raw OSM XML for netconvert. Overpass refuses requests without a
    User-Agent (HTTP 406), and single mirrors go down, so try a few."""
    north, south, east, west = bbox
    q = f"{west},{south},{east},{north}"
    last = None
    for base in OVERPASS:
        try:
            print(f"downloading OSM extract from {base.split('/')[2]} -> {path}")
            req = urllib.request.Request(
                base + q, headers={"User-Agent": "signal-aware-routing/1.0 (research)"})
            with urllib.request.urlopen(req, timeout=300) as r, open(path, "wb") as f:
                f.write(r.read())
            if os.path.getsize(path) > 100000:
                return path
            last = f"short file ({os.path.getsize(path)} bytes)"
        except Exception as exc:
            last = str(exc)
            print(f"   failed: {last}")
    raise RuntimeError(f"could not fetch OSM extract: {last}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--area", default=None)
    ap.add_argument("--no-sumo", action="store_true")
    a = ap.parse_args()
    cands = [c for c in CANDIDATES if a.area is None or c[0] == a.area]
    chosen, tried = None, []
    for area, n, s, e, w in cands:
        try:
            net, meta, stats, bbox, exlog = build(area, n, s, e, w)
        except Exception as exc:
            print(f"[{area}] failed: {exc}")
            tried.append({"area": area, "error": str(exc)})
            continue
        fails = []
        if stats["nodes"] < MIN_NODES:
            fails.append(f"intersections {stats['nodes']} < {MIN_NODES}")
        if stats["twoway_pairs"] < MIN_TWOWAY_LINKS:
            fails.append(f"two-way links {stats['twoway_pairs']} < {MIN_TWOWAY_LINKS}")
        if stats["cv"] < MIN_CV:
            fails.append(f"CV {stats['cv']:.3f} < {MIN_CV}")
        stats["requirements_failed"] = fails
        tried.append(dict(stats))
        if fails:
            print(f"[{area}] REJECTED: {'; '.join(fails)}")
            continue
        print(f"[{area}] ACCEPTED")
        chosen = (area, net, meta, stats, bbox, exlog)
        break
    if chosen is None:
        print("no candidate met the requirements")
        json.dump({"tried": tried}, open("real_net_candidates.json", "w"), indent=1)
        return 1
    area, net, meta, stats, bbox, exlog = chosen
    art = arterial_side(net, meta)
    taus = [net.tau[k] for k in net.tau]
    doc = {
        "provenance": "OpenStreetMap geometry, link lengths and speed limits; "
                      "synthetic fixed-time signal plans of the paper's model class "
                      "(Section 2.2). No field signal-timing data is used.",
        "area": area, "bbox_north_south_east_west": list(bbox), "stats": stats,
        "tried": tried,
        "nodes": {str(v): list(net.nodes[v]) for v in net.nodes},
        "links": [{"u": str(u), "v": str(v), "tau": net.tau[(u, v)],
                   "length_m": net.length[(u, v)], "speed_mps": net.speed[(u, v)],
                   "highway": meta.get((u, v), (0, 0, "", ""))[2],
                   "name": meta.get((u, v), (0, 0, "", ""))[3]}
                  for (u, v) in sorted(net.tau, key=lambda k: (str(k[0]), str(k[1])))],
        "axes": {str(v): {"A_deg": net.axes[v]["A_deg"], "B_deg": net.axes[v]["B_deg"],
                          "sep_deg": net.axes[v]["sep_deg"],
                          "leg": {str(w): t for w, t in net.axes[v]["leg"].items()}}
                 for v in net.axes},
        "arterial_axis": {str(v): art[v] for v in art},
        "excluded_nodes": [[str(v), why] for v, why in exlog],
        "link_time_histogram": [{"lo": lo, "hi": hi, "n": k}
                                for lo, hi, k in histogram(taus)],
    }
    json.dump(doc, open("real_net.json", "w"), indent=1)
    print(f"wrote real_net.json ({stats['nodes']} nodes, {stats['links']} links, "
          f"CV {stats['cv']:.3f}); {len(exlog)} node exclusions logged")
    print("link-time histogram (s):")
    for lo, hi, k in histogram(taus):
        print(f"  {lo:6.1f} - {hi:6.1f}  {'#' * min(k, 60)} ({k})")
    if not a.no_sumo:
        osm = f"{area}.osm"
        if not os.path.exists(osm):
            fetch_osm(bbox, osm)
        speed = f"{statistics.mean([net.speed[k] for k in net.tau]):.2f}"
        import sumo, subprocess
        exe = os.path.join(sumo.SUMO_HOME, "bin",
                           "netconvert" + (".exe" if os.name == "nt" else ""))
        args = [exe, "--osm-files", osm, "--junctions.join", "--tls.guess-signals",
                "--no-turnarounds", "--default.lanenumber", "2",
                "--default.speed", speed, "-o", "real.net.xml"]
        print("run:", " ".join(args[1:]))
        r = subprocess.run(args, capture_output=True, text=True)
        print((r.stdout or r.stderr)[-2000:])
        if r.returncode != 0:
            print("netconvert FAILED")
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
