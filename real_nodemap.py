"""Node-id map between the three identifier spaces of the real-topology study,
and a hand-checkable dump of the SUMO-to-analytical movement mapping.

  OSM node id          the key used in real_net.json ("nodes", "links")
  analytical id        the same value; real_net.json keeps OSM ids verbatim
  SUMO junction id     what netconvert produced, which is the OSM id for an
                       unjoined node and cluster_<id>_<id>... when
                       --junctions.join merged several OSM nodes into one

The map is built by id first and by nearest coordinate second (netconvert
renames joined clusters, and the cluster centroid moves), with the match
distance reported so a bad match is visible rather than silent.

Usage:
  python real_nodemap.py                 # write real_nodemap.json
  python real_nodemap.py --verify 5      # dump N intersections for hand-checking
"""
import argparse
import json
import math
import random
import sys

import sumolib

import general_net as gn


def load_analytical(path="real_net.json"):
    doc = json.load(open(path))
    nodes = {k: tuple(v) for k, v in doc["nodes"].items()}
    links = {(l["u"], l["v"]): (l["length_m"], l["speed_mps"]) for l in doc["links"]}
    net = gn.GNet(nodes, links)
    return doc, net


def build_map(doc, snet):
    """analytical/OSM id -> (sumo junction id, match kind, distance in metres)."""
    bbox = doc["bbox_north_south_east_west"]
    north, south, east, west = bbox
    lat0 = (north + south) / 2.0
    mx = 111320.0 * math.cos(math.radians(lat0))
    my = 110540.0

    # SUMO junction coordinates are in the net's own metric frame; convert them
    # to the same lon/lat-derived frame real_net.json uses.
    sj = []
    for j in snet.getNodes():
        x, y = j.getCoord()
        lon, lat = snet.convertXY2LonLat(x, y)
        sj.append((j.getID(), (lon - west) * mx, (lat - south) * my))
    by_id = {jid: (jx, jy) for jid, jx, jy in sj}

    out = {}
    for nid, (nx, ny) in doc["nodes"].items():
        if nid in by_id:
            jx, jy = by_id[nid]
            out[nid] = (nid, "exact id", math.hypot(jx - nx, jy - ny))
            continue
        best = None
        for jid, jx, jy in sj:
            d = math.hypot(jx - nx, jy - ny)
            if best is None or d < best[2]:
                best = (jid, "nearest" + (" (cluster)" if jid.startswith("cluster") else ""), d)
        out[nid] = best
    return out


def verify(doc, net, snet, nmap, k, seed=777):
    """Print, for k random intersections, the SUMO controlled links beside the
    analytical (axis, movement) pair each one maps to, for hand-checking."""
    rng = random.Random(seed)
    cand = [v for v in net.axes if not nmap[v][0].startswith("cluster") or True]
    picks = rng.sample(sorted(cand), min(k, len(cand)))
    tls = {t.getID(): t for t in snet.getTrafficLights()}
    for v in picks:
        jid, kind, dist = nmap[v]
        ax = net.axes[v]
        print(f"\n=== analytical node {v}  ->  SUMO junction {jid} "
              f"[{kind}, {dist:.1f} m] ===")
        print(f"    axis A {ax['A_deg']:.1f} deg, axis B {ax['B_deg']:.1f} deg, "
              f"separation {ax['sep_deg']:.1f} deg")
        print("    analytical movements from each approach:")
        for u, _ in [(u, 0) for (u, w) in net.tau if w == v]:
            h = net._heading[(u, v)]
            a = net.axis(v, h)[1]
            row = []
            for w, d in net.nbrs(v):
                m = net.move_type(v, h, d)
                if m:
                    row.append(f"{w}:{m}")
            print(f"      from {u} (axis {a}): {', '.join(row) if row else '(none)'}")
        if jid in tls:
            links = tls[jid].getConnections()
            print(f"    SUMO controlled connections: {len(links)}")
            for i, (inL, outL, _idx) in enumerate(links[:12]):
                print(f"      {i:2d} {inL.getID()} -> {outL.getID()}")
            if len(links) > 12:
                print(f"      ... {len(links) - 12} more")
        else:
            print("    SUMO junction carries no traffic light")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", default="real.net.xml")
    ap.add_argument("--verify", type=int, default=0)
    a = ap.parse_args()
    doc, net = load_analytical()
    snet = sumolib.net.readNet(a.net)
    nmap = build_map(doc, snet)
    d = [v[2] for v in nmap.values()]
    exact = sum(1 for v in nmap.values() if v[1] == "exact id")
    print(f"mapped {len(nmap)} analytical nodes: {exact} by exact id, "
          f"{len(nmap) - exact} by nearest coordinate")
    print(f"match distance: median {sorted(d)[len(d) // 2]:.1f} m, max {max(d):.1f} m")
    far = [(k, v) for k, v in nmap.items() if v[2] > 30.0]
    if far:
        print(f"WARNING {len(far)} nodes matched more than 30 m away:")
        for k, v in far[:10]:
            print(f"   {k} -> {v[0]} ({v[2]:.1f} m)")
    json.dump({k: {"sumo": v[0], "match": v[1], "dist_m": v[2]}
               for k, v in nmap.items()}, open("real_nodemap.json", "w"), indent=1)
    print("wrote real_nodemap.json")
    if a.verify:
        verify(doc, net, snet, nmap, a.verify)
    return 0


if __name__ == "__main__":
    sys.exit(main())
