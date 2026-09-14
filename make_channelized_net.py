"""Channelized-approach variant of the grid: four lanes per edge with the kerb
lane right-turn only, two through lanes, and the median lane left-turn only,
so that turning vehicles never queue behind through vehicles (the lane layout
of channelized urban intersections, and the layout under which the paper's
model assumption w_R = 0 holds under congestion). Writes grid8_rt.net.xml.

Steps: netgenerate the 8x8 grid with four lanes; list every connection of the
four-lane net with sumolib; write a connection file that deletes all of them
and re-creates the channelized pattern (lane 0 -> right, lanes 1 and 2 ->
through, lane 3 -> left); netconvert with that file; verify the pattern on
every internal edge. Usage: python make_channelized_net.py"""
import os, subprocess, sys
import sumo, sumolib

SH = sumo.SUMO_HOME
EXE = ".exe" if os.name == "nt" else ""
NG = os.path.join(SH, "bin", "netgenerate" + EXE)
NC = os.path.join(SH, "bin", "netconvert" + EXE)
BASE, OUT, CON = "grid8_4lane.net.xml", "grid8_rt.net.xml", "channelized.con.xml"
PATTERN = {"r": (0,), "s": (1, 2), "l": (3,)}

subprocess.run([NG, "--grid", "--grid.number", "8", "--grid.length", "500", "--grid.attach-length", "500",
                "--default.lanenumber", "4", "--default.speed", "8.33", "--default-junction-type", "traffic_light",
                "--no-turnarounds", "-o", BASE], check=True)
net = sumolib.net.readNet(BASE)
lines = ["<connections>"]
for e in net.getEdges():
    for out, conns in e.getOutgoing().items():
        d = conns[0].getDirection()
        for c in conns:
            lines.append(f'  <delete from="{e.getID()}" to="{out.getID()}" fromLane="{c.getFromLane().getIndex()}" toLane="{c.getToLane().getIndex()}"/>')
        if d not in PATTERN:
            continue
        for k, fl in enumerate(PATTERN[d]):
            tl = PATTERN["s"][k] if d == "s" else (0 if d == "r" else 3)
            lines.append(f'  <connection from="{e.getID()}" to="{out.getID()}" fromLane="{fl}" toLane="{tl}"/>')
lines.append("</connections>")
open(CON, "w").write("\n".join(lines) + "\n")
subprocess.run([NC, "-s", BASE, "--connection-files", CON, "--no-turnarounds", "-o", OUT], check=True)
# verify
net = sumolib.net.readNet(OUT)
bad = 0
for e in net.getEdges():
    if not (e.getToNode().getType() == "traffic_light"):
        continue
    got = {}
    for out, conns in e.getOutgoing().items():
        for c in conns:
            got.setdefault(c.getDirection(), set()).add(c.getFromLane().getIndex())
    want = {d: set(l) for d, l in PATTERN.items() if d in got}
    if got != want:
        bad += 1
        print("pattern mismatch", e.getID(), got)
print(f"{OUT}: {len(net.getEdges())} edges, {bad} edges deviating from the channelized pattern")
sys.exit(1 if bad else 0)
