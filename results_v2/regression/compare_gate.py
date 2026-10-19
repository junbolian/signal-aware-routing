"""Task 0.4: compare the network-0 regression runs with sumo_results_depth.
Run from the repository root: python results_v2/regression/compare_gate.py"""
import json

NEW, OLD = "results_v2/regression", "sumo_results_depth"
lines = ["Regression gate (Task 0.4): SR_NETSEED=0, seed 11, demand 1/15, decision 45 m, grid8.net.xml",
         f"new: {NEW}/dem15_<METHOD>_n0_11.json   reference: {OLD}/dem15_<METHOD>_11.json", ""]
ok_all = True
for m in ("STATIC", "LA1", "TDOPT_replan", "TDOPTQ", "LA2"):
    new = json.load(open(f"{NEW}/dem15_{m}_n0_11.json"))
    old = json.load(open(f"{OLD}/dem15_{m}_11.json"))
    dnew = json.load(open(f"{NEW}/diag_dem15_{m}_n0_11.json"))
    dold = json.load(open(f"{OLD}/diag_dem15_{m}_11.json"))
    same = new == old
    same_diag = dnew == dold
    ok_all &= same
    lines.append(f"{m:13s} new {new}  reference {old}  travel times identical: {'YES' if same else 'NO'}"
                 f"  per-crossing diag records identical: {'YES' if same_diag else 'NO'}")
lines += ["", "GATE " + ("PASSED: all 20 trips identical" if ok_all else "FAILED")]
open(f"{NEW}/REGRESSION.txt", "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
