"""Verify the numbers of Section 4.2 (and its tables) of main.tex against the
result files of the experiment.

Usage:
  python check_numbers.py main.tex sumo_results/congestion_summary.json sumo_results_rt/congestion_summary.json \
         [sumo_results/discharge_calibration.json sumo_results_rt/discharge_calibration.json]

What is checked
  A. Tables 3 to 5 of the manuscript, cell by cell, against the JSON (means and
     per-instance intervals; paired differences with seed-clustered intervals and
     wins; right-turn diagnostic n, mean count ahead, error, sd).
  B. Every "x +- y" pair in the prose of Section 4.2 must occur, at the printed
     precision, among the (mean, interval) pairs of the JSON files (per-instance
     or clustered interval), the diagnostic (error, sd) pairs, or the empty-net
     deviations.
  C. Every "a of 40" / "(a)" wins count quoted in Section 4.2 must occur among the
     wins counts of the JSON files; the value-calibration numbers quoted in the
     prose must match section E.
  D. Calibration constants quoted in the prose (saturation headway, residual,
     crossings, onsets) against discharge_calibration.json when given.
Prints every mismatch; exit code 1 if any."""
import json, re, sys

tex = open(sys.argv[1], encoding="utf-8").read()
base = json.load(open(sys.argv[2]))["levels"]
rt = json.load(open(sys.argv[3]))["levels"]
cal = [json.load(open(p)) for p in sys.argv[4:6]] if len(sys.argv) > 5 else None

NAMES = {"SP-STATIC": "STATIC", "GREEDY": "GREEDY", "LA-$1$": "LA1", "MDP": "MDP", "TD-OPT open-loop": "TDOPT_open",
         "TD-OPT replan": "TDOPT_replan", "TD-OPT": "TDOPT_replan", "SP-STATIC-Q": "STATICQ", "MDP-Q": "MDPQ", "TD-OPT-Q": "TDOPTQ"}
LEVELS = [("light", base), ("moderate", base), ("heavy", base), ("moderate", rt)]
bad = []

def r1(x):
    """Same rounding as the aggregator's printed tables (Python's format at one decimal)."""
    return float(f"{float(x):.1f}")

def sec42():
    i = tex.index("\\subsection{Microscopic study")
    j = tex.index("\\section{Conclusions}")
    return tex[i:j]

S = sec42()

# ---------------------------------------------------------------- A. tables
def table(label):
    i = tex.index("\\label{" + label + "}")
    j = tex.rfind("\\begin{table}", 0, i)
    return tex[j:i]

# Table of means
rows = re.findall(r"^(.*?) & (.*?) \\\\$", table("tab:sumo"), re.M)
for lab, rest in rows:
    if lab not in NAMES:
        continue
    key = NAMES[lab]
    cells = [c.strip() for c in ("& " + rest).split("&")[1:]]
    for (lev, src), cell in zip(LEVELS, cells):
        if cell == "--":
            continue
        m = re.match(r"\$([\-\+\d\.]+) \\pm ([\d\.]+)\$", cell)
        mean, ci = float(m.group(1)), float(m.group(2))
        got = src.get(lev, {}).get("means", {}).get(key)
        if got is None or r1(got["mean"]) != r1(mean) or r1(got["ci"]) != r1(ci):
            bad.append(f"Table means: {lab} {lev} {'rt' if src is rt else 'base'}: tex {mean} +- {ci}, json {got and (r1(got['mean']), r1(got['ci']))}")

# Table of paired differences
rows = re.findall(r"^(.*?) \$-\$ (.*?) & (.*?) \\\\$", table("tab:paired"), re.M)
for a, b, rest in rows:
    ka, kb = NAMES.get(a.strip()), NAMES.get(b.strip())
    key = f"{ka}-{kb}"
    cells = [c.strip() for c in ("& " + rest).split("&")[1:]]
    for (lev, src), cell in zip(LEVELS, cells):
        m = re.match(r"\$([\-\+\d\.]+) \\pm ([\d\.]+)\$ \(\$?(\d+)\$?\)", cell)
        if not m:
            bad.append(f"Table paired: cannot parse cell '{cell}'"); continue
        diff, ci, wins = float(m.group(1)), float(m.group(2)), int(m.group(3))
        got = src.get(lev, {}).get("paired", {}).get(key)
        if got is None:
            bad.append(f"Table paired: {key} {lev}: missing in json"); continue
        if r1(got["mean"]) != r1(diff) or r1(got["ci_clustered"]) != r1(ci) or got["wins"] != wins:
            bad.append(f"Table paired: {key} {lev} {'rt' if src is rt else 'base'}: tex {diff} +- {ci} ({wins}), json {r1(got['mean'])} +- {r1(got['ci_clustered'])} ({got['wins']})")

# Table of right-turn diagnostics
rows = re.findall(r"^(.*?) & (.*?) \\\\$", table("tab:diag"), re.M)
for lab, rest in rows:
    if lab not in NAMES:
        continue
    key = NAMES[lab] + "/R"
    cells = [c.strip() for c in ("& " + rest).split("&")[1:]]
    for (src, cs) in ((base, cells[:3]), (rt, cells[3:])):
        if cs[0] == "--":
            continue
        n = int(cs[0].strip("$")); q = float(cs[1].strip("$"))
        m = re.match(r"\$([\-\+\d\.]+)\$ \(\$([\d\.]+)\$\)", cs[2])
        err, sd = float(m.group(1)), float(m.group(2))
        got = src.get("moderate", {}).get("diagnostic", {}).get(key)
        if got is None or got["n"] != n or round(got["q_ahead"], 2) != q or r1(got["error"]) != r1(err) or r1(got["sd"]) != r1(sd):
            bad.append(f"Table diag: {lab} {'rt' if src is rt else 'base'}: tex n={n} q={q} {err} ({sd}), json {got and (got['n'], round(got['q_ahead'],2), r1(got['error']), r1(got['sd']))}")

# ------------------------------------------------- B. prose: x +- y pairs
pairs = set(); wins = set(); errs = set(); calib_pairs = set()
for src in (base, rt):
    for lev, d in src.items():
        for k, v in d.get("means", {}).items():
            pairs.add((r1(v["mean"]), r1(v["ci"])))
        for k, v in d.get("paired", {}).items():
            pairs.add((r1(v["mean"]), r1(v["ci"]))); pairs.add((r1(v["mean"]), r1(v["ci_clustered"])))
            pairs.add((r1(-v["mean"]), r1(v["ci"]))); pairs.add((r1(-v["mean"]), r1(v["ci_clustered"])))
            wins.add(v["wins"]); wins.add(v["n"] - v["wins"])
        for k, v in d.get("diagnostic", {}).items():
            pairs.add((r1(v["error"]), r1(v["sd"]))); errs.add(r1(v["error"])); errs.add(r1(v["sd"]))
        for k, rows_ in d.get("value_calibration", {}).items():
            for rec in rows_:
                calib_pairs.add((round(rec["planned"]), round(rec["realised"])))
emp = json.load(open(sys.argv[2])).get("empty", {})
for k, v in emp.items():
    if isinstance(v, dict) and "dev_pct" in v:
        pairs.add((r1(v["dev_pct"]), r1(v["dev_ci"]))); pairs.add((r1(-0.0), r1(v["dev_ci"])))
prose = re.sub(r"\\begin\{table\}.*?\\end\{table\}", "", S, flags=re.S)
for a, b in re.findall(r"\$([\-\+]?\d+\.\d)\s*\\pm\s*(\d+\.\d)\$", prose):
    if (float(a), float(b)) not in pairs and (abs(float(a)), float(b)) not in pairs:
        bad.append(f"Prose pair not found in json: {a} +- {b}")
for a in re.findall(r"\$(\d+)\$ (?:of|and) \$?40\$?", prose) + re.findall(r"in \$(\d+)\$ of \$40\$", prose):
    if int(a) not in wins:
        bad.append(f"Prose wins count not found: {a}")
for m in re.finditer(r"(\d{3,4})\$ s and realizes \$(\d{3,4})", prose):
    if (int(m.group(1)), int(m.group(2))) not in calib_pairs:
        bad.append(f"Value calibration pair not found: {m.group(1)}/{m.group(2)}")

# ------------------------------------------------- D. calibration constants
if cal:
    for c, tag in zip(cal, ("base", "rt")):
        hs, ls, rm, nc, no = c["HS"], c["LS"], c["rmse"], c["n_crossings"], c["n_onsets"]
        if tag == "base":
            for s in (f"${round(hs,2)}$", f"{nc:,}".replace(",", "{,}"), f"{no:,}".replace(",", "{,}"), f"${round(rm,2)}$"):
                if s not in S:
                    bad.append(f"Calibration constant not quoted as in file ({tag}): {s}")
        else:
            if f"${round(hs,2)}$" not in S:
                bad.append(f"Calibration constant not quoted as in file ({tag}): {round(hs,2)}")

print(f"{len(bad)} mismatch(es)")
for b_ in bad:
    print("  ", b_)
sys.exit(1 if bad else 0)
