"""Branch-level fmincon-SQP vs profiled-QP comparison on identical inputs.

Reads results/runs/shadow_<tag>/{branches,frames}.csv written by
compare_branch_solvers.m and writes OUTPUT_DIR/shadow_summary.csv.
A branch counts as better/worse only beyond a relative objective tolerance
of 1e-6 (or when only one solver returned an affine-feasible point).
Usage: python summarize_shadow.py OUTPUT_DIR
"""
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for d in sorted((ROOT / "results" / "runs").glob("shadow_s0_*")):
        if not (d / "frames.csv").exists():
            continue
        B = list(csv.DictReader(open(d / "branches.csv")))
        F = list(csv.DictReader(open(d / "frames.csv")))
        f = lambda r, k: float(r[k])
        tol = lambda r: 1e-6 * max(1.0, abs(f(r, "sqpObjective")))
        better = [r for r in B if r["qpFeasible"] == "1" and (r["sqpFeasible"] != "1" or f(r, "qpMinusSqp") < -tol(r))]
        worse = [r for r in B if r["sqpFeasible"] == "1" and (r["qpFeasible"] != "1" or f(r, "qpMinusSqp") > tol(r))]
        sel = [r for r in F if f(r, "qpObjective") < f(r, "sqpObjective") - 1e-6 * max(1.0, abs(f(r, "sqpObjective")))]
        _, scenario, level = d.name.rsplit("_", 2)
        rows.append({
            "scenario": d.name.split("_")[2], "noise_mm": level.replace("mm", "").replace("p", "."), "frames": len(F), "branches": len(B),
            "qpBetterBranches": len(better), "qpWorseBranches": len(worse),
            "sqpAtIterationCap": sum(r["sqpAtCap"] == "1" for r in B),
            "sqpInfeasibleQpFeasible": sum(r["sqpFeasible"] != "1" and r["qpFeasible"] == "1" for r in B),
            "sameSelectedMode": sum(r["sameMode"] == "1" for r in F),
            "framesQpLowerSelectedObjective": len(sel),
            "maxSelectedForceDifferenceN": max(f(r, "forceDifferenceN") for r in F),
            "certifiedFrames": sum(r["qpCertificate"] == "1" for r in F),
            "sqpBranchSeconds": sum(f(r, "sqpBranchSeconds") for r in F),
            "qpBranchSeconds": sum(f(r, "qpBranchSeconds") for r in F),
        })
    with open(out / "shadow_summary.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
