"""Markdown tables for the robust-pass report from summarize_*.py outputs.

Usage: python make_report_tables.py SUMMARY_DIR CONFIG [CONFIG ...]
SUMMARY_DIR must contain pooled.csv (summarize_robust.py); shadow_summary.csv
and named_frames.csv are rendered when present.
"""
import csv
import pathlib
import sys


def fmt(x, digits=3):
    x = float(x)
    return f"{x:.{digits}g}" if abs(x) < 1e-3 or abs(x) >= 1e4 else f"{x:.{digits}f}".rstrip("0").rstrip(".") if digits else str(x)


def main():
    d = pathlib.Path(sys.argv[1])
    configs = sys.argv[2:]
    rows = list(csv.DictReader(open(d / "pooled.csv")))
    order = {c: i for i, c in enumerate(configs)}
    rows = [r for r in rows if r["config"] in order]
    rows.sort(key=lambda r: (r["scenario"], float(r["level"]), order[r["config"]]))
    print("| Scenario | Noise mm | Config | Seeds | Tip RMSE N | Contact RMSE N | Shape RMS mm | Max tip N | Max contact N | Tip jitter N | Contact jitter N | Mode acc. | Mean s/frame | BVP not conv. | BVP rejected | FBG>1.24 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        mode = "-" if r["modeAccuracy"] in ("nan", "") else f"{float(r['modeAccuracy']):.2f}"
        print(f"| {r['scenario']} | {float(r['level']):g} | {r['config']} | {r['seeds']} | {float(r['tipRMSE_N']):.3f} | {float(r['contactRMSE_N']):.3f} | {float(r['shapeRMS_mm']):.3f} | "
              f"{float(r['maxTip_N']):.2f} | {float(r['maxContact_N']):.2f} | {float(r['tipJitter_N']):.3f} | {float(r['contactJitter_N']):.3f} | {mode} | {float(r['meanSeconds']):.3f} | "
              f"{r['bvpNotConverged']} | {r['bvpRejected']} | {r['fbgAbove1p24']} |")
    shadow = d / "shadow_summary.csv"
    if shadow.exists():
        print()
        print("| Scenario | Noise mm | Frames | Branches | QP lower objective | QP higher objective | SQP at 100-iteration cap | SQP infeasible, QP feasible | Same selected mode | Frames with lower selected objective (QP) | Max selected force difference N | SQP branch s | QP branch s |")
        print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for r in csv.DictReader(open(shadow)):
            print(f"| {r['scenario']} | {r['noise_mm']} | {r['frames']} | {r['branches']} | {r['qpBetterBranches']} | {r['qpWorseBranches']} | {r['sqpAtIterationCap']} | {r['sqpInfeasibleQpFeasible']} | "
                  f"{r['sameSelectedMode']} | {r['framesQpLowerSelectedObjective']} | {float(r['maxSelectedForceDifferenceN']):.2f} | {float(r['sqpBranchSeconds']):.1f} | {float(r['qpBranchSeconds']):.1f} |")
    named = d / "named_frames.csv"
    if named.exists():
        print()
        print("| Case | Frame | Config | Mode | Tip err N | Contact err N | Shape RMS mm | FBG RMS (pre-BVP) | Terminal moment N mm | BVP conv./rejected | SQP-QP selected objective |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
        for r in csv.DictReader(open(named)):
            print(f"| {r['scenario']} {r['noise_mm']} mm | {r['frame']} | {r['config']} | {r['mode']} | {float(r['tipErrorN']):.3f} | {float(r['contactErrorN']):.3f} | {float(r['shapeRmsMm']):.3f} | "
                  f"{float(r['fbgRms']):.2f} ({float(r['preBvpFbgRms']):.2f}) | {float(r['tipMomentNmm']):.3g} | {r['bvpConverged']}/{r['bvpRejected']} | {float(r['shadowSqpMinusQpObjective']):.3g} |")


if __name__ == "__main__":
    main()
