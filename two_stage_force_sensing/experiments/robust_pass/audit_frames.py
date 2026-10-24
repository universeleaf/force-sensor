"""Frames named in the 2026-10-13 handoff, compared across configurations.

Usage: python audit_frames.py OUTPUT_DIR STAMP CONFIG [CONFIG ...]
Uses seed 0 (the published noise payload). Shadow columns report whether the
fmincon branch solve was suboptimal at that frame on identical inputs.
"""
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
RUNS = ROOT / "results" / "runs"
FRAMES = [("insertion", "0", 99), ("insertion", "0", 102), ("rotation", "0p5", 91), ("insertion", "2", 173), ("rotation", "2", 72), ("rotation", "2", 78)]


def frame_row(path, k):
    if not path.exists():
        return None
    for r in csv.DictReader(open(path)):
        if int(r["frame"]) == k:
            return r
    return None


def main():
    out, stamp, configs = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3:]
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario, level, k in FRAMES:
        shadow = frame_row(RUNS / f"shadow_s0_{scenario}_{level}mm" / "frames.csv", k)
        for c in configs:
            r = frame_row(RUNS / f"robust_{stamp}_{c}_s0_{scenario}_{level}mm" / "frames.csv", k)
            if r is None:
                continue
            rows.append({
                "scenario": scenario, "noise_mm": level.replace("p", "."), "frame": k, "config": c, "mode": r["mode"],
                "tipErrorN": float(r["tipErrorN"]), "contactErrorN": float(r["contactErrorN"]), "shapeRmsMm": float(r["shapeRmsMm"]),
                "fbgRms": float(r["fbgRms"]), "preBvpFbgRms": float(r["preBvpFbgRms"]), "tipMomentNmm": float(r["tipMomentNmm"]),
                "bvpConverged": r["bvpConverged"], "bvpRejected": r["bvpRejected"],
                "shadowSqpMode": shadow["sqpMode"] if shadow else "", "shadowQpMode": shadow["qpMode"] if shadow else "",
                "shadowSqpMinusQpObjective": float(shadow["sqpObjective"]) - float(shadow["qpObjective"]) if shadow else float("nan"),
            })
    with open(out / "named_frames.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['scenario']:9s} {r['noise_mm']:3s} f{r['frame']:<4d} {r['config']:22s} mode {r['mode']:10s} tip {r['tipErrorN']:7.3f} contact {r['contactErrorN']:7.3f} shape {r['shapeRmsMm']:7.3f} fbg {r['fbgRms']:6.2f} pre {r['preBvpFbgRms']:6.2f} M {r['tipMomentNmm']:8.2g} bvp {r['bvpConverged']}/{r['bvpRejected']} shadow dObj {r['shadowSqpMinusQpObjective']:.3g}")


if __name__ == "__main__":
    main()
