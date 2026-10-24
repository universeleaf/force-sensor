"""Summarize paired robust-pass trajectories.

Reads results/runs/robust_<stamp>_<config>_s<seed>_<scenario>_<level>mm/frames.csv
(written by run_robust_trajectory.m) and writes per-trajectory metrics plus
seed-pooled tables. Restored rotation frames are excluded from every score.
No frame is removed for failing; failures are counted separately.

Usage: python summarize_robust.py OUTPUT_DIR STAMP [STAMP ...]
"""
import csv
import math
import pathlib
import re
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
RUNS = ROOT / "results" / "runs"
TAG = re.compile(r"^robust_(?P<stamp>.+?)_(?P<config>reference|qp[A-Za-z0-9_.]*)_s(?P<seed>\d+)_(?P<scenario>insertion|rotation|varying)_(?P<level>[0-9p]+)mm$")


def truth_modes():
    pub = ROOT / "results" / "published" / "fine_bvp_comparison_20261012_step5" / "reproduction"
    modes = {}
    for scenario in ("insertion", "rotation"):
        rows = list(csv.DictReader(open(pub / f"{scenario}_truth_frames.csv")))
        modes[scenario] = [r["mode"] for r in rows]
    return modes


def estimated_label(mode):
    mode = mode.strip()
    if mode in ("zeros(1,0)", "[]", "0", "[0]"):
        return "free"
    if mode in ("1", "[1]"):
        return "sticking"
    if mode in ("2", "[2]"):
        return "sliding"
    return "multi"


def metrics(path, scenario, modes):
    rows = list(csv.DictReader(open(path)))
    rows = [r for r in rows if r["restored"].lower() not in ("1", "true")]
    f = lambda key: np.array([float(r[key]) for r in rows])
    tip, contact, shape = f("tipErrorN"), f("contactErrorN"), f("shapeRmsMm")
    est = np.stack([f(k) for k in ("tipFx", "tipFy", "tipFz", "contactFx", "contactFy", "contactFz")], 1)
    true = np.stack([f(k) for k in ("trueTipFx", "trueTipFy", "trueTipFz", "trueContactFx", "trueContactFy", "trueContactFz")], 1)
    d = np.diff(est - true, axis=0)  # first-difference error = frame-to-frame jitter
    out = {
        "frames": len(rows),
        "tipRMSE_N": math.sqrt(np.mean(tip ** 2)),
        "contactRMSE_N": math.sqrt(np.mean(contact ** 2)),
        "shapeRMS_mm": math.sqrt(np.mean(shape ** 2)),
        "tipP95_N": float(np.percentile(tip, 95)),
        "contactP95_N": float(np.percentile(contact, 95)),
        "maxTip_N": float(tip.max()),
        "maxContact_N": float(contact.max()),
        "maxShape_mm": float(shape.max()),
        "tipJitter_N": math.sqrt(np.mean(np.sum(d[:, :3] ** 2, 1))),
        "contactJitter_N": math.sqrt(np.mean(np.sum(d[:, 3:] ** 2, 1))),
        "meanSeconds": float(f("seconds").mean()),
        "medianSeconds": float(np.median(f("seconds"))),
        "maxSeconds": float(f("seconds").max()),
        "bvpNotConverged": int(sum(r["bvpConverged"] not in ("1", "true", "True") for r in rows)),
        "bvpRejected": int(sum(r["bvpRejected"] in ("1", "true", "True") for r in rows)),
        "fbgAbove1p24": int(sum(float(r["fbgRms"]) > 1.24 for r in rows)),
        "penetrationOver0p1mm": int(sum(float(r["penetrationMm"]) > 0.1 for r in rows)),
        "maxTipMomentNmm": float(f("tipMomentNmm").max()),
        "failedFrames": int(sum(r.get("failed", "0") in ("1", "true", "True") for r in rows)),
    }
    if scenario in modes:
        frames = [int(r["frame"]) for r in rows]
        truthm = [modes[scenario][k - 1] for k in frames]
        estm = [estimated_label(r["mode"]) for r in rows]
        out["modeAccuracy"] = float(np.mean([a == b for a, b in zip(truthm, estm)]))
    else:
        out["modeAccuracy"] = float("nan")
    return out


def main():
    outdir = pathlib.Path(sys.argv[1])
    stamps = set(sys.argv[2:])
    outdir.mkdir(parents=True, exist_ok=True)
    modes = truth_modes()
    records = []
    for folder in sorted(RUNS.glob("robust_*")):
        m = TAG.match(folder.name)
        if not m or m["stamp"] not in stamps or not (folder / "result.mat").exists():
            continue
        rec = dict(m.groupdict())
        rec["seed"] = int(rec["seed"])
        rec["level"] = float(rec["level"].replace("p", "."))
        rec.update(metrics(folder / "frames.csv", rec["scenario"], modes))
        records.append(rec)
    if not records:
        sys.exit("no complete trajectories")
    keys = list(records[0].keys())
    with open(outdir / "trajectories.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(records)
    # Pool seeds: RMS metrics pooled as root-mean of squared per-trajectory RMS
    # (equal frames per trajectory), maxima as maxima, counts as sums.
    groups = {}
    for r in records:
        groups.setdefault((r["scenario"], r["level"], r["config"]), []).append(r)
    pooled = []
    for (scenario, level, config), rs in sorted(groups.items()):
        p = {"scenario": scenario, "level": level, "config": config, "seeds": " ".join(str(r["seed"]) for r in sorted(rs, key=lambda r: r["seed"])), "trajectories": len(rs)}
        for k in ("tipRMSE_N", "contactRMSE_N", "shapeRMS_mm", "tipJitter_N", "contactJitter_N"):
            p[k] = math.sqrt(np.mean([r[k] ** 2 for r in rs]))
        for k in ("tipP95_N", "contactP95_N", "meanSeconds", "medianSeconds", "modeAccuracy"):
            p[k] = float(np.mean([r[k] for r in rs]))
        for k in ("maxTip_N", "maxContact_N", "maxShape_mm", "maxSeconds", "maxTipMomentNmm"):
            p[k] = max(r[k] for r in rs)
        for k in ("bvpNotConverged", "bvpRejected", "fbgAbove1p24", "penetrationOver0p1mm", "failedFrames", "frames"):
            p[k] = sum(r[k] for r in rs)
        pooled.append(p)
    with open(outdir / "pooled.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, list(pooled[0].keys()))
        w.writeheader()
        w.writerows(pooled)
    for p in pooled:
        print(f"{p['scenario']:9s} {p['level']:3.1f} {p['config']:24s} n={p['trajectories']} tip {p['tipRMSE_N']:.3f} contact {p['contactRMSE_N']:.3f} shape {p['shapeRMS_mm']:.3f} "
              f"maxTip {p['maxTip_N']:.2f} maxC {p['maxContact_N']:.2f} jitT {p['tipJitter_N']:.3f} jitC {p['contactJitter_N']:.3f} mode {p['modeAccuracy']:.3f} "
              f"s {p['meanSeconds']:.3f} bvpNC {p['bvpNotConverged']} rej {p['bvpRejected']} fbg {p['fbgAbove1p24']} fail {p['failedFrames']}")


if __name__ == "__main__":
    main()
