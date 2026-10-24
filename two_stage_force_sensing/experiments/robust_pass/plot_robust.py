"""Force-trajectory and error plots for paired robust-pass runs.

Usage: python plot_robust.py OUTPUT_DIR STAMP SEED LEVEL CONFIG [CONFIG ...]
Plots tip Fx and contact Fx/Fz (estimate vs reference) for each scenario and
the per-frame vector errors. Restored rotation frames are omitted.
"""
import csv
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
RUNS = ROOT / "results" / "runs"
COLORS = ["#4C72B0", "#DD8452", "#55A868", "#C44E52", "#8172B3", "#937860"]


def load(stamp, config, seed, scenario, level):
    path = RUNS / f"robust_{stamp}_{config}_s{seed}_{scenario}_{level}mm" / "frames.csv"
    if not path.exists():
        return None
    rows = [r for r in csv.DictReader(open(path)) if r["restored"].lower() not in ("1", "true")]
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k not in ("mode", "flags", "restored", "failure", "failed")}


def main():
    outdir, stamp, seed, level = pathlib.Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]), sys.argv[4].replace(".", "p")
    configs = sys.argv[5:]
    outdir.mkdir(parents=True, exist_ok=True)
    for scenario in ("insertion", "rotation", "varying"):
        data = {c: load(stamp, c, seed, scenario, level) for c in configs}
        data = {c: d for c, d in data.items() if d is not None}
        if not data:
            continue
        first = next(iter(data.values()))
        x = first["frame"]
        fig, ax = plt.subplots(5, 1, figsize=(10, 13), sharex=True)
        for key, a, label in (("TipFx", ax[0], "tip Fx (N)"), ("ContactFx", ax[1], "contact Fx (N)"), ("ContactFz", ax[2], "contact Fz (N)")):
            a.plot(x, first["true" + key], color="k", lw=2.2, label="truth")
            for i, (c, d) in enumerate(data.items()):
                a.plot(d["frame"], d[key[0].lower() + key[1:]], color=COLORS[i % len(COLORS)], lw=1, label=c)
            a.set_ylabel(label)
            a.grid(alpha=0.3)
        ax[0].legend(fontsize=8, ncol=3)
        for i, (c, d) in enumerate(data.items()):
            ax[3].plot(d["frame"], d["tipErrorN"], color=COLORS[i % len(COLORS)], lw=1, label=c)
            ax[4].plot(d["frame"], d["contactErrorN"], color=COLORS[i % len(COLORS)], lw=1, label=c)
        ax[3].set_ylabel("|tip error| (N)")
        ax[4].set_ylabel("|contact error| (N)")
        ax[4].set_xlabel("frame")
        for a in ax[3:]:
            a.set_yscale("log")
            a.grid(alpha=0.3, which="both")
        fig.suptitle(f"{scenario}, {sys.argv[4]} mm noise, seed {seed}")
        fig.tight_layout()
        fig.savefig(outdir / f"{scenario}_{level}mm_s{seed}.png", dpi=110)
        plt.close(fig)
        print("wrote", outdir / f"{scenario}_{level}mm_s{seed}.png")


if __name__ == "__main__":
    main()
