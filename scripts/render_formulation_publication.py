"""Verified publication figures from paired, completed full-window ledgers.

The diagram is a conceptual illustration. Quantitative figures use archived
solver outputs only. Three seed draws are not three independent trajectories.
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np

from render_formulation_factors import ROOT, LABELS, sha, save, render, nonpositive_exit

METHODS = ["full", "point", "gaussian"]
COLORS = {"full": "#0072B2", "point": "#D55E00", "gaussian": "#009E73"}
SCENES = {"three_contact": "Three body contacts", "spatial_sliding": "Spatial frictional sliding"}


def style() -> None:
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "pdf.fonttype": 42, "svg.fonttype": "none", "axes.linewidth": .6})


def method_diagram(folder: Path) -> list[Path]:
    """Semantic factor structure, never presented as a solved simulation."""
    style()
    folder.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(3.45, 3.55))
    ax.set(xlim=(0, 10), ylim=(0, 10))
    ax.axis("off")

    def box(x, y, w, h, title, body, color):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=.08,rounding_size=.1",
                                   linewidth=.7, edgecolor=color, facecolor="#F6F8FB"))
        ax.text(x + w / 2, y + h - .17, title, ha="center", va="top", fontsize=7.5, weight="bold", color=color)
        ax.text(x + w / 2, y + h / 2 - .17, body, ha="center", va="center", fontsize=7, linespacing=1.35)

    def arrow(x1, y1, x2, y2, color="#526173"):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=9,
                                    color=color, linewidth=.85))

    box(.15, 7.8, 4.35, 1.8, "Measured shape", "Bending channels\nCovariance + base pose", "#0072B2")
    box(5.5, 7.8, 4.35, 1.8, "Observed environment", "Plane points / normals\nCovariance + friction", "#009E73")
    box(.15, 4.8, 4.35, 1.8, "Previous state", "Cosserat equilibrium\nLatent contacts + tip", "#0072B2")
    box(5.5, 4.8, 4.35, 1.8, "Current state", "Cosserat equilibrium\nLatent contacts + tip", "#0072B2")
    box(.8, 1.85, 8.4, 1.9, "Joint constrained MAP", "Shape / environment likelihoods\nContact and friction complementarity\nInitial + temporal state priors", "#6E4C93")
    ax.text(5, 7.08, "All-time shape / environment likelihoods", ha="center", fontsize=6.7, color="#526173",
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 1})
    # Both observed tensors constrain every time; the horizontal bus avoids
    # implying that shape measures only the past or geometry only the present.
    ax.plot([2.3, 7.7], [7.45, 7.45], color="#526173", linewidth=.85)
    ax.plot([2.3, 7.7], [7.45, 7.45], "o", color="#526173", markersize=2)
    for x in (2.3, 7.7):
        arrow(x, 7.72, x, 6.7)
        arrow(x, 4.69, x, 3.86)
    arrow(4.59, 5.7, 5.4, 5.7)
    ax.text(5, 4.24, "Friction: same material coordinate $s_j$", ha="center", fontsize=6.7, color="#526173")
    arrow(5, 1.72, 5, 1.2)
    ax.text(5, .97, "Contact arcs • force vectors • tip force", ha="center", weight="bold", fontsize=7.3)
    ax.text(5, .58, "Physical audit • local uncertainty • ODE counts", ha="center", fontsize=6.7, color="#526173")
    ax.text(5, .16, "Factor structure illustration", ha="center", color="#687587", fontsize=6.2)
    fig.subplots_adjust(left=.02, right=.99, bottom=.02, top=.98)
    return save(fig, folder, "method_overview")


def load_report(folder: Path) -> dict:
    report = json.loads((folder / "comparison.json").read_text(encoding="utf-8"))
    if report["state"] != "complete" or report["failureCount"]:
        raise ValueError(f"Incomplete or failed ledger: {folder}")
    if not isinstance(report["cases"], list):
        report["cases"] = [report["cases"]]
    return report


def publication(folder: Path) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    factor_folder = ROOT / "out/benchmarks/formulation_factors"
    base_folder = ROOT / "out/benchmarks/formulation_literature"
    # Both checks verify all method artifacts and the full Cartesian matrix.
    render(factor_folder)
    render(base_folder)
    factors, baselines = load_report(factor_folder), load_report(base_folder)
    selected = [c for c in factors["cases"] if c["method"] == "full"] + baselines["cases"]
    paired = defaultdict(dict)
    for case in selected:
        key = (case["sceneId"], case["seed"], case["noiseStdPerMm"])
        if case["method"] in paired[key]:
            raise ValueError(f"Duplicate paired method: {case['id']}")
        paired[key][case["method"]] = case
    for key, packet in paired.items():
        if set(packet) != set(METHODS):
            raise ValueError(f"Unpaired methods: {key}")
        for field in ("inputSha256", "truthSha256"):
            if len({c[field] for c in packet.values()}) != 1:
                raise ValueError(f"Different {field} in comparison: {key}")
    rows, groups = [], defaultdict(list)
    for case in selected:
        m = case["metrics"]
        row = {"scene": case["sceneId"], "method": case["method"], "seed": case["seed"],
               "noiseStdPerMm": case["noiseStdPerMm"], "contactRmseN": m["contactForceRmseN"],
               "tipRmseN": m["tipForceRmseN"], "totalRmseN": m["totalForceRmseN"],
               "arcRmseMm": m["contactArcRmseMm"], "magnitudeMaeN": m["contactMagnitudeMaeN"],
               "reviewFrames": m["reviewCount"], "solverExitflag": case["solver"].get("exitflag"),
               "wallSeconds": case["wallSeconds"], "inputSha256": case["inputSha256"],
               "truthSha256": case["truthSha256"], "estimateSha256": case["estimateSha256"]}
        rows.append(row)
        groups[(row["scene"], row["noiseStdPerMm"], row["method"])].append(row)
    csv_path = folder / "paired_source_data.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    style()
    figures = folder / "figures"
    figures.mkdir(exist_ok=True)
    scenes = list(dict.fromkeys(c["sceneId"] for c in selected))
    noise = sorted({c["noiseStdPerMm"] for c in selected if c["noiseStdPerMm"] > 0})
    fig, axes = plt.subplots(len(scenes), 3, figsize=(7.1, 4.2), squeeze=False)
    for i, scene in enumerate(scenes):
        for j, (metric, label) in enumerate((("contactRmseN", "Contact-vector RMSE (N)"),
                                            ("tipRmseN", "Tip-vector RMSE (N)"),
                                            ("totalRmseN", "Total-vector RMSE (N)"))):
            ax = axes[i, j]
            for k, method in enumerate(METHODS):
                for n, sigma in enumerate(noise):
                    valid_rows = [r for r in groups[(scene, sigma, method)] if r[metric] is not None]
                    values = [r[metric] for r in valid_rows]
                    if not values:
                        continue
                    x = n + (k - 1) * .2
                    y = np.mean(values)
                    ax.errorbar(x, y, yerr=[[y - min(values)], [max(values) - y]], color=COLORS[method],
                                fmt="o", ms=4, lw=1.1, capsize=2,
                                label=LABELS[method] if n == 0 else None, zorder=3)
                    seed_x = x + np.linspace(-.035, .035, len(values))
                    ax.scatter(seed_x, values,
                               color=COLORS[method], s=9, alpha=.55, zorder=2)
                    stopped = [p for p, row in enumerate(valid_rows) if nonpositive_exit(row["solverExitflag"])]
                    if stopped:
                        ax.scatter(seed_x[stopped], np.asarray(values)[stopped], marker="^", s=24,
                                   facecolors="white", edgecolors="#A33A32", linewidths=.8, zorder=4)
            ax.set(xticks=range(len(noise)), xticklabels=[f"{n*1e6:g}" for n in noise],
                   xlabel="Noise SD (10⁻⁶ / mm)", ylabel=label, ylim=(0, None))
            ax.grid(axis="y", color="#E8ECF0", lw=.6)
            ax.text(-.17, 1.04, chr(97 + i * 3 + j), transform=ax.transAxes, weight="bold", fontsize=10)
            if j == 0:
                ax.set_title(SCENES.get(scene, scene), loc="left", fontsize=8, pad=14)
    if any(nonpositive_exit(r["solverExitflag"]) and r["noiseStdPerMm"] > 0 for r in rows):
        axes[0, 0].plot([], [], linestyle="none", marker="^", markerfacecolor="white",
                        markeredgecolor="#A33A32", label="Nonpositive exit")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    labels = [LABELS[m] for m in METHODS if LABELS[m] in by_label] + (
        ["Nonpositive exit"] if "Nonpositive exit" in by_label else [])
    handles = [by_label[label] for label in labels]
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), frameon=False, fontsize=7)
    fig.subplots_adjust(left=.08, right=.99, top=.94, bottom=.16, wspace=.47, hspace=.63)
    products = save(fig, figures, "matched_baselines")
    fig, axes = plt.subplots(len(scenes), 1, figsize=(3.45, 3.9), squeeze=False)
    magnitude_rows = []
    for i, scene in enumerate(scenes):
        ax = axes[i, 0]
        cases = [c for c in selected if c["sceneId"] == scene and c["method"] == "full" and c["noiseStdPerMm"] == noise[0]]
        frames = cases[0]["metrics"]["perFrame"]
        contact_count = cases[0]["metrics"]["trueContactCount"]
        truth = np.concatenate([np.atleast_1d(f["trueForceN"]) for f in frames])
        predicted = np.array([np.concatenate([np.atleast_1d(f["estimatedForceN"]) for f in c["metrics"]["perFrame"]]) for c in cases])
        x = np.arange(len(truth))
        ax.plot(x, truth, "o", color="#222E3B", ms=5, label="Independent truth", zorder=3)
        mean = predicted.mean(axis=0)
        ax.errorbar(x + .13, mean, yerr=[mean - predicted.min(axis=0), predicted.max(axis=0) - mean],
                    fmt="s", ms=4, capsize=2, color=COLORS["full"], label="EnFiRCE seed mean / range", zorder=3)
        for k, case in enumerate(cases):
            ax.scatter(x + .13 + (k - 1) * .03, predicted[k], s=10, color=COLORS["full"], alpha=.4)
            for j, (true, estimate) in enumerate(zip(truth, predicted[k])):
                magnitude_rows.append({"scene": scene, "seed": case["seed"], "frame": j // contact_count + 1,
                                       "contact": j % contact_count + 1, "trueMagnitudeN": true, "estimatedMagnitudeN": estimate})
        ax.set(xticks=x + .065, xticklabels=[f"{j//contact_count+1}:{j%contact_count+1}" for j in x],
               xlabel="State : contact (arc order)", ylabel="Force magnitude (N)", title=SCENES.get(scene, scene))
        ax.grid(axis="y", color="#E8ECF0", lw=.6)
        ax.text(-.14, 1.08, chr(97+i), transform=ax.transAxes, weight="bold", fontsize=10)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=1, frameon=False, fontsize=7)
    fig.subplots_adjust(left=.17, right=.98, top=.9, bottom=.2, hspace=.9)
    products += save(fig, figures, "contact_magnitudes")
    fig, axes = plt.subplots(1, 2, figsize=(3.45, 2.25))
    for scene in scenes:
        coverage, widths = [], []
        for sigma in noise:
            windows = [c for c in factors["cases"] if c["sceneId"] == scene and c["method"] == "full" and c["noiseStdPerMm"] == sigma]
            eligible = sum(c["coverage"]["eligibleComponents"] for c in windows)
            covered = sum(c["coverage"]["coveredComponents"] for c in windows)
            valid = [(c["coverage"]["meanIntervalWidthN"], c["coverage"]["eligibleComponents"]) for c in windows
                     if c["coverage"]["meanIntervalWidthN"] is not None]
            coverage.append(covered / eligible if eligible else np.nan)
            denominator = sum(n for _, n in valid)
            widths.append(sum(w * n for w, n in valid) / denominator if denominator else np.nan)
        label = "Three contacts" if scene == "three_contact" else "Spatial sliding"
        axes[0].plot(np.array(noise) * 1e6, coverage, "o-", ms=3, label=label)
        axes[1].plot(np.array(noise) * 1e6, widths, "o-", ms=3)
    axes[0].axhline(.95, color="#687587", linestyle="--", linewidth=.7)
    axes[0].set(ylabel="Eligible coverage", ylim=(0, 1.06))
    axes[1].set(ylabel="Mean full width (N)", ylim=(0, None))
    for i, ax in enumerate(axes):
        ax.set(xticks=np.array(noise) * 1e6, xlabel="Noise SD (10⁻⁶ / mm)")
        ax.grid(axis="y", color="#E8ECF0", lw=.6)
        ax.text(-.25, 1.04, chr(97+i), transform=ax.transAxes, weight="bold", fontsize=9)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="lower center", ncol=2, frameon=False, fontsize=6.5)
    fig.subplots_adjust(left=.16, right=.98, top=.94, bottom=.33, wspace=.77)
    products += save(fig, figures, "local_uncertainty")
    products += method_diagram(figures)
    magnitude_csv = folder / "contact_magnitude_source_data.csv"
    with magnitude_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(magnitude_rows[0]))
        writer.writeheader()
        writer.writerows(magnitude_rows)
    summary_groups = []
    for (scene, sigma, method), values in sorted(groups.items()):
        item = {"scene": scene, "noiseStdPerMm": sigma, "method": method, "windowCount": len(values),
                "reviewFrames": sum(r["reviewFrames"] for r in values),
                "nonpositiveExits": sum(nonpositive_exit(r["solverExitflag"]) for r in values)}
        for metric in ("contactRmseN", "tipRmseN", "totalRmseN", "arcRmseMm", "magnitudeMaeN", "wallSeconds"):
            v = [r[metric] for r in values if r[metric] is not None]
            item[metric] = {"mean": float(np.mean(v)), "min": min(v), "max": max(v), "eligible": len(v)} if v else None
        summary_groups.append(item)
    summary = {"state": "complete", "pairedPackets": len(paired), "methodRuns": len(selected),
               "groups": summary_groups, "sotaEstablished": False,
               "scope": "Identical input/truth bytes, observation-derived candidate count; adapted point/Gaussian LS, not official factor-graph implementations. Seed ranges, no trajectory-level CI."}
    summary_path = folder / "paired_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    tex = [r"\begin{table}[t]", r"\caption{Matched curvature packets at noise SD $2.5\times10^{-5}$ mm$^{-1}$. Entries are seed means (N); three sensor draws per fixed two-state trajectory.}",
           r"\label{tab:paired}", r"\centering\small", r"\begin{tabular}{llrrr}", r"\toprule", r"Scene & Method & Contact & Tip & Total\\", r"\midrule"]
    for scene in scenes:
        for method in METHODS:
            v = next(v for v in summary_groups if v["scene"] == scene and v["method"] == method and v["noiseStdPerMm"] == noise[0])
            entries = [f"{v[m]['mean']:.3f}" if v[m] else "--" for m in ("contactRmseN", "tipRmseN", "totalRmseN")]
            method_label = "EnFiRCE" if method == "full" else method.capitalize() + " LS"
            if v["nonpositiveExits"]:
                method_label += r"$^{\dagger}$"
            tex.append(" & ".join(["Three" if scene == "three_contact" else "Spatial", method_label, *entries]) + r"\\")
    tex += [r"\bottomrule", r"\end{tabular}"]
    if any(v["nonpositiveExits"] for v in summary_groups if v["noiseStdPerMm"] == noise[0]):
        tex += [r"\par\smallskip\footnotesize $^{\dagger}$Includes a nonpositive final exit; estimates are retained."]
    tex += [r"\end{table}", ""]
    tex_path = folder / "paired_table.tex"
    tex_path.write_text("\n".join(tex), encoding="utf-8")
    products += [csv_path, magnitude_csv, summary_path, tex_path]
    runtime_path = folder / "runtime_host.json"
    if runtime_path.exists():
        products.append(runtime_path)
    provenance = {"schemaVersion": 1, "pairedPackets": len(paired), "methodRuns": len(selected),
                  "rendererSha256": sha(Path(__file__)), "sourceLedgers": [
                      {"path": (f / "comparison.json").relative_to(ROOT).as_posix(), "sha256": sha(f / "comparison.json")}
                      for f in (factor_folder, base_folder)],
                  "outputs": [{"path": p.relative_to(ROOT).as_posix(), "sha256": sha(p)} for p in products],
                  "statistics": summary["scope"]}
    (folder / "figure_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    print(f"Verified and plotted {len(selected)} method runs on {len(paired)} byte-identical packets.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagram-only", action="store_true")
    parser.add_argument("--folder", type=Path, default=ROOT / "out/benchmarks/publication")
    args = parser.parse_args()
    if args.diagram_only:
        method_diagram(args.folder / "figures")
    else:
        publication(args.folder.resolve())
