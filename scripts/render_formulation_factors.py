"""Plot completed full-window factor evidence; verify every archived artifact."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from itertools import product
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LABELS = {
    "full": "EnFiRCE",
    "no_temporal": "No temporal prior",
    "cone_only": "Friction cone only",
    "no_geometry": "No contact geometry",
    "no_camera": "No camera likelihood",
    "legacy_partitions": "Fixed arc partitions",
    "point": "Adapted point LS",
    "gaussian": "Adapted Gaussian LS",
}
COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#595959", "#56B4E9", "#882255"]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nonpositive_exit(value) -> bool:
    """Window solver uses a scalar; independent-frame baselines use an array."""
    return value is not None and bool(np.any(np.atleast_1d(value) <= 0))


def save(fig: plt.Figure, folder: Path, name: str) -> list[Path]:
    files = []
    for extension in ("pdf", "svg", "png"):
        path = folder / f"{name}.{extension}"
        fig.savefig(path, dpi=240, bbox_inches="tight")
        if extension == "svg":
            path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="utf-8").splitlines()) + "\n", encoding="utf-8")
        files.append(path)
    plt.close(fig)
    return files


def render(folder: Path) -> None:
    path = folder / "comparison.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    if report["state"] != "complete" or report["failureCount"]:
        raise ValueError("Only a completed, exception-free ledger can generate this comparison.")
    methods = report["options"]["methods"]
    scenes = report["options"]["sceneIds"]
    methods = [methods] if isinstance(methods, str) else methods
    scenes = [scenes] if isinstance(scenes, str) else scenes
    cases = report["cases"]
    cases = [cases] if isinstance(cases, dict) else cases
    expected = len(methods) * len(scenes) * np.size(report["options"]["seeds"]) * np.size(report["options"]["noiseLevelsPerMm"])
    if len(cases) != expected or len({case["id"] for case in cases}) != expected:
        raise ValueError("The ledger does not contain every configured case exactly once.")
    configured = set(product(scenes, methods, np.atleast_1d(report["options"]["seeds"]),
                             np.atleast_1d(report["options"]["noiseLevelsPerMm"])))
    recorded = {(c["sceneId"], c["method"], c["seed"], c["noiseStdPerMm"]) for c in cases}
    if configured != recorded:
        raise ValueError("Recorded cases differ from the configured experiment matrix.")
    artifacts = []
    rows = []
    groups = defaultdict(list)
    for case in cases:
        if not case["completed"]:
            raise ValueError(f"Incomplete case: {case['id']}")
        method_folder = (folder / case["artifactFolder"]).resolve()
        if not method_folder.is_relative_to(folder.resolve()):
            raise ValueError("Artifact escaped the result folder.")
        for artifact, expected_sha in (
            (method_folder.parent / "input.mat", case["inputSha256"]),
            (method_folder.parent / "truth.mat", case["truthSha256"]),
            (method_folder / "estimate.mat", case["estimateSha256"]),
            (method_folder / "forces.csv", case["csvSha256"]),
        ):
            if sha(artifact) != expected_sha:
                raise ValueError(f"Artifact checksum mismatch: {artifact}")
            artifacts.append({"path": artifact.relative_to(ROOT).as_posix(), "sha256": expected_sha})
        metrics = case["metrics"]
        coverage = case["coverage"]
        row = {"scene": case["sceneId"], "method": case["method"], "seed": case["seed"],
               "noiseStdPerMm": case["noiseStdPerMm"], "contactRmseN": metrics["contactForceRmseN"],
               "totalRmseN": metrics["totalForceRmseN"], "tipRmseN": metrics["tipForceRmseN"],
               "arcRmseMm": metrics["contactArcRmseMm"], "reviewFrames": metrics["reviewCount"],
               "countMatches": metrics["countMatches"], "wallSeconds": case["wallSeconds"],
               "coveredComponents": coverage["coveredComponents"], "eligibleComponents": coverage["eligibleComponents"],
               "unresolvedComponents": coverage["unresolvedComponents"], "unmatchedFrames": coverage["unmatchedFrames"],
               "meanIntervalWidthN": coverage["meanIntervalWidthN"],
               "solverExitflag": case["solver"].get("exitflag"),
               "maxConfiguredViolation": max(np.atleast_1d(case["quality"].get("configuredConstraintViolation", [0]))),
               "maxWhitenedCurvatureRms": max(np.atleast_1d(case["quality"]["curvatureResidualRms"]))
                   if "curvatureResidualRms" in case["quality"] else None,
               "observationFitWarning": bool(np.any(case["quality"]["hasObservationFitWarning"]))
                   if "hasObservationFitWarning" in case["quality"] else None,
               "odeEvaluations": case["solver"].get("mechanicalEvaluations"),
               "odeCacheHits": case["solver"].get("mechanicalCacheHits")}
        rows.append(row)
        groups[(row["scene"], row["method"], row["noiseStdPerMm"])].append(row)
    csv_path = folder / "source_data.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42, "svg.fonttype": "none"})
    figures = folder / "figures"
    figures.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1, len(scenes), figsize=(3.6 * len(scenes), 2.9), squeeze=False, constrained_layout=True)
    for index, scene in enumerate(scenes):
        ax = axes[0, index]
        for j, method in enumerate(methods):
            values = []
            for key, group in sorted(groups.items()):
                if key[:2] != (scene, method):
                    continue
                errors = [v["contactRmseN"] for v in group if v["contactRmseN"] is not None]
                if errors:
                    values.append((key[2] * 1e6, np.mean(errors), min(errors), max(errors)))
            if values:
                x, y, low, high = np.array(values).T
                ax.plot(x, y, "o-", ms=3, lw=1.8 if method == "full" else 1,
                        zorder=5 if method == "full" else 2,
                        color=COLORS[j % len(COLORS)], label=LABELS[method])
                ax.fill_between(x, low, high, alpha=.09, color=COLORS[j % len(COLORS)])
                warned = [v for v in rows if v["scene"] == scene and v["method"] == method
                          and v["observationFitWarning"] and v["contactRmseN"] is not None]
                if warned:
                    ax.scatter([v["noiseStdPerMm"] * 1e6 for v in warned],
                               [v["contactRmseN"] for v in warned], marker="x", s=18,
                               color="#333333", linewidths=.7, zorder=6)
        stopped = [v for v in rows if v["scene"] == scene
                   and nonpositive_exit(v["solverExitflag"]) and v["contactRmseN"] is not None]
        if stopped:
            ax.scatter([v["noiseStdPerMm"] * 1e6 for v in stopped],
                       [v["contactRmseN"] for v in stopped], marker="^", s=28,
                       facecolors="white", edgecolors="#A33A32", linewidths=.9, zorder=7, clip_on=False,
                       label=f"Nonpositive exit ({len(stopped)} windows)")
        ax.set_yscale("symlog", linthresh=.1, linscale=.65)
        ax.set_ylim(bottom=0)
        ax.set(title=scene.replace("_", " ").capitalize(), xlabel="Curvature noise SD (10⁻⁶ / mm)", ylabel="Contact-vector RMSE (N)")
        ax.text(.02, .98, "Symlog: linear below 0.1 N", transform=ax.transAxes, va="top", fontsize=6, color="#595959")
        ax.grid(alpha=.2)
        if index == len(scenes) - 1:
            ax.legend(fontsize=6.5)
    files = save(fig, figures, "factor_accuracy")
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.9), constrained_layout=True)
    for j, scene in enumerate(scenes):
        values = []; widths = []
        for key, group in sorted(groups.items()):
            if key[:2] != (scene, "full") or not key[2]:
                continue
            eligible = sum(v["eligibleComponents"] for v in group)
            covered = sum(v["coveredComponents"] for v in group)
            if eligible:
                values.append((key[2] * 1e6, covered / eligible))
                weighted = sum(v["meanIntervalWidthN"] * v["eligibleComponents"] for v in group
                               if v["meanIntervalWidthN"] is not None)
                widths.append((key[2] * 1e6, weighted / eligible))
        if values:
            x, y = np.array(values).T
            axes[0].plot(x, y, "o-", color=COLORS[j], label=scene.replace("_", " "))
            x, y = np.array(widths).T
            axes[1].plot(x, y, "o-", color=COLORS[j])
    if not any(v["eligibleComponents"] for v in rows if v["method"] == "full"):
        axes[0].text(.5, .5, "No eligible full-method intervals\nin this comparison", transform=axes[0].transAxes,
                     ha="center", va="center", fontsize=8, color="#595959")
    axes[0].axhline(.95, color="#595959", ls="--", lw=.8, label="Nominal 95%")
    axes[0].set(xlabel="Curvature noise SD (10⁻⁶ / mm)", ylabel="Eligible-component coverage", ylim=(0, 1.02))
    axes[0].legend(fontsize=7)
    axes[1].set(xlabel="Curvature noise SD (10⁻⁶ / mm)", ylabel="Mean eligible interval full width (N)", ylim=(0, None))
    if not any(v["eligibleComponents"] for v in rows if v["method"] == "full"):
        axes[1].text(.5, .5, "Intervals not computed", transform=axes[1].transAxes,
                     ha="center", va="center", fontsize=8, color="#595959")
    latency = [[v["wallSeconds"] for v in rows if v["method"] == method] for method in methods]
    short = {"full": "EnFiRCE", "no_temporal": "No temporal", "cone_only": "Cone only", "no_geometry": "− Geometry",
             "no_camera": "− Camera", "legacy_partitions": "Partitions", "point": "Point LS", "gaussian": "Gaussian LS"}
    axes[2].boxplot(latency, tick_labels=[short[m] for m in methods], showfliers=True, orientation="horizontal",
                    medianprops={"color": "#0072B2"})
    axes[2].tick_params(axis="y", labelsize=7)
    axes[2].set(xlabel="Method call (s)")
    for j, ax in enumerate(axes):
        ax.grid(axis="x" if j == 2 else "y", alpha=.2)
        ax.set_title(chr(97+j), loc="left", fontweight="bold", fontsize=10)
    files += save(fig, figures, "coverage_runtime")
    lines = ["# 完整三维窗口因素实验", "", f"实际完成 {len(rows)} 次方法/观测包运行；run `{report['runRecord']['runId']}`。", "",
             "每个固定两帧条件内有三个独立种子，标准化噪声跨等级/同尺寸场景共用，clean 控制相同。均值按种子计算，阴影是种子最小/最大值，不是置信区间。纵轴为 symlog，0.1 N 以下线性；叉号为观测拟合复核，空心三角标记非正最终退出（重叠控制的数量见图例）。接触数不匹配的窗口不进入接触误差均值，数量明确列出；总合力误差保留。", "",
             "| 场景 | 噪声 SD /mm | 方法 | 有效窗口/全部 | 接触 RMSE 均值 / N | 总合力 RMSE 均值 / N | review 帧 | 非正退出窗口 | 覆盖分子/分母 | 未辨识分量 |", "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|"]
    for key, group in sorted(groups.items()):
        errors = [v["contactRmseN"] for v in group if v["contactRmseN"] is not None]
        contact = f"{np.mean(errors):.6g}" if errors else "不匹配"
        nonpositive = sum(nonpositive_exit(v['solverExitflag']) for v in group)
        lines.append(f"| {key[0]} | {key[2]:g} | {LABELS[key[1]]} | {len(errors)}/{len(group)} | {contact} | {np.mean([v['totalRmseN'] for v in group]):.6g} | {sum(v['reviewFrames'] for v in group)} | {nonpositive} | {sum(v['coveredComponents'] for v in group)}/{sum(v['eligibleComponents'] for v in group)} | {sum(v['unresolvedComponents'] for v in group)} |")
    lines += ["", "局部区间尚未包含接触模式混合或模型标定误差；有相关分量和相邻帧，不将其作为独立 Bernoulli 样本。没有由此建立 SOTA 或实时性。", "", "![逐因素精度](figures/factor_accuracy.png)", "", "![覆盖率和时间](figures/coverage_runtime.png)", ""]
    summary = folder / "summary.md"
    summary.write_text("\n".join(lines), encoding="utf-8")
    products = files + [csv_path, summary]
    provenance = {"sourceComparisonSha256": sha(path), "runId": report["runRecord"]["runId"],
                  "artifacts": list({v["path"]: v for v in artifacts}.values()),
                  "rendererSha256": sha(Path(__file__)), "outputs": [{"path": p.relative_to(ROOT).as_posix(), "sha256": sha(p)} for p in products],
                  "statistics": "Seed means/min-max of fixed trajectories; no independent-frame CI or global coverage claim."}
    (folder / "plot_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    print(f"Verified {len(provenance['artifacts'])} artifacts; rendered {len(rows)} cases.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=Path, default=ROOT / "out/benchmarks/formulation_factors")
    render(parser.parse_args().folder.resolve())
