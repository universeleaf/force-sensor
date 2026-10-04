"""Verify and plot new fixed configurations without hiding failures or exits."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from itertools import product
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from render_formulation_factors import LABELS, nonpositive_exit, save, sha

ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    "parallel_channel": "Parallel channel",
    "tapered_channel": "Tapered channel",
    "wider_channel": "Wider channel",
    "serpentine_tip_load": "Serpentine / tip load",
    "longer_serpentine": "Longer serpentine",
    "spatial_sparse": "Spatial / 12 observations",
}
METHODS = ["full", "no_geometry", "point", "gaussian"]
COLORS = {"full": "#0072B2", "no_geometry": "#CC79A7", "point": "#D55E00", "gaussian": "#009E73"}
SHORT = ["EnFiRCE", "− Geometry", "Point LS", "Gaussian LS"]


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def vector(v):
    return np.atleast_1d(v).tolist()


def csv_write(path, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def finite(values):
    return [float(v) for v in values if v is not None and np.isfinite(v)]


def collect(folder):
    artifacts = {}
    ledgers = []
    rows = []
    force_rows = []
    geometry = {}
    definitions = {}
    expected_options = None
    checked_sources = {}

    def check(path, expected):
        path = path.resolve()
        if not path.is_relative_to(ROOT) or sha(path) != expected:
            raise ValueError(f"Changed or unsafe artifact: {path}")
        artifacts[path.relative_to(ROOT).as_posix()] = expected

    for scene in SCENES:
        scene_folder = folder / scene
        ledger_path = scene_folder / "comparison.json"
        report = read(ledger_path)
        if report["state"] not in ("complete", "complete-with-failures"):
            raise ValueError(f"Scene is still running: {scene}")
        options = report["options"]
        signature = {k: options[k] for k in ("seeds", "methods", "noiseLevelsPerMm", "sampleEnvironment", "solverOptions")}
        if expected_options is None:
            expected_options = signature
        if signature != expected_options or vector(options["sceneIds"]) != [scene] or vector(options["methods"]) != METHODS:
            raise ValueError("Every scene must use the same paired protocol.")
        cases = report["cases"]
        cases = [cases] if isinstance(cases, dict) else cases
        expected = set(product(METHODS, vector(options["seeds"]), vector(options["noiseLevelsPerMm"])))
        actual = {(c["method"], c["seed"], c["noiseStdPerMm"]) for c in cases}
        if actual != expected or len(cases) != len(expected) or len({c["id"] for c in cases}) != len(cases):
            raise ValueError(f"Missing or duplicate cases: {scene}")
        if report["failureCount"] != sum(not c["completed"] for c in cases):
            raise ValueError("Failure count disagrees with individual records.")
        for source in report["runRecord"]["source"] + report["runRecord"]["dependency"]["files"]:
            previous = checked_sources.get(source["path"])
            if previous is None:
                check(ROOT / source["path"], source["sha256"])
                checked_sources[source["path"]] = source["sha256"]
            elif previous != source["sha256"]:
                raise ValueError("Scenes executed different source versions.")
        manifest_path = scene_folder / "clean/manifest.json"
        check(manifest_path, report["referenceComparisonSha256"])
        manifest = read(manifest_path)
        for name, extension in (("input", "mat"), ("truth", "mat"), ("geometry", "json")):
            check(scene_folder / "clean" / f"{name}.{extension}", manifest["artifactSha256"][name])
        geometry[scene] = read(scene_folder / "clean/geometry.json")
        definitions[scene] = manifest["definition"]
        packet_hashes = {}
        for case in cases:
            dest = (scene_folder / case["artifactFolder"]).resolve()
            if not dest.is_relative_to(scene_folder.resolve()):
                raise ValueError("Case folder escaped its scene.")
            check(dest.parent / "input.mat", case["inputSha256"])
            check(dest.parent / "truth.mat", case["truthSha256"])
            packet = (case["seed"], case["noiseStdPerMm"])
            hashes = (case["inputSha256"], case["truthSha256"])
            if packet in packet_hashes and hashes != packet_hashes[packet]:
                raise ValueError("Methods did not use byte-identical packets.")
            packet_hashes[packet] = hashes
            metrics = case["metrics"] if case["completed"] else {}
            solver = case.get("solver", {})
            row = {"scene": scene, "method": case["method"], "seed": case["seed"],
                   "noiseStdPerMm": case["noiseStdPerMm"], "completed": case["completed"],
                   "contactRmseN": metrics.get("contactForceRmseN"),
                   "magnitudeMaeN": metrics.get("contactMagnitudeMaeN"),
                   "tipRmseN": metrics.get("tipForceRmseN"), "totalRmseN": metrics.get("totalForceRmseN"),
                   "arcRmseMm": metrics.get("contactArcRmseMm"), "shapeRmseMm": metrics.get("shapeRmseMm"),
                   "countMatches": metrics.get("countMatches"), "reviewFrames": metrics.get("reviewCount"),
                   "nonpositiveExit": nonpositive_exit(solver.get("exitflag")),
                   "wallSeconds": case["wallSeconds"], "error": case["error"],
                   "inputSha256": case["inputSha256"], "truthSha256": case["truthSha256"]}
            rows.append(row)
            if case["completed"]:
                check(dest / "estimate.mat", case["estimateSha256"])
                check(dest / "forces.csv", case["csvSha256"])
                frames = metrics["perFrame"]
                frames = [frames] if isinstance(frames, dict) else frames
                for frame in frames:
                    actual_force = vector(frame["trueForceN"])
                    estimated_force = vector(frame["estimatedForceN"])
                    # No pairing is fabricated when active-contact counts differ.
                    if len(actual_force) != len(estimated_force):
                        continue
                    for slot, (actual_f, estimated_f) in enumerate(zip(actual_force, estimated_force), 1):
                        force_rows.append({"scene": scene, "method": case["method"], "seed": case["seed"],
                                           "frame": frame["frame"], "contact": slot, "trueMagnitudeN": actual_f,
                                           "estimatedMagnitudeN": estimated_f, "nonpositiveExit": row["nonpositiveExit"]})
        ledgers.append({"path": ledger_path.relative_to(ROOT).as_posix(), "sha256": sha(ledger_path),
                        "runId": report["runRecord"]["runId"], "state": report["state"]})
    return rows, force_rows, geometry, definitions, ledgers, artifacts, expected_options


def render(folder):
    rows, forces, geometry, definitions, ledgers, artifacts, options = collect(folder)
    csv_write(folder / "source_data.csv", rows)
    if forces:
        csv_write(folder / "force_data.csv", forces)
    groups = defaultdict(list)
    for row in rows:
        groups[(row["scene"], row["method"])].append(row)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 7.5, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42, "svg.fonttype": "none"})
    figures = folder / "figures"
    figures.mkdir(exist_ok=True)
    outputs = []
    fig, axes = plt.subplots(2, 3, figsize=(7.2, 4.5), layout="constrained")
    for panel, (scene, ax) in enumerate(zip(SCENES, axes.flat)):
        for j, method in enumerate(METHODS):
            selected = groups[(scene, method)]
            errors = finite(v["contactRmseN"] for v in selected)
            if errors:
                center = np.mean(errors)
                ax.errorbar(j, center, yerr=[[center-min(errors)], [max(errors)-center]],
                            fmt="s", color=COLORS[method], capsize=3, markersize=4, lw=1.1)
                for k, v in enumerate(selected):
                    if v["contactRmseN"] is not None:
                        ax.scatter(j + (k-(len(selected)-1)/2)*.1, v["contactRmseN"],
                                   color=COLORS[method], s=10, zorder=3)
                        if v["nonpositiveExit"]:
                            ax.scatter(j + (k-(len(selected)-1)/2)*.1, v["contactRmseN"], marker="^",
                                       facecolors="white", edgecolors="#333333", s=28, zorder=4)
            missing = sum(v["contactRmseN"] is None for v in selected)
            if missing:
                ax.text(j, .04, f"missing {missing}/{len(selected)}", transform=ax.get_xaxis_transform(),
                        ha="center", rotation=90, fontsize=6)
        ax.set_xticks(range(4), SHORT, rotation=22, ha="right", fontsize=6.5)
        ax.set_yscale("symlog", linthresh=.1, linscale=.65)
        ax.set_ylim(bottom=0)
        ax.set_ylabel("Contact-vector RMSE (N)")
        ax.set_title(f"{chr(97+panel)}  {SCENES[scene]}", loc="left", fontsize=8)
        ax.grid(axis="y", alpha=.2)
    outputs += save(fig, figures, "generalization_accuracy")

    fig, axes = plt.subplots(2, 3, figsize=(7.2, 5.1), layout="constrained")
    for panel, (scene, ax) in enumerate(zip(SCENES, axes.flat)):
        g = geometry[scene]
        p = np.asarray(g["pMm"])
        points = np.asarray(g["contactPointsMm"])
        planes = np.asarray(g["planePointMm"])
        normals = np.asarray(g["planeNormal"])
        z = np.linspace(float(p[2].min())-5, float(p[2].max())+5, 100)
        for wall in range(planes.shape[1]):
            n, q = normals[:, wall, 0], planes[:, wall, 0]
            x = q[0] - n[2]*(z-q[2])/n[0]
            ax.plot(x, z, color="#A3ADB4", lw=1.5)
        for t, (style, color) in enumerate((("-", "#0072B2"), ("--", "#D55E00"))):
            ax.plot(p[0, :, t], p[2, :, t], style, color=color, lw=1.2)
            ax.scatter(points[0, :, t], points[2, :, t], s=12, facecolors="white", edgecolors=color, zorder=3)
        if scene == "spatial_sparse":
            ax.text(.03, .03, "x–z projection; Δbase y = 0.5 mm", transform=ax.transAxes, fontsize=5.8)
        ax.set(title=f"{chr(97+panel)}  {SCENES[scene]}", xlabel="x (mm)", ylabel="z (mm)")
        ax.set_aspect("equal", adjustable="datalim")
    outputs += save(fig, figures, "generalization_geometry")

    fig, axes = plt.subplots(2, 3, figsize=(7.2, 4.7), layout="constrained")
    for panel, (scene, ax) in enumerate(zip(SCENES, axes.flat)):
        selected = [v for v in forces if v["scene"] == scene]
        positions = sorted({(v["frame"], v["contact"]) for v in selected})
        for j, position in enumerate(positions):
            at_contact = [v for v in selected if (v["frame"], v["contact"]) == position]
            if at_contact:
                ax.plot([j-.35, j+.35], [at_contact[0]["trueMagnitudeN"]]*2, color="#333333", lw=1.3)
            for k, method in enumerate(METHODS):
                values = [v["estimatedMagnitudeN"] for v in at_contact if v["method"] == method]
                if values:
                    x = j + (k-1.5)*.16
                    mean = np.mean(values)
                    ax.errorbar(x, mean, yerr=[[mean-min(values)], [max(values)-mean]], fmt="o",
                                color=COLORS[method], ms=3, capsize=2, lw=.7)
        ax.set_xticks(range(len(positions)), [f"C{k} / t{t-1}" for t, k in positions], rotation=28, ha="right", fontsize=6)
        ax.set_ylim(bottom=0)
        ax.set_ylabel("Contact force magnitude (N)")
        ax.set_title(f"{chr(97+panel)}  {SCENES[scene]}", loc="left", fontsize=8)
        ax.grid(axis="y", alpha=.2)
    handles = [plt.Line2D([0], [0], color="#333333", label="Independent truth")]
    handles += [plt.Line2D([0], [0], color=COLORS[m], marker="o", lw=0, label=LABELS[m]) for m in METHODS]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, fontsize=6.5)
    outputs += save(fig, figures, "generalization_magnitudes")
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 3.4), sharey=True, layout="constrained")
    comparators = ["no_geometry", "point", "gaussian"]
    metrics = [("magnitudeMaeN", "Magnitude MAE"), ("tipRmseN", "Tip-vector RMSE"), ("totalRmseN", "Total-vector RMSE")]
    for panel, (ax, (metric, title)) in enumerate(zip(axes, metrics)):
        for i, scene in enumerate(SCENES):
            ours = {v["seed"]: v[metric] for v in groups[(scene, "full")]}
            for j, method in enumerate(comparators):
                others = {v["seed"]: v[metric] for v in groups[(scene, method)]}
                valid = [seed for seed in ours if ours[seed] is not None and others.get(seed) is not None and others[seed] > 0]
                if not valid:
                    continue
                ratios = [ours[seed]/others[seed] for seed in valid]
                center = np.mean([ours[seed] for seed in valid])/np.mean([others[seed] for seed in valid])
                y = i + (j-1)*.19
                ax.errorbar(center, y, xerr=[[center-min(ratios)], [max(ratios)-center]], fmt="o",
                            color=COLORS[method], ms=3.5, lw=.8, capsize=2)
                ax.scatter(ratios, [y]*len(ratios), s=8, color=COLORS[method], alpha=.45)
        ax.axvline(1, ls="--", lw=.8, color="#595959")
        ax.set_xscale("log")
        ax.set_title(f"{chr(97+panel)}  {title}", loc="left", fontsize=8)
        ax.set_xlabel("EnFiRCE / comparator error")
        ax.grid(axis="x", alpha=.15)
    axes[0].set_yticks(range(len(SCENES)), ["Parallel", "Tapered", "Wider", "Changed tip", "Longer", "Spatial / 12"])
    axes[0].invert_yaxis()
    handles = [plt.Line2D([0], [0], color=COLORS[m], marker="o", lw=0, label=LABELS[m]) for m in comparators]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, fontsize=6.5)
    outputs += save(fig, figures, "generalization_load_errors")

    summary = []
    for (scene, method), group in groups.items():
        item = {"scene": scene, "method": method, "attempted": len(group),
                "completed": sum(v["completed"] for v in group),
                "matched": sum(v["countMatches"] is True for v in group),
                "nonpositiveExits": sum(v["nonpositiveExit"] for v in group)}
        for metric in ("contactRmseN", "magnitudeMaeN", "tipRmseN", "totalRmseN", "arcRmseMm"):
            values = finite(v[metric] for v in group)
            item[metric] = float(np.mean(values)) if values else None
        summary.append(item)
    document = {"state": "complete" if all(v["completed"] for v in rows) else "complete-with-failures",
                "caseCount": len(rows), "failureCount": sum(not v["completed"] for v in rows),
                "sotaEstablished": False, "options": options, "sceneDefinitions": definitions,
                "sourceLedgers": ledgers, "summary": summary,
                "scope": "Fixed configurations, three sensor realizations each. Point/Gaussian are disclosed adaptations, not official implementations. No across-paper hardware ranking or real-time claim."}
    lines = ["# 新配置的同观测力估计比较", "", "更新：2026-10-05。", "",
             f"实际记录 {len(rows)} 次运行：6 配置 × 3 传感器种子 × 4 方法。异常 {document['failureCount']} 次。结果来自新的独立配置或明确标注的观测密度控制；不是插值视频或论文报告数值搬运。", "",
             "## 1. 比较协议", "",
             "曲率噪声 SD 2.5e-5 /mm，同时按观测包的平面协方差采样位置/法向噪声：位置分量 SD 0.05 mm、法向分量 SD 0.001，法向随后单位化。种子为 11、23、37；同维数配置使用共同标准噪声。每个固定配置的三个观测实现独立，但不是三条机器人轨迹。", "",
             "每份 input.mat 与 truth.mat 的字节在四种方法间相同。真值从未送入逆模型；候选来自观测。full 和 no_geometry 都保留完整 3-D Cosserat 窗口、前一时刻平衡、独立末端力与潜在环境参数；后者只去掉接触 gap/tangency/nonpenetration。Point LS / Gaussian LS 为项目中的文献思想适配，保留既有三个独立初值、每初值 80 次迭代，不接受 EnFiRCE 解初始化；共享观测候选数量，使用自己的固定有序弧长分区。它们省略环境/过程因素，故比较完整估计器的输入利用能力，不能称为信息量匹配的单一优化器竞赛。", "",
             "下表均值按三个窗口计算；每个窗口内部计算两帧、所有匹配接触的向量 RMS。力大小 MAE 单独计算。非正退出/复核不删除，接触数不匹配时不给虚构的接触误差。", "",
             "## 2. 全部方法结果", "",
             "| 场景 | 方法 | 匹配/尝试 | 接触向量 RMSE /N | 力大小 MAE /N | 末端 RMSE /N | 总力 RMSE /N | 非正退出 |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    def fmt(value):
        return f"{value:.6g}" if value is not None else "缺失/不匹配"
    for item in summary:
        lines.append(f"| {SCENES[item['scene']]} | {LABELS[item['method']]} | {item['matched']}/{item['attempted']} | "
                     + " | ".join(fmt(item[k]) for k in ("contactRmseN", "magnitudeMaeN", "tipRmseN", "totalRmseN"))
                     + f" | {item['nonpositiveExits']} |")
    lines += ["", "![六个配置的接触力误差](../out/benchmarks/generalization/v1/figures/generalization_accuracy.png)", "",
              "方形标记和线段为三个种子的均值及最小/最大范围，圆点为每个种子；范围不是置信区间。纵轴 symlog 在 0.1 N 以下线性，以上为对数。空心三角标记非正退出，并保留在统计中。", "",
              "![力大小、末端和总力的配对误差比值](../out/benchmarks/generalization/v1/figures/generalization_load_errors.png)", "",
              "辅助误差图的中心是 EnFiRCE 三种子均值 / 对照方法三种子均值，点和范围为逐种子配对比值。小于 1 表示该指标完整法误差更低，大于 1 的结果同样保留；范围不是置信区间。", "",
              "## 3. 各接触的实际力大小", "",
              "![真实力大小与四种方法](../out/benchmarks/generalization/v1/figures/generalization_magnitudes.png)", "",
              "灰线为独立 shooting 真值，彩点及范围为各方法三个种子的估计。C1/C2/C3 按材料弧长排序，t0/t1 是实际独立求解的两个平衡。单位为 N，采用项目当前校准刚度，不能解释为硬件测量。", "",
              "## 4. 物理配置与实现", "",
              "![实际求解的场景几何](../out/benchmarks/generalization/v1/figures/generalization_geometry.png)", "",
              "- parallel_channel：140 mm S 形杆、两壁 x=±10 mm，两接触，基座从 x=-0.6 至 +0.6 mm，独立末端力 [0.1,0,-0.08] N。",
              "- tapered_channel：同长度，非平行法向 [1,0,-0.05] / [-1,0,-0.05] 经单位化，真实两接触。",
              "- wider_channel：两壁 x=±11 mm，末端力变为 [0.15,0,-0.10] N；重新求解真值。",
              "- serpentine_tip_load：210 mm 三接触蛇形杆，末端力变为 [0.25,0,-0.15] N。",
              "- longer_serpentine：210 mm 配置按 8/7 几何缩放为 240 mm；段长 80 mm、曲率 0.0175 /mm、壁距和基座移动同样缩放；保持刚度，载荷按 (7/8)^2 缩放。这是相似几何尺度对照，不是新接触拓扑。",
              "- spatial_sparse：旧真实 3-D 双接触滑动真值，µ=0.03，基座 +y 0.5 mm，末端力 [0.1,0.3,-0.08] N；仅将 24 个观测位置均匀减为 12 个。几何图为 x-z 投影，不冒充新的物理轨迹或 stick-slip。", "",
              "前五项用独立连续平面 shooting 生成新平衡，再嵌入 3-D 观测；逆模型仍使用非线性 3-D Cosserat。每次构造检查整杆穿透、接触 gap/tangency、端部力矩和平衡。240 mm 的第一次配置保持 20 mm 通道不变，整杆穿透 1.95 mm，审计拒绝，未进入估计排名；拒绝记录保留在 fixture_diagnostics.json。", "",
              "代码对应：", "",
              "| 部分 | 实现 |",
              "|---|---|",
              "| 六场景定义、独立输出、同输入方法循环、SHA/恢复 | [run_generalization_comparison.m](../rod/run_generalization_comparison.m) |",
              "| 连续平面真值 | [build_multi_contact_demo_truth.m](../rod/build_multi_contact_demo_truth.m)、[solve_planar_multi_contact.m](../rod/solve_planar_multi_contact.m) |",
              "| 真正空间摩擦真值 | [build_spatial_friction_packet.m](../rod/build_spatial_friction_packet.m) |",
              "| 完整窗口 MAP、协方差和质量标志 | [estimate_formulation_window.m](../rod/estimate_formulation_window.m) |",
              "| 观测候选与跨帧关联 | [formulation_contact_candidates.m](../rod/formulation_contact_candidates.m) |",
              "| 曲率文献适配基线 | [estimate_literature_curvature_baseline.m](../rod/estimate_literature_curvature_baseline.m) |",
              "| 曲率/环境观测采样 | [resample_formulation_observations.m](../rod/resample_formulation_observations.m) |",
              "| 活动接触匹配、力大小/向量/末端/总力误差 | [score_formulation_window.m](../rod/score_formulation_window.m) |",
              "| 数据核验、配对、绘图与本报告 | [render_generalization_comparison.py](../scripts/render_generalization_comparison.py) |", "",
              "## 5. 复现、结果解释与数据", "",
              "```matlab", "addpath('rod'); addpath(genpath('LCP-Continuum'));",
              "ids={'parallel_channel','tapered_channel','wider_channel', ...",
              "     'serpentine_tip_load','longer_serpentine','spatial_sparse'};",
              "for i=1:numel(ids), run_generalization_comparison(ids{i}); end", "```", "",
              "```text", "python scripts/render_generalization_comparison.py", "```", "",
              "实际运行按场景使用六个并行 MATLAB -singleCompThread 进程，计时包含局部协方差及共享资源；不能用来宣布隔离速度或实时性能。重复运行精确验证选项、来源及保存文件，配置/源码改变必须建立新版本，不能覆盖旧结果。此前 108 次因素与 36 次基线数据没有改写。", "",
              "- [逐种子所有指标 CSV](../out/benchmarks/generalization/v1/source_data.csv)",
              "- [每帧每接触真值和估计大小 CSV](../out/benchmarks/generalization/v1/force_data.csv)",
              "- [机器可读配置及均值](../out/benchmarks/generalization/v1/summary.json)",
              "- [来源与图表校验清单](../out/benchmarks/generalization/v1/plot_provenance.json)",
              "- [首次不合法配置与修正说明](../out/benchmarks/generalization/v1/fixture_diagnostics.json)", "",
              "优势结论应限定为本组保存的条件/输入与这两个适配基线；基线不是原作者官方实现。已有论文使用不同传感器、载荷、硬件及指标，不能直接按其论文中的误差大小排行。Point 的思想依据 [Xiao and Chen, 2021](https://arxiv.org/abs/2109.12469)，Gaussian 的依据 [Aloi et al., 2022](https://doi.org/10.1109/LRA.2022.3188905)。本组不改动完整平衡、摩擦或递归近似范围，也不把更多仿真样例说成已经证明全局 SOTA。", ""]
    doc = ROOT / "docs/GENERALIZATION_RESULTS.md"
    doc.write_text("\n".join(lines), encoding="utf-8", newline="\n")

    report_path = folder / "summary.json"
    report_path.write_text(json.dumps(document, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    outputs += [folder / "source_data.csv", folder / "force_data.csv", report_path, doc, folder / "fixture_diagnostics.json"]
    provenance = {"sourceLedgers": ledgers, "rendererSha256": sha(Path(__file__)),
                  "helperSha256": sha(ROOT / "scripts/render_formulation_factors.py"),
                  "contractSha256": sha(ROOT / "docs/GENERALIZATION_FIGURE_CONTRACT.md"),
                  "artifacts": [{"path": path, "sha256": digest} for path, digest in artifacts.items()],
                  "outputs": [{"path": p.relative_to(ROOT).as_posix(), "sha256": sha(p)} for p in outputs],
                  "statistics": "Means, individual seeds and seed min/max; ranges are not confidence intervals. Every configured case is retained; unmatched contacts have no fabricated error."}
    (folder / "plot_provenance.json").write_text(json.dumps(provenance, indent=2)+"\n", encoding="utf-8")
    print(f"Verified {len(artifacts)} source/data artifacts; plotted {len(rows)} cases.")
    for item in summary:
        print(item["scene"], item["method"], item["contactRmseN"], "matched", item["matched"], "exits", item["nonpositiveExits"])


def verified_summary(folder=ROOT / "out/benchmarks/generalization/v1"):
    """Publication consumers must verify both executed inputs and plot outputs."""
    provenance = read(folder / "plot_provenance.json")
    if provenance["rendererSha256"] != sha(Path(__file__)) or provenance["helperSha256"] != sha(ROOT / "scripts/render_formulation_factors.py"):
        raise ValueError("Comparison renderer changed; regenerate its figures.")
    if provenance["contractSha256"] != sha(ROOT / "docs/GENERALIZATION_FIGURE_CONTRACT.md"):
        raise ValueError("Figure contract changed.")
    for record in provenance["sourceLedgers"] + provenance["artifacts"] + provenance["outputs"]:
        path = (ROOT / record["path"]).resolve()
        if not path.is_relative_to(ROOT) or sha(path) != record["sha256"]:
            raise ValueError(f"Generalization artifact changed: {record['path']}")
    report = read(folder / "summary.json")
    if report["state"] != "complete" or report["failureCount"]:
        raise ValueError("Do not write complete-comparison prose for failed runs.")
    rows, _, _, _, ledgers, _, _ = collect(folder)
    if report["caseCount"] != len(rows) or report["sourceLedgers"] != ledgers:
        raise ValueError("Comparison summary is not bound to executed cases.")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=Path, default=ROOT / "out/benchmarks/generalization/v1")
    render(parser.parse_args().folder.resolve())
