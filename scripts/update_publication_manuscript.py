"""Update the supplied manuscript and public results from completed evidence.

No placeholder scores are accepted. The MATLAB completion ledger and figure
checksums must be intact before numerical prose or tables are written.
"""
from __future__ import annotations

import json
import re
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np

from render_formulation_factors import LABELS, nonpositive_exit, sha

ROOT = Path(__file__).resolve().parents[1]
PUBLICATION = ROOT / "out/benchmarks/publication"
SCENES = {"three_contact": "Three contacts", "spatial_sliding": "Spatial sliding"}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def evidence() -> tuple[dict, dict, dict, dict, dict]:
    from verify_publication_release import verify
    verify()
    completion = read(PUBLICATION / "completion.json")
    if completion["state"] != "complete" or completion["quickMode"]:
        raise ValueError("The complete publication protocol is required.")
    steps = {s["name"]: s for s in completion["completedSteps"]}
    if set(steps) != {"factors", "literature", "derivatives", "engineering"}:
        raise ValueError("A publication step is missing or duplicated.")
    reports = {}
    for name, step in steps.items():
        path = (ROOT / step["path"]).resolve()
        if not path.is_relative_to(ROOT) or sha(path) != step["sha256"]:
            raise ValueError(f"Completion checksum mismatch: {name}")
        reports[name] = read(path)
    if not reports["engineering"]["allPassed"]:
        raise ValueError("Engineering checks failed.")
    if reports["factors"]["failureCount"] or reports["literature"]["failureCount"]:
        raise ValueError("An inference exception remains.")
    for record in completion["runRecord"]["source"]:
        if sha(ROOT / record["path"]) != record["sha256"]:
            raise ValueError(f"Source changed since the completed protocol: {record['path']}")
    for record in completion["runRecord"]["dependency"]["files"]:
        if sha(ROOT / record["path"]) != record["sha256"]:
            raise ValueError(f"Mechanics dependency changed since execution: {record['path']}")
    provenance = read(PUBLICATION / "figure_provenance.json")
    required_ledgers = {(steps[name]["path"], steps[name]["sha256"]) for name in ("factors", "literature")}
    if {(s["path"], s["sha256"]) for s in provenance["sourceLedgers"]} != required_ledgers:
        raise ValueError("Figure ledgers differ from the completed protocol.")
    if sha(ROOT / "scripts/render_formulation_publication.py") != provenance["rendererSha256"]:
        raise ValueError("Publication renderer changed; regenerate the figures.")
    for record in provenance["sourceLedgers"] + provenance["outputs"]:
        path = (ROOT / record["path"]).resolve()
        if not path.is_relative_to(ROOT) or sha(path) != record["sha256"]:
            raise ValueError(f"Figure checksum mismatch: {record['path']}")
    paired = read(PUBLICATION / "paired_summary.json")
    if paired["state"] != "complete" or paired["pairedPackets"] != 18:
        raise ValueError("Expected 18 matched archived packets.")
    return completion, reports["factors"], reports["literature"], reports["derivatives"], paired


def replace_once(source: str, pattern: str, replacement: str) -> str:
    result, count = re.subn(pattern, lambda _: replacement, source, flags=re.S)
    if count != 1:
        raise ValueError(f"Manuscript section marker is not unique: {pattern}")
    return result


def replace_results_section(source: str, tail: str) -> str:
    # The paired float precedes the replaced section. Remove its old copy
    # explicitly so a second update cannot leave two figures with one label.
    figure_pattern = r"\\begin\{(figure\*?)\}.*?\\end\{\1\}"
    for block in reversed(list(re.finditer(figure_pattern, source, flags=re.S))):
        if r"\label{fig:paired}" in block.group():
            source = source[:block.start()] + source[block.end():].lstrip("\n")
    source = replace_once(source, r"\\section\{Simulation and Comparison Protocols\}.*?(?=% Author, affiliation)", tail)
    # Declare wide floats before their discussion: the double-column
    # template normally queues them until the following page.
    placements = (
        ("fig:paired", r"\section{Simulation and Comparison Protocols}"),
        ("fig:factors", r"\subsection{Factors, scoring, and provenance}"),
        ("fig:magnitude", r"\subsection{Multicontact force separation}"),
        ("fig:uncertainty", r"\subsection{Factor ablations and local uncertainty}"),
    )
    for label, marker in placements:
        blocks = list(re.finditer(figure_pattern, source, flags=re.S))
        matching = [b for b in blocks if "\\label{" + label + "}" in b.group()]
        if len(matching) != 1 or source.count(marker) != 1:
            raise ValueError(f"Figure placement marker is not unique: {label}")
        block = matching[0]
        figure = block.group()
        source = source[:block.start()] + source[block.end():]
        source = source.replace(marker, figure + "\n\n" + marker, 1)
    return source


def update_project_docs(completion: dict, factors: dict, baselines: dict, derivatives: dict,
                        nominal_rows: list, covered: int, eligible: int, width: float) -> None:
    """Keep the entry points and detailed code map tied to the same release."""
    checks = next(s["caseCount"] for s in completion["completedSteps"] if s["name"] == "engineering")
    means = {(scene, method): contact for scene, method, contact, _, _ in nominal_rows}
    summary = ["## 当前结果", "",
               "最新完整三维实验及逐种子表见[完整软件实验报告](docs/PUBLICATION_RESULTS.md)。下面来自同一个冻结版本；历史二维原型、单种子窗口和连续视频分别保留自己的来源。", "",
               "| 协议 | 本版本结果 |", "|---|---|",
               f"| 完整三维因素矩阵 | {len(factors['cases'])} 次：2 场景 × 3 种子 × 3 噪声 × 6 方法；推断异常 {factors['failureCount']} |",
               f"| 同观测文献适配 | {len(baselines['cases'])} 次点载荷/Gaussian 基线，18 个字节相同的输入/真值包；推断异常 {baselines['failureCount']} |",
               f"| 三接触，标称噪声 | 三种子接触向量 RMSE 均值：EnFiRCE {means[('three_contact','full')]:.3f} N，Point {means[('three_contact','point')]:.3f} N，Gaussian {means[('three_contact','gaussian')]:.3f} N |",
               f"| 空间摩擦双接触，标称噪声 | EnFiRCE {means[('spatial_sliding','full')]:.3f} N，Point {means[('spatial_sliding','point')]:.3f} N，Gaussian {means[('spatial_sliding','gaussian')]:.3f} N |",
               f"| 局部 95% 区间 | 含噪完整法 {covered}/{eligible} 个可计分力分量覆盖；平均全宽 {width:.3f} N；条件诊断，未作全局校准声明 |",
               f"| 工程检查与归档重放 | {checks}/{checks} 通过；包括 JSON 恢复、搜索域、导数和原始传感器重放 |",
               f"| 求导对照 | 2 次顺序冷启动；ODE {derivatives['cases'][0]['solver']['mechanicalEvaluations']} → {derivatives['cases'][1]['solver']['mechanicalEvaluations']}，最大力差 {derivatives['maxForceDifferenceN']:.3g} N |", "",
               "接触力、末端力与总合力分别计分；各方法的非正退出、review 和不利结果在报告中完整列出。三个种子重复传感器噪声，而非独立机器人轨迹。因素计时共享并行资源，不能作为独立速度排名。对比是文献思想适配，未运行官方完整因子图系统，当前不宣称 SOTA。", "",
               "发布脚本回归 8/8 通过：论文重复更新保持字节一致；重复完成步骤、未完成案例及缺失检查不能通过发布核验。当前缺陷与 SOTA 证据边界见[技术说明 26.3–26.4](docs/TECHNICAL_OVERVIEW.md)。", "",
               "完整三维路径从形状与环境生成候选，联合优化各帧 Cosserat 平衡、未知接触弧长、力、环境参数和摩擦历史；见[完整 formulation 工作流](docs/FORMULATION_WORKFLOW.md)。它是直接非线性窗口 MAP，递归预测后验/迭代 EKF 的原式仍单独列为差异。真实传感器尚未接入。", "",
               "历史 396 次二维多接触及 54 次平面偏移实验见[二维软件报告](docs/SOFTWARE_BENCHMARK_2026-09-30.md)；历史七窗口与缓存结果见[文献比较说明](docs/LITERATURE_COMPARISON.md)。这些历史记录未改写成当前源码结果。", "", ""]
    readme = ROOT / "README.md"
    content = readme.read_text(encoding="utf-8")
    content = replace_once(content, r"## 当前结果\n.*?(?=## 公开视频与演示)", "\n".join(summary))
    content = content.replace("但大体积 MAT 和逐帧日志保留在本地；精选视频与结果摘要随仓库发布。",
                              "完整投稿协议的精选输入/真值/估计 MAT、CSV、完成记录与图源随仓库发布；其他中间数据和逐帧日志保留在本地。")
    content = content.replace("最新的[文献基线与缓存技术报告]", "历史版本的[文献基线与缓存技术报告]")
    readme.write_text(content, encoding="utf-8")
    status = ["# 当前状态", "", "更新：2026-10-04。", "",
              "EnFiRCE 是利用稀疏形状和环境信息估计连续体机器人接触力及独立末端力的 MATLAB 仿真研究实现。", "",
              f"本版本完整协议 `{completion['runRecord']['runId']}` 已完成：{len(factors['cases'])} 项完整三维因素运行、{len(baselines['cases'])} 项同输入文献适配、2 项顺序导数对照、{checks} 项工程检查。全部原始文件及图表按源码/数据 SHA 对应，详细数值、复核标志、退出状态和消融定义见[完整软件实验报告](PUBLICATION_RESULTS.md)。", "",
              "## 本轮代码改进", "",
              "- 候选跨帧关联、全长有序接触位置与稳定维数的整杆非穿透。",
              "- 每时刻非线性 Cosserat、真实前驱同一材料点摩擦、共享潜在平面和独立末端力继续保留。",
              "- 目标/约束/协方差共享同一中央差分矩阵；退化活动分支恢复保留弱观测项，再优化原 MAP；分支约束容差统一至精确同伦目标，并保存停止消息及一阶最优性。",
              "- 恢复步骤重新计算固定物理/弱 MAP 残差平方范数，只接受没有变差的返回点，防止失败恢复覆盖原初值；非法试探局部拒绝，真实编程错误继续抛出。",
              "- JSON 往返的数组方向/字段顺序不再误报源码或依赖改变；真实校验值改变仍拒绝恢复。",
              "- 因素单线程与历史归档默认线程分开调度；同一旧 wall 输入在默认线程下状态/合力差均为 0，保留严格重放阈值与实际执行脚本归档。",
              "- 六个独立场景/种子目录并行求解并校验合并；图、论文、网站只接收完整数据。", "",
              "- 修复论文重复更新时旧比较图残留、导致标签重复而中断的问题。发布核验增加步骤唯一性、逐案例完成状态/数量及工程检查非空/数量检查；8 项 Python 回归通过，真实稿件连续更新两次字节相同。此次未修改 MATLAB 求解器或实验数值。", "",
              "## 阅读和复现", "",
              "- [完整技术说明](TECHNICAL_OVERVIEW.md)：所有模块、公式、接口和代码地图。",
              "- [PDF 公式逐项对应](FORMULATION_WORKFLOW.md)：包括保留内容及直接窗口 MAP 与递归形式的差异。",
              "- [因素实验实现](FORMULATION_FACTORS.md)：删除因素的精确定义、局部区间及发布流程。",
              "- [场景与视频](SCENARIO_MATRIX.md)：真实完整状态数、运动方向和历史求解范围。", "",
              "```text", "python scripts/run_publication_parallel.py --jobs 6",
              "python scripts/render_formulation_publication.py",
              "python scripts/update_publication_manuscript.py",
              "python scripts/compile_publication_manuscript.py",
              "python scripts/sync_publication_website.py", "```", "",
              "MATLAB 默认顺序入口为 `force('publication')`。执行代码必须与已有记录相同，否则使用新目录；并行计时不解释为隔离实时性能。", "",
              "## 证据范围", "",
              "连续 MP4 保留原先实际 12 个平衡状态：单接触为三维旧估计器，多接触为已知顺序/数量的平面原型。新统计为完整三维两状态 MAP；二者分别注明，未把旧视频当作新统计的完整运动实验。", "",
              "论文与网站采用本完整版本的统计，适配基线与官方系统区分。真实硬件、独立多轨迹、曲面/有限面片、接触模式混合、材料失配校准和实时递归属于尚需研究证据的范围；本页不将它们写成已完成。", ""]
    (ROOT / "docs/STATUS.md").write_text("\n".join(status), encoding="utf-8")
    technical = ROOT / "docs/TECHNICAL_OVERVIEW.md"
    content = technical.read_text(encoding="utf-8")
    section = ["## 26. 当前完整软件实验与交付", "",
               f"当前完整协议记录 `{completion['runRecord']['runId']}` 保存 {len(completion['runRecord']['source'])} 个 MATLAB 源文件指纹。{len(factors['cases'])} 个因素组合、{len(baselines['cases'])} 个同输入基线、2 个导数组合和 {checks} 项工程检查已完成；具体数值、每种消融、非正退出和 review 见[完整软件实验报告](PUBLICATION_RESULTS.md)。", "",
               "### 26.1 从入口到可复算结果", "",
               "| 阶段 | 代码 | 输入与输出 |", "|---|---|---|",
               "| 调度 | `force('publication')` / `run_publication_parallel.py` | 冻结源码、固定实验矩阵 → 四步完成记录；并行方式增加工作进程来源记录 |",
               "| 观测实现 | `resample_formulation_observations.m` | 干净传感器包 + 种子/协方差 → 独立曲率实现；不读取参考力 |",
               "| 完整逆解 | `estimate_formulation_window.m` | 同一个观测包 → 各帧未知状态、力、局部协方差与原始质量字段 |",
               "| 文献适配 | `estimate_literature_curvature_baseline.m` | 逐字节相同包、观测候选数 → 自有初值的 Point/Gaussian 估计 |",
               "| 独立评分 | `score_formulation_window.m` / `score_formulation_coverage.m` | 完成估计后才读真值 → 向量/大小/弧长误差、有效区间分母 |",
               "| 图与表 | `render_formulation_publication.py` | 已完成 JSON/MAT/CSV 的 SHA → 真实数值图、种子范围、图源和 LaTeX 表 |",
               "| 文稿 | `update_publication_manuscript.py` | 完整四步与图源核验 → 当前结果文档、稿件与图表 |",
               "| 编译与网站 | `compile_publication_manuscript.py` / `sync_publication_website.py` | 实际 TeX/参考文献/模板/图表 SHA → 对应 PDF、编译记录及真实 MP4 字节副本 |", "",
               "### 26.2 适合发送给学长的材料", "",
               "先发送完整软件实验报告、同输入比较/消融/逐接触力大小图，再发送本技术说明和 `FORMULATION_WORKFLOW` 的逐式对应。需要复算时附 `input.mat/truth.mat/estimate.mat/forces.csv/comparison.json`。视频提供运动直观理解，统计图提供可配对的力精度；它们的求解版本与状态数分别说明。", "",
               "讨论重点仍是实际杆刚度与高反力是否合理、环境观测精度、怎样构造信息充分的摩擦转变、末端/接触力真值获取以及官方图方法的输入匹配。已有不利结果也随报告提供。完成软件流程不自动构成 SOTA、硬件准确性或全局概率校准证据。", "",
               "### 26.3 当前代码问题、修复和检查范围", "",
               "2026-10-04 的后续排错检查了候选生成/跨帧关联、有序弧长域、状态初始化、每帧积分缓存、前驱材料点位移、共享导数、分支恢复、局部协方差、评分及发布脚本。以下是实际复现后修复的程序问题；它们不改变力学模型或已完成的实验数值。", "",
               "| 问题及原行为 | 根因与修改位置 | 修复后行为 |", "|---|---|---|",
               "| 论文第二次同步报 `fig:paired` 标签不唯一 | [update_publication_manuscript.py](../scripts/update_publication_manuscript.py) 的 `replace_results_section`：比较图曾移到章节替换范围前，旧图未被删除 | 替换前删除旧比较图，再生成和放置同一批图；真实稿件及生成文档连续运行两次，第二次字节完全相同 |",
               "| 完成步骤重复仍可通过发布核验 | [verify_publication_release.py](../scripts/verify_publication_release.py) 的 `verify`：按名字构造字典会静默覆盖重复项 | 同时校验四个必需名字和列表/字典数量相等，拒绝重复步骤 |",
               "| 未完成基线记录仍可通过核验 | 同一 `verify`：原先只检查账本总状态与异常数 | 每项必须 `completed=true`，案例总数必须与完成步骤声明相符 |",
               "| 空工程检查列表仍可被解释为全部通过 | 同一 `verify`：Python 的 `all([])` 为真 | 要求检查列表非空、数量匹配、状态完成且逐项通过 |", "",
               "[test_publication_serialization.py](../scripts/test_publication_serialization.py) 复用现有检查文件，包含 3 个 JSON 往返身份用例、2 个文稿替换用例、3 个完成状态用例，共 8/8 实际通过。错误完成状态只在内存中注入，原始实验文件未改写；真实发布核验仍通过全部 589 个源码/数据/图文件。36 项 MATLAB 工程检查来自同一冻结源码的既有完成记录，本轮校验其字节及逐项通过状态，未将它们写成新执行。", "",
               "本轮未发现需要修改当前完整方法的新增力学实现错误，但这只描述已审查范围，不是任意输入均无缺陷的证明。已归档完整方法 18 个窗口均为正退出；3 个旧分区消融窗口和 2 个 Gaussian 窗口的非正退出继续保留，不能改退出码或删除后声称所有方法收敛。首次空间状态缺少真实前驱时的 review 也保留。", "",
               "复杂度控制：仍使用原来的 Cosserat 积分、`lsqnonlin/fmincon`、联合窗口和图表流程，没有新增估计器、候选枚举策略或实验框架。论文放置逻辑提取成一个小函数用于回归；发布核验只增加直接条件。后续修复应以可复现错误为依据，不用增加模型分支代替诊断。", "",
               "### 26.4 为什么仍不能宣称 SOTA", "",
               "现在能成立的结论是：在两条指定的两状态仿真配置、标称曲率噪声和三个传感器种子下，完整方法的接触向量 RMSE 均值为三接触 0.215 N、空间滑动 0.288 N，均优于这里实现的 Point/Gaussian 适配。这不是对所有论文和场景的排名。", "",
               "| 尚缺的比较依据 | 与程序 bug 的区别 |", "|---|---|",
               "| 原作者完整算法或官方实现的复现 | 当前是自实现的点载荷和 Gaussian 思想适配；修好发布脚本不会将它们变成原方法 |",
               "| 匹配信息预算的对照 | 输入包字节相同，但 EnFiRCE 使用环境/时间信息，两个基线只拟合曲率；该实验比较完整估计器，不能将差距全部归因于优化算法 |",
               "| 独立轨迹、形状、几何和接触模式的泛化 | 三个种子只重复噪声，两状态配置固定；干净种子重复不增加独立轨迹数量。有限候选和已选分支也不保证任意接触拓扑全局最优 |",
               "| 成本和任务范围匹配 | 现有完整调用以秒至分钟计，尚非实时；真实硬件和全局不确定性校准未完成。硬件并非任何软件排名的必需条件，但限制了对真实机器人性能的声明 |", "",
               "既有反例也不能回避：历史三接触总合力 RMSE 曾为 EnFiRCE 0.820 N、Point 0.632 N；当前因素消融并非每个删除项都显著变差。接触分离、末端力、总合力、区间与速度必须分别报告。更大范围的优势需要相应实验依据，不能通过修复 bug、换一个指标或过滤不利结果产生。", ""]
    if "## 26. 当前完整软件实验与交付" in content:
        content = content.split("## 26. 当前完整软件实验与交付")[0]
    technical.write_text(content.rstrip() + "\n\n" + "\n".join(section), encoding="utf-8")


def update() -> None:
    completion, factors, baselines, derivatives, paired = evidence()
    runtime = read(PUBLICATION / "runtime_host.json")
    processor = runtime["processor"].replace("(R)", "").replace("(TM)", "")
    runtime_description = (f"{processor} ({runtime['physicalCores']} physical cores, "
                           f"{runtime['logicalProcessors']} logical processors), "
                           f"{runtime['visibleMemoryGiB']:.2f} GiB visible RAM, Windows 11; "
                           f"MATLAB {completion['runRecord']['matlabVersion']}")
    groups = {(g["scene"], g["noiseStdPerMm"], g["method"]): g for g in paired["groups"]}
    nominal = 2.5e-5

    def mean(scene: str, method: str, metric: str = "contactRmseN") -> float:
        value = groups[(scene, nominal, method)][metric]
        if value is None or value["eligible"] != 3:
            raise ValueError("A nominal paired metric is missing; prose needs revision.")
        return value["mean"]

    nominal_rows = [(scene, method, mean(scene, method), mean(scene, method, "tipRmseN"),
                     mean(scene, method, "totalRmseN"))
                    for scene in SCENES for method in ("full", "point", "gaussian")]
    contact_wins = sum(mean(scene, "full") < mean(scene, method)
                       for scene in SCENES for method in ("point", "gaussian"))
    total_wins = sum(mean(scene, "full", "totalRmseN") < mean(scene, method, "totalRmseN")
                     for scene in SCENES for method in ("point", "gaussian"))
    full = [c for c in factors["cases"] if c["method"] == "full"]
    clean = [c["metrics"]["contactForceRmseN"] for c in full if not c["noiseStdPerMm"]]
    noisy_full = [c for c in full if c["noiseStdPerMm"]]
    covered = sum(c["coverage"]["coveredComponents"] for c in noisy_full)
    eligible = sum(c["coverage"]["eligibleComponents"] for c in noisy_full)
    unresolved = sum(c["coverage"]["unresolvedComponents"] for c in noisy_full)
    widths = [(c["coverage"]["meanIntervalWidthN"], c["coverage"]["eligibleComponents"])
              for c in noisy_full if c["coverage"]["meanIntervalWidthN"] is not None]
    interval_width = sum(w * n for w, n in widths) / sum(n for _, n in widths) if widths else float("nan")
    review_full = sum(c["metrics"]["reviewCount"] for c in full)
    missing_history_review = sum(
        not history and review
        for c in full if c["sceneId"] == "spatial_sliding"
        for history, review in zip(c["quality"]["frictionHistoryAvailable"], c["quality"]["requiresReview"]))
    review_factors = sum(c["metrics"]["reviewCount"] for c in factors["cases"])
    review_baselines = sum(c["metrics"]["reviewCount"] for c in baselines["cases"])
    exit_factors = sum(nonpositive_exit(c["solver"].get("exitflag")) for c in factors["cases"])
    exit_baselines = sum(nonpositive_exit(c["solver"].get("exitflag")) for c in baselines["cases"])
    separate, shared = derivatives["cases"]
    full_times = np.array([c["wallSeconds"] for c in full])
    reused_calls = len(factors.get("execution", {}).get("reusedSequentialCaseIds", []))
    factor_groups = defaultdict(list)
    for c in factors["cases"]:
        factor_groups[(c["sceneId"], c["noiseStdPerMm"], c["method"])].append(c)

    def factor_mean(scene: str, method: str) -> float:
        values = [c["metrics"]["contactForceRmseN"] for c in factor_groups[(scene, nominal, method)]]
        if any(v is None for v in values):
            raise ValueError("An ablation has unmatched counts; adapt the manuscript rather than hiding it.")
        return float(np.mean(values))

    factor_sentence = "; ".join(
        f"{label}: {factor_mean('three_contact', method):.3f}/{factor_mean('spatial_sliding', method):.3f} N"
        for method, label in (("no_temporal", "no temporal prior"), ("cone_only", "cone only"),
                              ("no_geometry", "no contact geometry"), ("no_camera", "no camera likelihood")))
    table = ["# 完整三维软件实验与论文结果", "", "更新：2026-10-04。", "",
             "本轮仍然研究：从稀疏形状与环境信息分离杆身接触力和独立末端力。全部统计来自完整三维 Cosserat 时间窗口，历史二维结果单独保留。", "",
             f"完整协议 run `{completion['runRecord']['runId']}`：{len(factors['cases'])} 次因素运行、{len(baselines['cases'])} 次同输入基线、2 次导数对照、{completion['completedSteps'][-1]['caseCount']} 项工程检查全部完成。因素/基线异常为 {factors['failureCount']}/{baselines['failureCount']}，非正最终退出窗口为 {exit_factors}/{exit_baselines}；复核帧为 {review_factors}/{review_baselines}。", "",
             "## 1. 实验究竟重复了什么", "",
             "两个固定两帧平衡：蛇形三接触、真正含出平面变形的摩擦滑动双接触。每帧 24 个位置、两个实际弯曲曲率通道。噪声 SD 为 0、2.5e-5、5e-5 /mm；每种条件的传感器种子为 11、23、37。无噪声的三个种子是相同观测的重复控制，并非三条独立轨迹。两个含噪等级共有 12 个含噪输入包；同一种子跨噪声等级复用标准化噪声，仅改变幅度，两个同维数场景也复用该随机流。每个固定条件内的三个种子是独立重复，跨条件的包不是全局独立样本。环境观测本轮保持固定，不把固定环境下的局部覆盖率称为含环境失配的覆盖率。", "",
             "六种方法各从自己的形状初值开始，不用完整方法后验初始化消融。点载荷/Gaussian 基线读取字节相同的输入和真值归档；真值仅在求解结束后评分。数量来自观测候选，比较条件化于候选数，不是未知数量检测比赛。", "",
             "## 2. 同输入的力估计", "",
             "标称曲率噪声下，下面是三个种子的窗口向量 RMSE 均值（N）。窗口内部对所有匹配接触与两帧计算 RMS；均值是窗口 RMSE 的均值。总力为杆身各接触力与末端力的向量和。", "",
             "| 场景 | 方法 | 接触力 | 末端力 | 总合力 |", "|---|---|---:|---:|---:|"]
    for scene, method, contact, tip, total in nominal_rows:
        table.append(f"| {SCENES[scene]} | {LABELS[method]} | {contact:.6g} | {tip:.6g} | {total:.6g} |")
    table += ["", f"接触力均值：EnFiRCE 在 {contact_wins}/4 个场景/基线组合中较小；合力均值是 {total_wins}/4。分别报告这些指标，不能用一个指标的优势代替所有指标的优势。三种子的最小/最大值用作范围，未声称统计显著。", "",
              "### 2.1 每个接触究竟估计了多少力", "",
              "标称噪声，每项按相同状态和弧长顺序配对；以下是力向量的大小，不能代替上面的向量误差。范围是三个噪声种子的最小/最大估计，绝对误差是三个大小误差的均值。", "",
              "| 场景 | 状态 | 接触 | 真值 /N | 估计均值 /N | 种子范围 /N | 大小绝对误差均值 /N |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for scene in SCENES:
        windows = [c for c in full if c["sceneId"] == scene and c["noiseStdPerMm"] == nominal]
        for frame_index, frame in enumerate(windows[0]["metrics"]["perFrame"]):
            true_forces = np.atleast_1d(frame["trueForceN"])
            estimates = np.array([np.atleast_1d(c["metrics"]["perFrame"][frame_index]["estimatedForceN"]) for c in windows])
            if estimates.shape != (3, len(true_forces)):
                raise ValueError("Magnitude table has unmatched active contact counts.")
            for contact, truth in enumerate(true_forces):
                values = estimates[:, contact]
                table.append(f"| {SCENES[scene]} | {frame_index+1} | {contact+1} | {truth:.6g} | {values.mean():.6g} | {values.min():.6g}–{values.max():.6g} | {np.mean(np.abs(values-truth)):.6g} |")
    table += ["",
              "![同观测对比](../out/benchmarks/publication/figures/matched_baselines.png)", "",
              "![逐接触力大小](../out/benchmarks/publication/figures/contact_magnitudes.png)", "",
              "力大小图显示每帧、每个接触的独立真值及三个种子的估计。约 134 N 的反力来自项目当前合成刚度与变形，不代表已测量的机器人载荷范围。比较其他论文的 N 级误差必须同时匹配载荷范围、传感器、真值和误差定义。", "",
              "## 3. 完整三维逐因素消融", "",
              "| 场景 | 噪声 SD /mm | 方法 | 接触 RMSE 均值 /N | 总力 RMSE 均值 /N | 复核帧 | 非正退出窗口 |", "|---|---:|---|---:|---:|---:|---:|"]
    for (scene, sigma, method), cases in sorted(factor_groups.items()):
        errors = [c["metrics"]["contactForceRmseN"] for c in cases if c["metrics"]["contactForceRmseN"] is not None]
        value = f"{np.mean(errors):.6g}" if errors else "数量不匹配"
        table.append(f"| {SCENES[scene]} | {sigma:g} | {LABELS[method]} | {value} | {np.mean([c['metrics']['totalForceRmseN'] for c in cases]):.6g} | {sum(c['metrics']['reviewCount'] for c in cases)} | {sum(nonpositive_exit(c['solver'].get('exitflag')) for c in cases)} |")
    table += ["", "![逐因素精度](../out/benchmarks/formulation_factors/figures/factor_accuracy.png)", "",
              "`no_temporal` 仅移除过程先验；`cone_only` 移除前驱位移/最大耗散互补，保留摩擦锥；`no_geometry` 移除接触间隙、切向与整杆非穿透，保留法向/摩擦参数化；`no_camera` 仅移除环境似然；`legacy_partitions` 恢复首帧固定弧长分区。后两者仍保留其余因子和观测生成初值。某个场景没有触及旧分区时，两种弧长范围结果可以相同；这不能作为全长搜索提升精度的证据。代码的跨分区迁移回归用构造位置验证搜索域，未冒充独立物理轨迹实验。", "",
              "## 4. 局部区间与计算代价", "",
              f"完整方法的含噪窗口共有 {covered}/{eligible} 个可计分世界坐标力分量落入局部 95% 区间，另有 {unresolved} 个未辨识分量；可计分区间平均全宽为 {interval_width:.6g} N。分母包括匹配的活动接触力与末端力，不把无穷标准差计为覆盖成功；无噪声重复控制不计覆盖。相邻帧和各分量相关，12 个噪声实现只支持本固定轨迹/模式的条件诊断，不能声称全局校准。完整方法所有 18 个窗口共 {review_full} 帧需复核，其中 {missing_history_review} 个空间首帧同时缺少真实摩擦前驱观测，保留静态锥并明确警告，不能伪造历史或把警告当作优化失败。", "",
              "![局部覆盖与时间](../out/benchmarks/formulation_factors/figures/coverage_runtime.png)", "",
              f"实际运行硬件：{runtime_description}。包含局部协方差的完整窗口调用：中位数 {np.median(full_times):.3f} s，范围 {full_times.min():.3f}–{full_times.max():.3f} s。因素矩阵按场景/种子分成六个单计算线程 MATLAB 进程，最多六个并行；{reused_calls} 个先前完成的组合保留其串行耗时。因此本矩阵的 wall time 是共享机器资源下的运行成本，不能当作隔离延迟，也不能据此对不同方法进行实时性能排名。36 个文献基线由先前单线程顺序阶段完成并原样复用。导数对照和历史回归重放采用 MATLAB 默认线程模式；旧 wall 重放在相同模式下保持原始严格容差。本轮导数对照在全部并行进程退出后，同机、相同冷启动、同协方差开关顺序执行：分开求导 {separate['seconds']:.3f} s，共享求导 {shared['seconds']:.3f} s，ODE {separate['solver']['mechanicalEvaluations']} → {shared['solver']['mechanicalEvaluations']} 次；最大力差 {derivatives['maxForceDifferenceN']:.6g} N，最大弧长差 {derivatives['maxArcDifferenceMm']:.6g} mm。只做一次、固定先后顺序，不作延迟显著性或实时声明。", "",
              "## 5. 本轮修复了什么", "",
              "- 逐帧合并、跨帧关联候选，避免同一接触迁移被误生成为多个槽。",
              "- 全长有序弧长替代默认首帧固定分区，保留最小间距。",
              "- 目标、约束与协方差共享同状态的中央差分矩阵，按帧复用力学，不近似替换平衡。",
              "- 退化活动分支恢复时用弱原 MAP 残差防止观测形状漂移，再优化原 MAP；只在分支精化删除盒约束已覆盖的冗余行和恒等零行，最终审核原始互补条件。",
              "- 分支约束容差与精确同伦目标统一为 1e-8，修正干净三接触点被 1e-9 内层容差与 1e-8 步长条件误拒绝的问题；保留原始审计和真实退出码，额外记录停止消息与一阶最优性。",
              "- 区分非法优化试探与程序错误；非有限/越界试探可拒绝，真实维度错误继续抛出，避免 NaN 弧长进入 ODE。",
              "- 整杆碰撞包含连续接触位置，评分只匹配活动接触，不把零力槽算成真接触。",
              "- 恢复器重新计算固定残差的真实平方范数，只接受未变差的恢复点；不再让失败的最小二乘返回点覆盖原初值。原失败输入重放已确认这一保护恢复原 MAP 拟合。",
              "- MATLAB 恢复保存的单项 struct 数组/对象、缺失覆盖诊断 null/[] 按明确字段规范化；所有数值、顺序、标志与文件 SHA 保持精确比较，三个 Python 回归检查覆盖合法往返、末位数值篡改和未声明字段。", "",
              "- 数据、源码与发布文件保持字节校验，所有图从已完成数据生成。", "",
              "## 6. 可以向学长展示与需要讨论的事", "",
              "可以直接展示：本报告的配对表和力大小图、完整三维六因素图、技术总说明第 25–26 节，以及真实 12 状态 MP4。一起发送输入/真值/估计 MAT、forces.csv 和 comparison.json，可以逐项复算。", "",
              "需要讨论：当前合成刚度导致反力较大，是否符合实际杆参数；无限平面与材料曲率校准是否符合学长的系统；哪些摩擦轨迹能真正区分完整历史模型与静态锥；末端力与接触力需要哪些硬件真值；官方因子图方法的输入如何匹配。", "",
              "已有反例也要展示：历史遗漏墙面的总力误差 17.834/15.964 N，摩擦系数失配 1.647 N，二维 8 点观测失准，且某些指标/消融可能打平或优于完整法。这些是失配/信息条件边界，不应从网站或稿件隐藏。", "",
              "## 7. SOTA 与论文状态", "",
              "两种比较是公开源码的文献思想适配，不是原作者官方完整系统。BENDIER 的官方 RAL 标签已经查明，但依赖 GTSAM、Eigen 与特定图观测；本轮未运行它。Prakash 等 2026 年的多接触图输入还包含末端位置/腱信息，其发表数字不可直接排行。当前可以报告本数据和给定改编基线上的结果，不能宣布 SOTA。", "",
              "论文目前是无作者信息的仿真稿，硬件校准、独立多轨迹、接触模式混合、材料/环境失配覆盖和实时运行仍需研究证据。软件流程完成并不使这些证据自动成立。", "",
              "## 8. 复现与文件", "",
              "```matlab", "addpath('rod'); addpath(genpath('LCP-Continuum'));", "force('publication');", "```", "",
              "```text", "python scripts/render_formulation_publication.py", "python scripts/update_publication_manuscript.py",
              "python scripts/compile_publication_manuscript.py", "python scripts/sync_publication_website.py", "```", "",
              "- [完整协议状态](../out/benchmarks/publication/completion.json)",
              "- [108 次因素原始记录](../out/benchmarks/formulation_factors/comparison.json)",
              "- [36 次基线原始记录](../out/benchmarks/formulation_literature/comparison.json)",
              "- [配对绘图源 CSV](../out/benchmarks/publication/paired_source_data.csv)",
              "- [力大小源 CSV](../out/benchmarks/publication/contact_magnitude_source_data.csv)",
              "- [图表校验记录](../out/benchmarks/publication/figure_provenance.json)",
              "- [论文 PDF](../out/benchmarks/publication/EnFiRCE_draft.pdf)",
              "- [论文编译来源记录](../out/benchmarks/publication/manuscript_build.json)",
              "- [导数性能对照](../out/benchmarks/formulation_derivatives/comparison.json)",
              "- [总技术文档](TECHNICAL_OVERVIEW.md)", ""]
    (ROOT / "docs/PUBLICATION_RESULTS.md").write_text("\n".join(table), encoding="utf-8")
    update_project_docs(completion, factors, baselines, derivatives, nominal_rows, covered, eligible, interval_width)

    paper = ROOT / "EnFiRCE_overleaf"
    for name in ("method_overview", "matched_baselines", "contact_magnitudes", "local_uncertainty"):
        shutil.copyfile(PUBLICATION / "figures" / f"{name}.pdf", paper / "figs" / f"{name}.pdf")
    for name in ("factor_accuracy", "coverage_runtime"):
        shutil.copyfile(ROOT / "out/benchmarks/formulation_factors/figures" / f"{name}.pdf", paper / "figs" / f"{name}.pdf")
    shutil.copyfile(PUBLICATION / "paired_table.tex", paper / "figs/paired_table.tex")
    source = (paper / "main.tex").read_text(encoding="utf-8")
    abstract = rf"""\begin{{abstract}}
Sparse shape observations may admit several external-load explanations when a continuum robot experiences body contacts and an independent tip load. We present \EnFiRCE, a joint-window maximum-a-posteriori estimator combining measured bending curvature, uncertain environment geometry, and friction history. Every time retains nonlinear three-dimensional Cosserat equilibrium. Contacts on the same plane share its latent parameters at each time, and friction displacement compares the same material coordinate in successive equilibria. We evaluate two fixed two-state multicontact configurations using three sensor seeds, three noise levels, and six factor settings: {len(factors['cases'])} full-window runs, followed by {len(baselines['cases'])} matched point/Gaussian literature adaptations on byte-identical observation packets. At nominal noise, seed-mean contact-vector RMSEs are {mean('three_contact','full'):.3f} and {mean('spatial_sliding','full'):.3f} N; corresponding point-load errors are {mean('three_contact','point'):.3f} and {mean('spatial_sliding','point'):.3f} N. Contact, tip, and resultant metrics, review states, branch-conditional uncertainty, and computational costs are reported separately. Shared derivatives reduce duplicated mechanics evaluations while preserving the estimate within a recorded numerical tolerance. The controlled simulations quantify individual force separation under repeated sensor noise, with inspectable factor ablations and branch-conditional diagnostics.
\end{{abstract}}"""
    source = replace_once(source, r"\\begin\{abstract\}.*?\\end\{abstract\}", abstract)
    old_eval = r"The evaluation retains.*?(?=\\begin\{figure\*\})"
    new_eval = r"""The evaluation retains independent truth equilibria, observation packets, raw estimates, and source records. A completed protocol runs six full-window factor settings over two configurations, three sensor seeds, and three noise levels, then evaluates matched curvature-only adaptations. Separate historical density and mismatch experiments expose failure boundaries. Figure~\ref{fig:method} summarizes the estimator; Fig.~\ref{fig:geometry} shows actual solved configurations. The implementation is an offline simulation method.

\begin{figure}[t]
\centering
\includegraphics[width=\columnwidth]{method_overview.pdf}
\caption{Factor structure illustration. Previous and current shapes are latent equilibria. Measured bending channels and environment likelihoods enter a joint MAP; friction compares the same material coordinate across times. Forces and diagnostics are estimator outputs.}
\label{fig:method}
\end{figure}

"""
    source = replace_once(source, old_eval, new_eval)
    source = source.replace("Mode mixtures, calibration uncertainty, and empirical coverage are not marginalized or validated.",
                            "Mode mixtures and calibration uncertainty are not marginalized. Empirical component coverage is evaluated only for repeated sensor noise on fixed configurations and a local branch.")
    tail = rf"""\section{{Simulation and Comparison Protocols}}
\subsection{{Independent truth and repeated observations}}
An independent continuous shooting solver generates equilibria before sparse sampling. The inverse receives no true arcs, forces, dense shape, or mode labels. A 210 mm alternating-curvature rod forms three contacts between planes; a spatial two-contact configuration includes out-of-plane loading, $\mu=0.03$, and tip force $[0.1,0.3,-0.08]^{{\mathsf T}}$ N. Base sliding by 0.5 mm along $+y$ induces opposing friction. This is continuous sliding, not a stick-to-slip experiment. Archived solved geometry in Fig.~\ref{{fig:geometry}} illustrates the fixture family.

Bending stiffness is 200700 N\,mm$^2$ and torsional stiffness $200700/1.3$ N\,mm$^2$. Radii 0.455/0.66 mm are calibration metadata, without radius collision correction. Reactions near 134 N arise from this synthetic stiffness and imposed deformation (Fig.~\ref{{fig:magnitude}}), not a measured hardware range.

Each configuration has two states with two bending channels at 24 positions. Curvature noise SDs are $0$, $2.5\times10^{{-5}}$, and $5\times10^{{-5}}$ mm$^{{-1}}$; clean inputs retain a $10^{{-7}}$ mm$^{{-1}}$ likelihood floor. Seeds 11, 23, and 37 provide independent draws within each fixed condition. The same standardized draw is scaled across noise levels and reused across equal-size configurations, enabling common-random-number pairing rather than independent cross-condition repetitions. Clean observations are identical. The 18 packets comprise 12 noisy inputs and six clean controls, not independent trajectories. Plane observations are fixed across draws, with point/normal-component SDs of 0.05 mm/0.001 in the likelihood.

\subsection{{Same-curvature literature adaptations}}
Both baselines use full body-frame equations
\begin{{align}}
 h'&=-u\times h-e_3\times n,\notag\\
 n'&=-u\times n-q_\ell,\qquad u=u_0+B^{{-1}}h,
 \label{{eq:body}}
\end{{align}}
where $h=R^{{\mathsf T}}m$, $n=R^{{\mathsf T}}N$, and $h(L)=0$. Backward integration avoids an inner base-moment shooting loop. Forward integration supplies world forces and shape. Point LS fits three unknown local components per contact, inspired by~\cite{{xiao2021efficient}} and generalized beyond its straight-rod specialization. Gaussian LS adapts Aloi's transverse parameterization~\cite{{aloi2022estimating}}:
\begin{{equation}}
 q_\ell(s)=\sum_j [a_{{x,j}},a_{{y,j}},0]^{{\mathsf T}}
 \frac{{e^{{-(s-c_j)^2/(2\sigma_j^2)}}}}{{A_j\sqrt{{2\pi}}\sigma_j}},
\end{{equation}}
with normalization $A_j$ over $[0,L]$, estimated width in $[0.25\text{{ mm}},L/3]$, and world force $\int Rq_{{\ell,j}}\,ds$. Its position likelihood is replaced by the same measured curvature; finite width and transverse loading differ from point-contact truth.

Each baseline restricts ordered arcs to uniform partitions and uses its own 25\%, 50\%, and 75\% initializers within them, 80 iterations per start, and the preceding baseline solution as another initializer. No EnFiRCE posterior is supplied. Candidate count alone is shared from observations, so comparison is conditional on count; these fixtures do not cross the baseline arc partitions. Baselines lack environment/process likelihoods; the experiment compares complete estimators, not one isolated factor. Weak amplitude/$10^6$ regularization remains. Local body/tip bounds are $\pm1000$/$\pm10$ N versus EnFiRCE normal/world-tip bounds of 200/$\pm25$ N. These do not clip truth. All 36 baseline runs, including review and nonpositive exits, are retained.

\subsection{{Factors, scoring, and provenance}}
Six settings independently cold-start the same packets: full method; no temporal process prior; static friction cone without displacement complementarity; no gap/tangency/nonpenetration factors; no camera likelihood; and legacy fixed arc partitions. Other factors and declared priors remain unchanged. Removing geometry retains contact normal/friction parameterization; removing camera retains geometric constraints and observation-derived initialization. There are 108 full-window calls, including local covariance.

Contact-vector RMSE is $\sqrt{{(KT)^{{-1}}\sum_{{j,k}}\|\widehat f_{{j,k}}-f_{{j,k}}\|^2}}$, using ordered active-contact matching. Tip and total metrics are vector RMSE; total is $f_e+\sum_j f_j$. Magnitude MAE and arc RMSE are separate. A count mismatch receives no fabricated correspondence. Every input/truth pair has identical bytes across matched methods; truth is loaded only after inference. Source/dependency hashes, MAT/CSV estimates, completion state, and figure source data accompany the results.

\section{{Results}}
\subsection{{Multicontact force separation}}
All 108 factor and 36 adapted-baseline calls completed without inference exceptions. Final nonpositive exits occurred in {exit_factors} factor and {exit_baselines} baseline windows; corresponding review totals were {review_factors} and {review_baselines} states, retained rather than filtered. Clean full-method contact RMSE ranged from {min(clean):.2e} to {max(clean):.2e} N: numerical consistency with model-consistent truth, not sensor accuracy.

\input{{figs/paired_table.tex}}

At nominal noise, mean contact RMSE was {mean('three_contact','full'):.3f} N for three contacts and {mean('spatial_sliding','full'):.3f} N for spatial sliding. Point LS gave {mean('three_contact','point'):.3f}/{mean('spatial_sliding','point'):.3f} N; Gaussian LS gave {mean('three_contact','gaussian'):.3f}/{mean('spatial_sliding','gaussian'):.3f} N. EnFiRCE's mean was lower in {contact_wins}/4 contact comparisons and {total_wins}/4 resultant comparisons. Figure~\ref{{fig:paired}} shows all noisy seed errors rather than only favorable averages. Component errors can cancel in the resultant, so total-force accuracy does not certify individual contact separation. Force magnitudes and their actual scale appear in Fig.~\ref{{fig:magnitude}}.

\begin{{figure*}}[!t]
\centering
\includegraphics[width=.98\textwidth]{{matched_baselines.pdf}}
\caption{{Matched observation bytes: three independent curvature draws per noise level on two fixed two-state configurations. Dots show individual window RMSEs; markers and whiskers show means and seed min/max, not confidence intervals. Hollow triangles mark nonpositive final exits, retained in the means. Point/Gaussian methods are disclosed curvature-only adaptations, conditioned on observation-derived count.}}
\label{{fig:paired}}
\end{{figure*}}

\begin{{figure}}[!t]
\centering
\includegraphics[width=\columnwidth]{{contact_magnitudes.pdf}}
\caption{{Actual force magnitudes at nominal noise: independent truth and EnFiRCE seed mean/range for each state and ordered contact. Large synthetic reactions follow the stated calibration.}}
\label{{fig:magnitude}}
\end{{figure}}

\subsection{{Factor ablations and local uncertainty}}
Figure~\ref{{fig:factors}} isolates the specified factor changes on identical packets. Nominal three-contact/spatial mean errors were {factor_sentence}. These controls do not establish uniform benefit: a removed factor may tie or outperform the full method on an individual configuration. Legacy partitions are inactive when contacts stay inside them, so a tie cannot validate migration performance. A separate engineering regression checks the corrected search domain under cross-partition migration, without claiming an additional physical experiment.

\begin{{figure*}}[!t]
\centering
\includegraphics[width=.98\textwidth]{{factor_accuracy.pdf}}
\caption{{Full nonlinear window ablations: three sensor seeds per condition, including identical clean controls. Lines show mean contact RMSE and shading shows seed min/max. The symlog ordinate is linear below 0.1 N; crosses mark observation-fit warnings and hollow triangles mark nonpositive final exits (coincident controls counted in legend). All variants independently initialize from observations. Geometry/camera ablations retain their stated remaining priors and parameterizations.}}
\label{{fig:factors}}
\end{{figure*}}

For noisy full-method windows, {covered}/{eligible} eligible world-force components fell inside local 95\% intervals with mean full width {interval_width:.3f} N; {unresolved} components were unresolved. Infinite standard deviations do not count as coverage successes. Clean controls are excluded. Components and adjacent times are correlated, and geometry observations are fixed, so this is branch-conditional evidence under sensor noise, not calibration over geometry, materials, or contact modes. Full-method windows retained {review_full} review states, including {missing_history_review} spatial initial states lacking predecessor friction observations. They use the static cone and retain the missing-history warning.

\begin{{figure}}[!t]
\centering
\includegraphics[width=\columnwidth]{{local_uncertainty.pdf}}
\caption{{Branch-conditional intervals: eligible force-component coverage and mean full width. Unresolved components and deterministic clean controls are excluded. Correlated components do not constitute independent coverage trials.}}
\label{{fig:uncertainty}}
\end{{figure}}

\subsection{{Failure boundaries and historical controls}}
Separate ordered planar experiments retain their historical implementation: 396 estimates vary density, noise, and geometric constraints; 54 replay uncertain plane offsets. At nominal noise and 24 positions, pooled contact RMSE was 0.490 versus 2.967 N for its shape-only baseline; eight positions gave 13.576 N. A 1 mm plane error gave 31.092 N under fixed geometry versus 1.341 N with a latent offset SD of 1 mm. These results support sensing/calibration boundaries, not statistics for the full 3-D method.

Corrected independent mismatch truth gave total errors of 17.834/15.964 N when a wall was hidden, and 1.647 N for true/inverse $\mu=0.03/0.01$. Noise-scaled fit checks flagged both states in each test. A warning neither recovers an absent reaction nor guarantees mismatch detection. Historical three-contact noise also gave a resultant counterexample: EnFiRCE 0.820 N versus Point LS 0.632 N. That distinct input remains available. Tip-only tests infer zero body candidates and a 0.156205 N tip force with model-consistent RMSE $6.78\times10^{{-10}}$ N.

\subsection{{Measured computational cost}}
The host was {runtime_description}. Full-method calls including covariance had median {np.median(full_times):.2f} s and range {full_times.min():.2f}--{full_times.max():.2f} s. Factors used six independent scene/seed processes, each with one computational thread; {reused_calls} earlier completed calls retain sequential timings. These shared-resource wall times are not isolated latency rankings. The 36 baseline calls retain their sequential single-thread timings. After every worker exited, a sequential clean two-contact derivative experiment used MATLAB default threads and kept mechanics, cold starts, covariance, and central-difference steps identical. Archived regression replay used its original default-thread mode and unchanged tolerances. Separate/shared derivatives took {separate['seconds']:.2f}/{shared['seconds']:.2f} s and {separate['solver']['mechanicalEvaluations']}/{shared['solver']['mechanicalEvaluations']} ODE evaluations; maximum force/arc changes were {derivatives['maxForceDifferenceN']:.2e} N/{derivatives['maxArcDifferenceMm']:.2e} mm. One fixed-order trial does not establish timing significance or real-time operation. An earlier exact-cache ablation, without covariance, reduced 116.29 to 50.75 s with zero force difference.

\section{{Discussion and Conclusion}}
EnFiRCE separates body reactions and an independent tip load through full equilibria, shared uncertain planes, and previous-equilibrium friction. Repeated noisy observations and factor controls support bounded multicontact comparisons with the two disclosed adaptations. They do not reproduce official probabilistic graphs or establish state-of-the-art performance. Additional observations in published systems preclude ranking their reported errors against these simulations.

Finite candidates and temporal association may miss reactions or confuse identities. Full-length ordered arcs remove an initialization restriction without enumerating topology. Infinite half-spaces omit curved/finite surfaces and rod-radius corrections. Gravity, shear, extension, material/actuation uncertainty, mode mixtures, and an independent stick-to-slip event lie outside the present nominal evidence. Local covariance and review flags remain diagnostics.

The present experiments assess two fixed full-window configurations under repeated sensor noise. Broader validation should include independent trajectories, informative friction transitions, and input-matched official implementations. Recursive marginalization and analytical sparse sensitivities may improve computational cost. Hardware validation requires calibrated FBG channels, synchronized geometry, measured stiffness, and independent contact/tip force truth.

"""
    source = replace_results_section(source, tail)
    (paper / "main.tex").write_text(source, encoding="utf-8", newline="\n")
    print(f"Updated actual results document and manuscript: {contact_wins}/4 contact comparisons; {covered}/{eligible} local components.")


if __name__ == "__main__":
    update()
