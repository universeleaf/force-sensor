"""Build a readable force report from the recorded MATLAB window experiments."""
from __future__ import annotations

import hashlib
import json
import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "out" / "formulation_window"
TITLES = {
    "two_contact": "S 通道双接触",
    "tapered_two_contact": "非平行双壁收窄通道",
    "three_contact": "蛇形三接触",
    "rotated_two_contact": "整体旋转后的双接触",
    "three_contact_noisy": "含噪三接触",
    "spatial_sliding": "面外摩擦双接触",
    "spatial_sliding_noisy": "含噪面外摩擦双接触",
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(value: float | None) -> str:
    return "未定义" if value is None else f"{value:.6g}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-revision', help='Verify recorded MATLAB source content against a recoverable Git revision.')
    args = parser.parse_args()
    report = json.loads((FOLDER / "comparison.json").read_text(encoding="utf-8"))
    if report["state"] != "complete":
        raise RuntimeError("The protocol is still running; do not publish a partial summary.")
    matches = all(
        (ROOT / item["path"]).is_file()
        and digest(ROOT / item["path"]) == item["sha256"]
        for item in report["runRecord"]["source"]
    )
    revision_note = ''
    if args.source_revision:
        for item in report['runRecord']['source']:
            blob = subprocess.run(['git', 'show', f"{args.source_revision}:{item['path']}"],
                                  cwd=ROOT, check=True, capture_output=True).stdout
            lf = blob.replace(b'\r\n', b'\n')
            possible = {hashlib.sha256(data).hexdigest() for data in (blob, lf, lf.replace(b'\n', b'\r\n'))}
            recorded_matches = item['sha256'] in possible
            current = (ROOT / item['path']).read_bytes()
            if not recorded_matches:
                recorded_matches = (hashlib.sha256(current).hexdigest() == item['sha256']
                                    and current.replace(b'\r\n', b'\n') == lf)
            if not recorded_matches:
                raise RuntimeError(f"Recorded source cannot be verified at {args.source_revision}: {item['path']}")
        revision_note = f"源码内容已核对 Git 版本 `{args.source_revision}`（允许文本换行符差异），可恢复本次求解代码。"
    fit_cases = {}
    fit_path = FOLDER / 'observation_fit.json'
    if fit_path.is_file():
        diagnostic = json.loads(fit_path.read_text(encoding='utf-8'))
        if diagnostic['state'] != 'complete' or diagnostic['originalRunId'] != report['runRecord']['runId']:
            raise RuntimeError('Derived fit diagnostic belongs to a different or incomplete run.')
        fit_cases = {case['id']: case for case in diagnostic['cases']}
    lines = [
        "# 完整三维多接触窗口：实际运行结果",
        "",
        f"运行 ID：`{report['runRecord']['runId']}`。源码 SHA-256 与当前工作区一致：{matches}。{revision_note}",
        "",
        "每组使用两个状态，24 个稀疏弯曲采样位置、两个实际观测通道；无噪声条件保留显式数值方差。",
        "真值独立求解接触平衡，仅进入评分；逆解通过观测生成候选，不读取接触数量/位置/力标签。",
        "review 是算法复核标志，不是用力真值作出的正确性判定。",
        "",
        "| 场景 | 候选/真接触数 | 接触力向量 RMSE / N | 接触力大小 MAE / N | 位置 RMSE / mm | 末端力 RMSE / N | 合力 RMSE / N | 原始复核帧数 | 合并拟合诊断后 | 优化时间 / s |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for case in report["cases"]:
        title = TITLES.get(case["id"], case["id"])
        if not case["completed"]:
            lines.append(f"| {title} | 运行失败 | — | — | — | — | — | — | — | — |")
            continue
        folder = FOLDER / case["artifactFolder"]
        for name, filename in (("input", "input.mat"), ("truth", "truth.mat"), ("estimate", "estimate.mat")):
            if digest(folder / filename) != case["artifactSha256"][name]:
                raise RuntimeError(f"Recorded artifact checksum differs: {case['id']}/{filename}")
        m = case["metrics"]
        combined = '未另评估'
        fit = fit_cases.get(case['id'])
        if fit and fit['completed']:
            if fit['estimateSha256'] != case['artifactSha256']['estimate']:
                raise RuntimeError(f"Derived fit diagnosis is stale: {case['id']}")
            combined = f"{sum(fit['combinedRequiresReview'])}/{m['frameCount']}"
        values = [
            m["contactForceRmseN"], m["contactMagnitudeMaeN"], m["contactArcRmseMm"],
            m["tipForceRmseN"], m["totalForceRmseN"],
        ]
        lines.append(
            f"| {title} | {m['candidateCount']}/{m['trueContactCount']} | "
            + " | ".join(number(v) for v in values)
            + f" | {m['reviewCount']}/{m['frameCount']} | {combined} | {m['optimizationSeconds']:.2f} |"
        )
    lines += [
        "", "模型一致的无噪声结果接近数值闭合精度，不能解释为实物传感器精度。",
        "空间滑动的第一状态缺少更早历史，其非零摩擦按静态锥处理并复核；第二状态使用前驱平衡与完整摩擦互补。",
        "优化时间不含 MATLAB 启动、真值生成和文件输出；本次并行运行了其他本地任务，不能与独立实时基准直接排序。",
        "局部协方差未经过全局覆盖率校准，且候选覆盖、模式混合、材料误差及原论文方法比较未由这些窗口验证。",
        "",
    ]
    for case in report["cases"]:
        if not case["completed"]:
            continue
        title = TITLES.get(case["id"], case["id"])
        lines += [f"## {title}：每个接触的力大小", "",
                  "| 状态 | 接触 | 真值 / N | 估计 / N | 向量误差 / N | 复核 |",
                  "|---|---:|---:|---:|---:|---|"]
        for frame in case["metrics"]["perFrame"]:
            errors = frame["contactVectorErrorN"]
            for i, (actual, estimate) in enumerate(zip(frame["trueForceN"], frame["estimatedForceN"])):
                error = errors[i] if i < len(errors) else None
                lines.append(
                    f"| {frame['frame']} | {i + 1} | {actual:.6f} | {estimate:.6f} | "
                    f"{number(error)} | {'是' if frame['requiresReview'] else '否'} |"
                )
        lines += ["", f"逐时刻世界坐标力分量：[forces.csv]({case['artifactFolder']}/forces.csv)。", ""]
    (FOLDER / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Summary written; {len(report['cases'])} cases, source matches: {matches}")


if __name__ == "__main__":
    main()
