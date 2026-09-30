"""Summarize corrected fixed-environment mismatch runs and derived fit checks."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'out' / 'model_mismatch'
TITLES = {'two-contact': 'S 通道遗漏一面墙',
          'curved-surface': '收窄通道遗漏一面墙（两个平面）',
          'friction-mismatch': '空间滑动：真实 mu=0.03，估计 mu=0.01'}


def main() -> None:
    report = json.loads((FOLDER / 'comparison.json').read_text(encoding='utf-8'))
    if report['state'] not in ('complete', 'complete-with-failures'):
        raise RuntimeError('Do not publish a running mismatch protocol as completed.')
    diagnostic = json.loads((FOLDER / 'observation_fit.json').read_text(encoding='utf-8'))
    if diagnostic['state'] != 'complete' or diagnostic['originalRunId'] != report['runRecord']['runId']:
        raise RuntimeError('Fit diagnostics are incomplete or refer to another run.')
    fits = {case['id']: case for case in diagnostic['cases']}
    run_folder = FOLDER / report['runRecord']['runId']
    lines = ['# 固定环境真值下的模型失配实验', '',
             f"运行 ID：`{report['runRecord']['runId']}`。状态：{report['state']}。", '',
             '真值先独立求解接触闭合与切触；随后只对逆解遗漏环境面或改变摩擦参数。',
             '这是故意违反模型假设的压力测试，不能作为正常精度或 SOTA 证据。', '',
             '| 场景 | 真接触/估计候选 | 杆身合力 RMSE / N | 末端力 RMSE / N | 总合力 RMSE / N | 原始复核比例 | 合并拟合诊断复核 | 平均优化时间 / s |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for case in report['cases']:
        kind = case['kind']
        title = TITLES.get(kind, kind)
        if not case['completed']:
            lines.append(f'| {title} | 运行失败 | — | — | — | — | — | — |')
            continue
        for key, filename in {'input': f'{kind}_input.mat', 'truth': f'{kind}_truth.mat',
                              'estimate': f'{kind}.mat', 'forces': f'{kind}_forces.csv'}.items():
            if hashlib.sha256((run_folder / filename).read_bytes()).hexdigest() != case['artifactSha256'][key]:
                raise RuntimeError(f'Recorded artifact changed: {filename}')
        fit = fits[kind]
        if not fit['completed'] or fit['estimateSha256'] != case['artifactSha256']['estimate']:
            raise RuntimeError(f'Derived diagnostic is missing or stale: {kind}')
        combined = fit['combinedRequiresReview']
        lines.append(f"| {title} | {case['trueContactCount']}/{case['estimatedContactCount']} | "
                     f"{case['contactRmseN']:.6g} | {case['tipRmseN']:.6g} | {case['totalRmseN']:.6g} | "
                     f"{case['reviewRate']:.0%} | {sum(combined)}/{len(combined)} | {case['meanFrameSeconds']:.2f} |")
    lines += ['', '“杆身合力”是所有接触力的向量和，不是逐接触力 RMSE；候选数量不同不能逐槽对比。',
              '原始质量来自求解时版本，新增诊断另存于 observation_fit.json；没有改写原始力或追改原始 quality。',
              '高拟合残差提示模型与观测有张力，不能自动确定是遗漏墙面、标定误差或噪声模型错误。',
              '旧 prescribed-load 压力结果已撤回；本报告不复用其数值。', '']
    for case in report['cases']:
        if case['completed']:
            lines.append(f"逐接触估计：[{' '.join(case['kind'].split('-'))} forces.csv]({report['runRecord']['runId']}/{case['kind']}_forces.csv)。")
    (FOLDER / 'summary.md').write_text('\n'.join(lines), encoding='utf-8')
    print(f"Mismatch summary written: {len(report['cases'])} cases.")


if __name__ == '__main__':
    main()
