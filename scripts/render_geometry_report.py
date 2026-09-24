"""Render the completed rod-plane ablation, preserving failures and provenance."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    report = json.loads((ROOT / 'out/geometry/comparison.json').read_text(encoding='utf-8'))
    if report['state'] not in ('complete', 'complete-with-failures'):
        raise RuntimeError('The run is still in progress.')
    run_id = report['runRecord']['runId']
    folder = ROOT / 'out/geometry' / run_id
    complete = [c for c in report['cases'] if c['completed']]
    rows, geometry_rows, csv_rows, links = [], [], [], []
    for c in complete:
        for key, label in [('baseline', '原单点约束'), ('corrected', '加入杆身约束')]:
            m = c[key]
            rows.append(f"| {c['id']} | {label} | {m['contactRmseN']:.6f} | {m['tipRmseN']:.6f} | {m['totalRmseN']:.6f} |")
            geometry_rows.append(f"| {c['id']} | {label} | {min(m['denseMinimumGapMm']):.3g} | {m['maxConstraintResidual']:.3g} | {m['maxConeViolationN']:.3g} | {sum(m['quality']['requiresReview'])}/{len(m['frameSeconds'])} |")
            for k in range(len(m['frameSeconds'])):
                row = dict(scene=c['id'], method=key, frame=k+1,
                           contact_s_mm=m['contactArcLengthMm'][k],
                           true_contact_s_mm=m['trueContactArcLengthMm'][k],
                           dense_min_gap_mm=m['denseMinimumGapMm'][k],
                           normal_tangent=m['normalTangent'][k],
                           requires_review=m['quality']['requiresReview'][k])
                for force, field, tf in [('contact', 'contactForceN', 'trueContactForceN'),
                                         ('tip', 'tipForceN', 'trueTipForceN'),
                                         ('total', 'totalForceN', 'trueTotalForceN')]:
                    est, truth = np.asarray(m[field])[:, k], np.asarray(m[tf])[:, k]
                    for a, axis in enumerate('xyz'):
                        row[f'{force}_estimated_{axis}_N'] = est[a]
                        row[f'{force}_true_{axis}_N'] = truth[a]
                    row[f'{force}_vector_error_N'] = np.linalg.norm(est-truth)
                csv_rows.append(row)
        links.append(f"- `{c['id']}`：[完整 MAT](../out/geometry/{run_id}/{c['id']}/results.mat)，原结果 SHA-256 `{c['sourceSha256']}`。")
    if csv_rows:
        with (folder / 'forces.csv').open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=csv_rows[0]); writer.writeheader(); writer.writerows(csv_rows)
    noisy = next((c for c in complete if c['id'] == 'sliding_noisy'), None)
    if complete:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
        x = np.arange(len(complete)); colors = ['#b86936', '#147c85']
        for j, (key, label) in enumerate([('baseline', 'Point contact only'), ('corrected', 'Rod admissibility')]):
            axes[0, 0].bar(x+(j-.5)*.36, [c[key]['contactRmseN'] for c in complete], .36, label=label, color=colors[j])
            axes[0, 1].bar(x+(j-.5)*.36, [max(0, -min(c[key]['denseMinimumGapMm'])) for c in complete], .36, label=label, color=colors[j])
        for ax, ylabel in zip(axes[0], ['Contact vector RMSE (N)', 'Max sampled penetration (mm)']):
            ax.set_xticks(x, [c['id'].replace('_', '\n') for c in complete], fontsize=8)
            ax.set_ylabel(ylabel); ax.grid(axis='y', alpha=.2); ax.legend(fontsize=8)
        if noisy:
            frames = np.arange(1, len(noisy['baseline']['frameSeconds'])+1)
            axes[1, 0].plot(frames, np.linalg.norm(noisy['baseline']['trueContactForceN'], axis=0), 'k-o', label='Independent truth')
            for j, (key, label) in enumerate([('baseline', 'Point contact only'), ('corrected', 'Rod admissibility')]):
                m = noisy[key]; f = np.asarray(m['contactForceN']); truth = np.asarray(m['trueContactForceN'])
                axes[1, 0].plot(frames, np.linalg.norm(f, axis=0), '--o', color=colors[j], label=label)
                axes[1, 1].plot(frames, np.linalg.norm(f-truth, axis=0), '-o', color=colors[j], label=label)
            for ax, ylabel in zip(axes[1], ['Contact magnitude (N)', 'Contact vector error (N)']):
                ax.set_title('sliding_noisy: same observations and weights')
                ax.set_xlabel('Sampled equilibrium'); ax.set_xticks(frames); ax.set_ylabel(ylabel)
                ax.grid(alpha=.2); ax.legend(fontsize=8)
        else:
            for ax in axes[1]: ax.axis('off')
        fig.savefig(folder / 'comparison.png', dpi=180); fig.savefig(folder / 'comparison.svg'); plt.close(fig)
    conclusion = '全部场景结果见下表，失败场景单独列出。'
    if noisy:
        a, b = noisy['baseline'], noisy['corrected']
        conclusion = (f"含噪滑动的接触／末端／合力向量 RMSE 从 **{a['contactRmseN']:.6f}／{a['tipRmseN']:.6f}／{a['totalRmseN']:.6f} N** "
                      f"变为 **{b['contactRmseN']:.6f}／{b['tipRmseN']:.6f}／{b['totalRmseN']:.6f} N**。"
                      "这是固定观测上的几何约束对照；不能用它代替多随机种子鲁棒性验证。")
    failures = '\n'.join(f"- {c['id']}：{c.get('errorIdentifier', '未完成')}，详细原因保留在 JSON。" for c in report['cases'] if not c['completed']) or '本次运行没有未完成场景。'
    note = f'''# 杆身接触几何修复与同输入验证（2026-09-21）

{conclusion}

本次完成 **{len(complete)}/{len(report['cases'])} 个场景**。运行标识 `{run_id}`，原六场景运行 `{report['sourceRunId']}`。仍是仿真；真值来自独立平面连续射击，估计器只接收模型、模拟 FBG 和模拟平面观测。

## 发现的根因

此前只在未知候选位置 s1 约束 `gap >= 0`、`gap * fn = 0`。这不能阻止杆的其他部分穿过刚性平面，也不能保证正压力作用于杆身的局部最低间隙处。因此“候选点互补通过”与“整根杆物理相容”并不等价。新的稠密核查以**估计平面**为基准，避免把平面测量误差混进碰撞约束。

原历史 MAP 样例前两帧的节点穿透约 0.066、0.129 mm。先单独把曲率似然标准差从 1.5e-4 改为已知注入噪声 5e-5，只把接触 RMSE 从 1.0055 降至 0.9734 N，合力 RMSE 却从 0.3512 升至 0.4927 N；未采用该调权方案。本表保持原场景的观测、权重、初值生成、历史取点与求解预算，仅增加杆身几何条件。

## 代码改动与 Formulation 对应

- 当前三维 Cosserat ODE 在固定弧长网格上连续求值，最大间隔 0.5 mm；每个采样点要求 `n'*(p(s)-p1) >= 0`。不使用节点间直线替代连续形状。
- 内部接触增加 `fn * 4*r*(1-r) * n'*t(s1) = 0`，其中 `r=(s1-s0)/(L-s0)`。有正压力的内部光滑接触必须相切；无接触及杆端接触不强制相切。
- Formulation 中的法向、各多边形摩擦方向与摩擦锥互补全部保留。这是对**单候选点模型**补充刚性平面的几何可行域，并非 PDF 已逐字包含整杆碰撞约束。
- 主 MPCC、最终活动集精修、最终残差使用同一个几何约束函数。拒绝力学发散试探时保持约束维数；中间阶段与最终评价包含新增等式残差。
- 相切条件中的力乘积也随 Scholtes 容差逐级收紧，最终精修检查原等式。直接从第一阶段施加强等式曾把无噪声第一帧卡在零接触力分支，已用专门回归复现并修正；没有固定接触模式或删除摩擦条件。
- 研究配置默认 `planeContactGeometry='rod'`；保存输入缺此字段时仍按 `point-only` 重放。质量输出增加 `minimumSampledRodGapMm`、`rodGeometryEnforced`、`hasRodPenetration`，穿透触发 `requiresReview`。旧结果中的旧质量标签不追溯修改，下面另列本次稠密审计。

关键代码：[几何约束](../rod/plane_contact_constraints.m)、[连续力学映射](../rod/solve_cosserat_force_map.m)、[完整 MPCC](../rod/solve_contact_mpcc.m)、[精修](../rod/solve_contact_mode_map.m)。

## 力误差

RMSE 为世界坐标下向量误差，单位 N；接触力、末端力与合力分别评分。

| 场景 | 方法 | 接触 RMSE | 末端 RMSE | 合力 RMSE |
|---|---|---:|---:|---:|
{chr(10).join(rows)}

![同输入几何约束对照](../out/geometry/{run_id}/comparison.png)

## 物理检查

在估计完成后，用相对容差 2e-8 的单独力学求解和 0.1 mm 网格检查。最小间隙负值表示穿透；绝对量级接近 1e-6 mm 的变化需结合 ODE 容差理解。优化本身用 2e-7 相对容差和 0.5 mm 采样。

| 场景 | 方法 | 稠密最小间隙 mm | 最终活动约束残差 | 最大摩擦锥违反 N | 保存结果需复核帧数 |
|---|---|---:|---:|---:|---:|
{chr(10).join(geometry_rows)}

原单点算法的残差只覆盖它原来的约束，不能与新增约束后的残差当作相同可行域比较。旧结果的 `requiresReview=false` 不代表它通过本次整杆几何审计。中间连续化阶段退出码与最终精修信息均保留在 JSON/MAT；求解完成也不等于每一级都收敛。

独立的方向与几何核验由 `validate_plane_geometry_run` 生成，直接检查摩擦做功 `Ft' * v <= 1e-6 N mm`、法向分量、锥界及 0.1 mm 网格。记录见本运行目录的 `physics_audit.json`；这是数值物理一致性核验，不是力精度认证。

## 检查与复现

新增回归先复现“单点可行但杆身穿透”，再验证完整求解和精修都保留新约束；另用解析圆弧验证节点之间的穿透能被连续采样发现，并检查刚体旋转、无接触与杆端情形。工程检查账本：[project_checks.json](../out/completion/project_checks.json)。

```matlab
force('geometry');  % 使用原六场景的保存输入，生成独立运行目录
validate_plane_geometry_run; % 独立核验保存结果的摩擦方向、锥界和穿透
force('check');     % 项目检查与两组旧输入完整重放
```

```text
python scripts/render_geometry_report.py
python scripts/render_notes.py docs/geometry.md README.md docs/STATUS.md
```

原六场景播放器仍展示旧算法结果，以本报告作为修复后的比较入口。[完整对照 JSON](../out/geometry/{run_id}/comparison.json)、[逐帧力与几何 CSV](../out/geometry/{run_id}/forces.csv)、[矢量图](../out/geometry/{run_id}/comparison.svg)。比较入口固定使用原运行 `{report['sourceRunId']}`，重新生成 demo 不会悄悄替换本对照的基线。`force('formulation')` 的旧三方法对照仍显式采用 `point-only`，以单独比较力学映射和 MPCC；新的研究配置、`estimate_formulation_forces` 及新生成的 demo 默认启用杆身几何。

{chr(10).join(links)}

{failures}

诊断过程也保留：[直接硬相切等式的中止运行](../out/geometry/408f4a5a-5cef-4c78-abe6-8bb5c9f09668/comparison.json)与[日志](../out/geometry/408f4a5a-5cef-4c78-abe6-8bb5c9f09668/run.log)。该轮只完成含噪一组，接触 RMSE 1.065743 N；无噪声在第二帧中止以修复第一帧的错误局部解，没有算作完成。初始化排序也曾单独诊断，但没有改变该例初值与连续化结果，因此撤回此改动。只保留有回归证据的约束与连续化修复。

## 仍未完成的研究内容

采样检查不是连续区间无穿透的严格证明；当前还限定为零半径中心线、单个接触和平面。相切必要条件不保证平衡唯一或稳定。历史 FBG 隐变量尚未与前驱力学平衡联合，力协方差也未边缘化历史、平面和接触模式。需要多随机种子、噪声等级、标定失配、真正空间载荷与多接触实验。

算法仍是递推约束 MAP，采用随机游走先验和局部协方差近似；EKF、Cosserat、摩擦互补以及本次几何缺陷修复本身都不能单独宣称 RA-L 创新。研究贡献仍需围绕形状与环境融合怎样改善分力可辨识性，以及噪声下可靠的接触状态和不确定性，给出充分对照证据。
'''
    (ROOT / 'docs/geometry.md').write_text(note, encoding='utf-8')
    print(f'Wrote docs/geometry.md and artifacts for {run_id}; {len(complete)} completed cases.')


if __name__ == '__main__':
    main()
