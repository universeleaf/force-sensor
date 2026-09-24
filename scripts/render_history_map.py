"""Render measured fixed-vs-latent MAP results and a Chinese technical note."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    report = json.loads((ROOT / 'out/history_map/comparison.json').read_text(encoding='utf-8'))
    if report['state'] not in ('complete', 'complete-with-failures'):
        raise RuntimeError('The comparison is still running; do not publish partial results as final.')
    run_id = report['runRecord']['runId']
    folder = ROOT / 'out/history_map' / run_id
    cases = report['cases']
    complete = [c for c in cases if c['completed']]
    rows, final_rows, history_rows, artifacts, failures = [], [], [], [], []
    csv_rows = []
    for case in cases:
        if not case['completed']:
            failures.append(f"- {case['id']}: 未完成，原因见原始 comparison.json。")
            continue
        for m in case['methods']:
            name = '固定历史' if m['name'] == 'fixed' else '联合历史 FBG'
            review = sum(m['quality']['requiresReview'])
            seconds = m['frameSeconds']
            rows.append(f"| {case['id']} | {name} | {m['contactRmseN']:.6f} | {m['tipRmseN']:.6f} | {m['totalRmseN']:.6f} | {min(seconds):.1f}–{max(seconds):.1f} | {review}/{len(seconds)} |")
            final_rows.append(f"| {case['id']} | {name} | {m['trueContactMagnitudeN'][-1]:.6f} | {m['contactMagnitudeN'][-1]:.6f} | {m['contactVectorErrorN'][-1]:.6f} |")
            for k, h in enumerate(m['history']):
                if h['latentDimension']:
                    history_rows.append(f"| {case['id']} | {k+1} | {h['latentDimension']} | {h['rmsCorrectionSigma']:.4f} | {h['maxCorrectionSigma']:.4f} | {np.linalg.norm(h['measuredTangentialStepMm']):.5f} | {np.linalg.norm(h['estimatedTangentialStepMm']):.5f} |")
                row = {'scene': case['id'], 'method': m['name'], 'frame': k + 1}
                for force, key, true_key in [('contact', 'contactForceN', 'trueContactForceN'), ('tip', 'tipForceN', 'trueTipForceN'), ('total', 'totalForceN', 'trueTotalForceN')]:
                    estimated = np.asarray(m[key])[:, k]
                    true = np.asarray(m[true_key])[:, k]
                    for a, axis in enumerate('xyz'):
                        row[f'{force}_true_{axis}_N'] = true[a]
                        row[f'{force}_estimated_{axis}_N'] = estimated[a]
                    row[f'{force}_vector_error_N'] = np.linalg.norm(estimated - true)
                    row[f'{force}_true_magnitude_N'] = np.linalg.norm(true)
                    row[f'{force}_estimated_magnitude_N'] = np.linalg.norm(estimated)
                csv_rows.append(row)
            artifacts.append(f"- {case['id']} / {name}：[结果 MAT](../out/history_map/{run_id}/{case['id']}/{m['name']}.mat)。")
    if csv_rows:
        with (folder / 'forces.csv').open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=csv_rows[0])
            writer.writeheader(); writer.writerows(csv_rows)
    if complete:
        fig, axes = plt.subplots(3, len(complete), squeeze=False, figsize=(6.3*len(complete), 9), layout='constrained')
        for col, case in enumerate(complete):
            baseline, latent = case['methods']
            frame = np.arange(1, len(baseline['frameSeconds'])+1)
            ax = axes[0, col]
            ax.plot(frame, baseline['trueContactMagnitudeN'], 'k-o', label='Truth')
            for m, color, label in [(baseline, '#bf6536', 'Fixed history'), (latent, '#157b86', 'Latent FBG history')]:
                ax.plot(frame, m['contactMagnitudeN'], '--o', color=color, label=label)
                axes[1, col].plot(frame, m['contactVectorErrorN'], '-o', color=color, label=label)
                tip_error = np.linalg.norm(np.asarray(m['tipForceN'])-np.asarray(m['trueTipForceN']), axis=0)
                axes[2, col].plot(frame, tip_error, '-o', color=color, label=label)
            ax.set_title(case['id']); ax.legend(fontsize=9)
            for row, label in enumerate(['Contact magnitude (N)', 'Contact vector error (N)', 'Tip vector error (N)']):
                axes[row, col].set_ylabel(label); axes[row, col].set_xlabel('Sampled equilibrium')
                axes[row, col].set_xticks(frame); axes[row, col].grid(alpha=.22)
        fig.savefig(folder / 'comparison.png', dpi=180)
        fig.savefig(folder / 'comparison.svg')
        plt.close(fig)
    noisy = next((c for c in complete if c['id'] == 'sliding_noisy'), None)
    conclusion = '结果以表格和失败记录为准。'
    if noisy:
        a, b = noisy['methods']
        direction = '下降' if b['contactRmseN'] < a['contactRmseN'] else '上升'
        conclusion = (f"同一含噪滑动输入下，接触力向量 RMSE 从 **{a['contactRmseN']:.6f} N** 变为 **{b['contactRmseN']:.6f} N**，"
                      f"{direction} **{abs(noisy['contactRmseReductionPct']):.2f}%**。这是一条固定噪声轨迹的对照，不是跨随机种子的普遍优势证明。")
    diagnostic_id = 'f8d34ea2-a2e3-42a0-bc12-44db8fa29a8c'
    diagnostic_path = ROOT / 'out/history_map' / diagnostic_id / 'comparison.json'
    diagnostic_text = ''
    if diagnostic_path.exists():
        diagnostic = json.loads(diagnostic_path.read_text(encoding='utf-8'))
        diagnostic_case = next(c for c in diagnostic['cases'] if c['id'] == 'sliding_noisy')
        old, first = diagnostic_case['methods']
        diagnostic_text = f'''### 先前诊断也保留

只增加历史隐变量、仍使用折线历史取点时，接触力 RMSE 为 {old['contactRmseN']:.6f} → {first['contactRmseN']:.6f} N，收益仅 0.09%，联合版本末帧出现非正退出。这没有被包装成成功。

随后圆弧检查复现了折线取点的节点导数跳变：约 0.02。改为分段 SE(3) 部分积分后，解析圆弧取点误差约 1.1e-13 mm；以 1e-4 mm 步长测得的左右导数差约 2e-6，符合平滑曲线的有限差分误差量级。新取点使用与重建相同的网格中点曲率，完整单元端点的位置和切向都一致，不额外对曲率进行平滑。

**本报告主表中的固定历史与联合历史都使用连续取点**，所以二者之差隔离历史隐变量的作用；不能将相对旧折线版本的全部变化都归功于联合 MAP。新研究配置默认 `historyPointInterpolation='integrated'`，旧保存输入缺此字段时仍按 `linear` 重放。

[首轮诊断 JSON](../out/history_map/{diagnostic_id}/comparison.json)与[日志](../out/history_map/{diagnostic_id}/run.log)保留。该轮含噪两方法完整完成；无噪控制在第三帧期间主动中止，以优先修正已复现的插值缺陷，未将其统计为已完成。本报告主表仅统计最终重新完成的工况。
'''
    note = f'''# 2026-09-21：历史 FBG 隐状态与完整摩擦 MAP

{conclusion}

运行标识：`{run_id}`。本次固定历史和联合历史两条路径均重新求解，并共同采用连续历史取点，没有用旧估计作为新方法的初值。项目宗旨仍是**由形状信息和环境信息估计接触力的大小、方向与位置，并与独立末端力分离**。只有仿真，没有接入真实机器人。

## 为什么改算法

原完整 MPCC 已经让当前形状满足三维 Cosserat 平衡，但前驱形状由含噪 FBG 重建后固定。摩擦式中的切向位移是当前接触点减去前驱同弧长位置；把后者当成精确值，会迫使当前载荷与接触模式适配历史测量误差。数值互补可行仍可能伴随较大的分力误差。

新 Cosserat 当前形状映射不依赖前驱形状，所以仅向当前曲率似然加入历史方差不能处理这条误差路径。本轮直接优化历史观测对应的隐变量。

{diagnostic_text}

## 现在求解什么

原物理状态完整保留：`x=[p1(3), eta(2), s1, fn, beta(16), lambda, fe(3)]`，共 27 维。新增每个历史 FBG 的两路弯曲量：24 个传感位置对应 **48 个自由变量**，总计 **75 维**。没有压成常数/线性低秩模式，没有把未知末端力设为零，也没有对历史修正设置任意的三倍标准差硬边界。

令 `q=(u_previous_latent-u_previous_measured)/sigma_history`，优化目标为：

```text
min_(x,q)  1/2 (x-x_prior)' P_prior^(-1) (x-x_prior)
         + 1/2 ||z_current - h(x)||_(R_current^(-1))^2
         + 1/2 ||q||^2

current shape = nonlinear 3-D Cosserat equilibrium(x)
previous shape = curvature integration(u_previous_measured + sigma_history*q)
v_t = (I-n*n') * (p_current(s1)-p_previous(s1))

0 <= gap                 perpendicular fn >= 0
0 <= D'*v_t + lambda*1   perpendicular beta >= 0
0 <= mu*fn-sum(beta)      perpendicular lambda >= 0
```

历史积分沿用相同的本征曲率差值插值与 SE(3) 中点积分，基座位姿保留在它自己的观测时刻。当前曲率仍只进入一次当前似然，历史曲率只进入一次历史似然。平面点与法向继续使用已有完整 6×6 环境协方差。原 Formulation 的力分解、间隙、位移、摩擦互补和随机游走先验均保留；扩展的是原式 (19) 的历史测量项。

完整 MPCC 连续化后仍进行活跃集合修正。本轮同时修正了修正器的变量选择，使附加历史坐标继续可优化，避免末步又将它们冻结。`historyCurvatureStdPerMm=0` 时不创建历史变量，严格回到固定历史路径；历史版本默认 `historyStateMode='fixed'`。

## 同一输入下的结果

误差均为三维力向量误差的 RMSE，单位 N；幅值误差另列，不能互相替代。两方法使用相同原始传感数据、材料、环境、当前测量权重和求解预算，真值只用于评分。

| 场景 | 方法 | 接触力 RMSE / N | 末端力 RMSE / N | 合力 RMSE / N | 每帧秒数 | 需复核帧 |
|---|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

| 场景末帧 | 方法 | 真实接触力大小 / N | 估计大小 / N | 向量误差 / N |
|---|---|---:|---:|---:|
{chr(10).join(final_rows)}

![同输入接触力和末端力对照](../out/history_map/{run_id}/comparison.png)

### 历史修正的大小

下表 RMS 和最大值均以标定历史曲率噪声标准差为单位。“原历史位移”使用**新方法最终的当前状态**配合未修正历史；“隐状态位移”使用同一当前状态配合修正历史。这两个量用于检查修正经过哪条路径进入摩擦，不是基线估计与真值滑移的误差。

| 场景 | 帧 | 隐变量数 | RMS / σ | 最大绝对值 / σ | 原历史位移 / mm | 隐状态位移 / mm |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(history_rows)}

{chr(10).join(failures)}

## 已完成与仍未完成

- 已完成：配对历史测量的隐变量 MAP；全部 48 个已观测曲率自由度；完整摩擦约束；历史修正和标准化残差导出；同输入重算对照。
- 历史形状目前满足曲率积分运动学，没有独立施加前驱时刻的载荷平衡，也没有联合估计前驱接触力。它是配对历史 MAP，不是两时刻完整物理状态的平滑器。
- 当前输出的 `posteriorCovariance` 仍是当前观测与先验信息矩阵近似，没有纳入约束曲率或边缘化历史/模式。`historyUncertaintyModeled=true` 表示历史似然参与优化，**不表示力区间覆盖率已验证**。相邻窗口共享样本的相关性也未完整传播。
- 历史未测扭转、基座位姿、刚度、本征曲率仍依赖给定重建/标定假设。历史噪声模型是独立同方差高斯，当前似然中的曲率权重沿用旧配置，未按本次真值调参。
- 单个平面、单个候选身体接触和局部准静态平衡的限制保留。滑动真值是已有的指定滑动分支配对平衡；没有把它冒充粘滑转换、多接触或空间摩擦验证。
- 本轮针对性检查覆盖完整历史坐标、零修正重建、缓存、配置限制和修正器中的附加变量；新增解析圆弧取点和网格节点导数连续性检查。首轮同时运行现有输入合同/配置/模式边界检查。没有重复宣称旧 21 项验收适用于全部新算法。

这一步补齐了历史观测进入 MAP 的路径，能否作为论文贡献仍需多随机种子、未见轨迹、标定失配、同信息基线和完整物理窗口的证据；当前不能据此宣称 RA-L 投稿就绪。

## 使用与产物

```matlab
r = force('history');  % 含噪滑动：固定历史和联合历史分别重算
addpath('rod');
r = run_history_map_benchmark({{'sliding_noisy','sliding_clean'}});

% 对已有传感器包启用；所有其余标定和观测保持原值
sensorInput.config = formulation_solver_config(sensorInput.config);
sensorInput.config.forceSensor.historyStateMode = 'latent-fbg';
sensorInput.config.forceSensor.historyCurvatureStdPerMm = 5e-5; % 示例；必须来自噪声标定
output = estimate_sensor_forces(sensorInput);
```

报告/图重生成：`python scripts/render_history_map.py`，随后 `python scripts/render_notes.py docs/history_map_2026-09-21.md`。

[本次完整比较 JSON](../out/history_map/{run_id}/comparison.json) · [逐帧全部力 CSV](../out/history_map/{run_id}/forces.csv) · [矢量图 SVG](../out/history_map/{run_id}/comparison.svg)。每方法 MAT 含原始输入、配置、完整解和连续化退出记录；源输入 SHA-256 与源码快照在比较 JSON 中。

{chr(10).join(artifacts)}

## 下一步按优先级执行

1. 固定本轮算法及噪声标定，在多种子和曲率/环境/标定误差矩阵中评价分力误差、失败率与耗时，避免围绕单个 seed 调优。
2. 把前驱时刻的力学平衡、接触状态和历史环境观测纳入物理窗口；显式处理共享观测和窗口间相关性，同时估计接触/末端力变化。
3. 针对接触靠近末端造成的分力退化，加入包含历史与几何变量的约束局部灵敏度分析，再评估区间覆盖率。
4. 补独立粘着—滑动—反向滑动真值和统一传感信息的公开方法基线；保留无法准确分力的场景。
'''
    (ROOT / 'docs/history_map_2026-09-21.md').write_text(note, encoding='utf-8')
    print(f'Rendered {len(complete)}/{len(cases)} cases: {folder}')


if __name__ == '__main__':
    main()
