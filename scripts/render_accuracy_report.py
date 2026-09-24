"""Render the paired archived-input accuracy benchmark; never replace inputs."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    report = json.loads((ROOT / 'out/accuracy/comparison.json').read_text(encoding='utf-8-sig'))
    if report['state'] != 'complete':
        raise RuntimeError('Accuracy run is incomplete; retain its failures instead of publishing success.')
    run_id = report['runRecord']['runId']
    folder = ROOT / 'out/accuracy' / run_id
    audit = json.loads((folder / 'physics_audit.json').read_text(encoding='utf-8-sig'))
    diag = json.loads((folder / 'diagnostics.json').read_text(encoding='utf-8-sig'))
    quality = json.loads((folder / 'quality_audit.json').read_text(encoding='utf-8-sig'))
    assert audit['runId'] == diag['runId'] == quality['runId'] == run_id
    cases = report['cases']
    rows, physics_rows, records = [], [], []
    for c in cases:
        a, b = c['baseline'], c['corrected']
        checked_quality = next(q for q in quality['cases'] if q['id'] == c['id'])
        rows.append(f"| {c['id']} | {a['contactRmseN']:.6g} → {b['contactRmseN']:.6g} | "
                    f"{a['tipRmseN']:.6g} → {b['tipRmseN']:.6g} | {a['totalRmseN']:.6g} → {b['totalRmseN']:.6g} |")
        physics_rows.append(f"| {c['id']} | {min(a['denseMinimumGapMm']):.4g} → {min(b['denseMinimumGapMm']):.4g} | "
                            f"{b['maxConstraintResidual']:.3g} | {sum(checked_quality['requiresReview'])}/{len(b['frameSeconds'])} |")
        for method in ('baseline', 'corrected'):
            m = c[method]
            for k in range(len(m['frameSeconds'])):
                row = dict(scene=c['id'], method=method, frame=k+1,
                           contact_s_mm=m['contactArcLengthMm'][k], true_contact_s_mm=m['trueContactArcLengthMm'][k],
                           dense_min_gap_mm=m['denseMinimumGapMm'][k], requires_review=m['quality']['requiresReview'][k])
                row['observation_quality_requires_review'] = checked_quality['requiresReview'][k] if method == 'corrected' else ''
                for label, field, true_field in [('contact', 'contactForceN', 'trueContactForceN'),
                                                   ('tip', 'tipForceN', 'trueTipForceN'),
                                                   ('total', 'totalForceN', 'trueTotalForceN')]:
                    est, true = np.asarray(m[field])[:, k], np.asarray(m[true_field])[:, k]
                    for i, axis in enumerate('xyz'):
                        row[f'{label}_estimated_{axis}_N'] = est[i]
                        row[f'{label}_true_{axis}_N'] = true[i]
                    row[f'{label}_error_N'] = float(np.linalg.norm(est-true))
                records.append(row)
    with (folder / 'forces.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=records[0]); writer.writeheader(); writer.writerows(records)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), layout='constrained')
    x = np.arange(len(cases))
    for i, (key, label, color) in enumerate([('baseline', 'Archived baseline', '#ac693f'),
                                            ('corrected', 'Corrected MAP', '#177d88')]):
        axes[0].bar(x+(i-.5)*.36, [c[key]['contactRmseN'] for c in cases], .36, label=label, color=color)
        axes[1].bar(x+(i-.5)*.36, [c[key]['tipRmseN'] for c in cases], .36, label=label, color=color)
    for ax, title in zip(axes, ['Contact vector RMSE (N)', 'Tip vector RMSE (N)']):
        ax.set_yscale('log'); ax.set_title(title)
        ax.set_xticks(x, [c['id'].replace('_', '\n') for c in cases], fontsize=8)
        ax.grid(axis='y', alpha=.2); ax.legend(fontsize=8)
    fig.savefig(folder / 'comparison.png', dpi=180); fig.savefig(folder / 'comparison.svg'); plt.close(fig)
    clean = next(c for c in cases if c['id'] == 'sliding_clean')
    noisy = next(c for c in cases if c['id'] == 'sliding_noisy')
    nd = next(c for c in diag['cases'] if c['id'] == 'sliding_noisy')
    final_gap = min(min(c['corrected']['denseMinimumGapMm']) for c in cases)
    final_res = max(c['corrected']['maxConstraintResidual'] for c in cases)
    frames = sum(len(c['corrected']['frameSeconds']) for c in cases)
    seconds = [v for c in cases for v in c['corrected']['frameSeconds']]
    mid_exits = sum(sum(c['intermediateNonpositiveExits']) for c in diag['cases'])
    rejected = sum(sum(c['rejectedMechanicsTrials']) for c in diag['cases'])
    physics_status = '全部数值物理检查通过' if audit['allPassed'] else '**物理检查未全部通过，不能把本轮作为合格交付**'
    noise = json.loads((ROOT / 'out/accuracy/noise/comparison.json').read_text(encoding='utf-8-sig'))
    assert noise['state'] == 'complete' and noise['baselineRunId'] == run_id
    noise_id = noise['runRecord']['runId']
    isolation = json.loads((ROOT / 'out/accuracy/noise' / noise_id / 'isolation_audit.json').read_text(encoding='utf-8-sig'))
    assert isolation['allPassed'] and isolation['baselineRunId'] == run_id
    noise_rows = [f"| 原含噪观测 | {noisy['corrected']['contactRmseN']:.6g} | {noisy['corrected']['tipRmseN']:.6g} | {noisy['corrected']['totalRmseN']:.6g} | {noisy['corrected']['maxConstraintResidual']:.3g} |"]
    labels = {'clean_history': '只去掉历史 FBG 噪声', 'clean_current': '只去掉当前 FBG 噪声', 'clean_plane': '只去掉平面观测噪声'}
    for c in noise['cases']:
        noise_rows.append(f"| {labels[c['id']]} | {c['contactRmseN']:.6g} | {c['tipRmseN']:.6g} | {c['totalRmseN']:.6g} | {c['maxConstraintResidual']:.3g} |")
    slip = noise['observedSlipDiagnostic']
    note = f'''# 精度根因与修复验证（2026-09-22）

同一批保存观测，**{len(cases)} 组 / {frames} 帧**完成。无噪滑动接触力向量 RMSE 从 **{clean['baseline']['contactRmseN']:.6g} N** 降至 **{clean['corrected']['contactRmseN']:.6g} N**；含噪滑动从 **{noisy['baseline']['contactRmseN']:.6g} N** 变为 **{noisy['corrected']['contactRmseN']:.6g} N**。无噪问题得到明显改善，含噪分力仍没有解决。数值可行不等于力准确。

## 已确认的原因

1. **观测权重沿用了旧近似模型的数值。** 旧曲率似然标准差始终为 `1.5e-4 /mm`，无噪场景也如此。独立无噪真值代入当前 Cosserat 模型后，48 个弯曲观测的 RMS 偏差仅 `5.6e-12–6.7e-12 /mm`。因此似然过弱，允许随机游走／零力先验以很小代价拉偏接触力和末端力。
2. **原来只检查候选接触点。** 杆身其他位置仍可穿透估计平面。原含噪滑动前三帧用 0.1 mm 网格重算的最小间隙约 `-0.0660/-0.1317/-0.000603 mm`。新增整杆采样非穿透及内部正压力接触相切，并在 MPCC 连续化、精修和最终验收中一致使用。
3. **力学内部积分没有计算量上限。** `fsolve` 的迭代上限并不限制单次 `ode45` 的工作量。此前一次复验在无噪滑动第三帧超过 10 小时未完成；那轮没有调用栈，不能确定唯一停滞位置。现在通过大载荷试探的回归确认 RHS 限额能及时报错，并由优化器拒绝该试探，绝不返回截断杆形。
4. **旧线性形状初值可能离当前非线性平衡太远。** 无噪顶面第 2/3 帧的初始曲率残差很大，紧似然下导致大量拒绝试探与停滞。新增仅使用当前稀疏 FBG 的七变量非线性初值拟合；接触、平面和摩擦仍交给完整 MAP。独立初值诊断使残差平方和约 `273.39 → 0.138`、`1128.05 → 0.176`，不使用真实力。候选变多也可能改变局部解，并不保证含噪误差更小。
5. **优化变量尺度仍只按力先验设置。** 校准似然后，强曲率约束和较弱的先验造成尺度不匹配：力很接近真值，SQP 却停在不可行端点。单独缩小差分步长至 `1e-6/2e-7` 没有解决。只使用局部观测信息对坐标归一化，同一倾斜平面第一帧的最终残差由 `7.061e-5` 降到 `2.644e-13`，退出码 `-2 → 1`；没有改变 MAP 目标或验收阈值。

第一项还有单因素证据：只在已补杆身约束的无噪第一帧改变曲率标准差，接触误差 `0.342287 N → 0.00187590 N`（`1e-5 /mm`），再到 `0.0000191431 N`（`1e-6 /mm`）。未使用真值力初始化。完整对照包含几何、似然、初值与坐标尺度修复，不能把全部改善归因于其中一项。

## 实现与复现

`calibrate_fbg_likelihood(cfg, sigmaSensor, sigmaModel)` 显式设置
`sigmaLikelihood = hypot(sigmaSensor, sigmaModel)`，历史 FBG 保留其传感噪声。仿真已知注入标准差：无噪为 0，含噪为 `5e-5 /mm`；本轮统一声明模型标准差 `1e-6 /mm`。这是数值模型误差设置，不是硬件标定结果，也没有从力误差反推噪声。环境协方差、力先验、观测数组和固定历史模型保持原设置。

新生成的 `force('demos')` 使用这一配置。历史保存输入缺少标定时仍按原权重重放；不会默默改写旧实验。`force('accuracy')` 对固定原 demo 输入作成对复验，结果进入独立运行目录。`force('geometry')` 仍仅比较几何条件，不调整权重。

研究路径启用 `useNonlinearShapeSeed` 和 `useObservationScaling`。后者采用 `min(priorScale, 1/sqrt(diag(H' R^-1 H + P^-1)))` 作为局部坐标尺度，只改变数值求解，不改变数据或概率模型。历史三方法 `force('formulation')` 消融显式保留原初值／尺度与 point-only 配置；它不是最新全部修复的效果入口。

每次 Cosserat 求解默认最多 50,000 次 RHS 计算，可用 `mechanicsMaxRhsEvaluations` 显式配置；该限制不是载荷上限，也不保证找到平衡。每个 MPCC 阶段保留耗时、函数次数、拒绝力学试探数及退出码。长实验逐帧写出 `frame_001.mat` 等检查点；检查点明确标为 `frame-complete`，不冒充完整估计结果。

本轮记录的单帧耗时 `{min(seconds):.1f}–{max(seconds):.1f} s`。实验与回归并行运行，不能当公平运行时比较；当前算法也没有实时性证据。

```matlab
r = force('accuracy');
addpath('rod'); audit_accuracy_run(r.runRecord.runId);
audit_friction_quality(r.runRecord.runId);
run_noise_attribution(r.runRecord.runId);
audit_noise_attribution;
force('check');
```

```text
python scripts/render_accuracy_report.py
python scripts/render_notes.py docs/accuracy.md README.md docs/STATUS.md
```

## 同输入力误差

各行是“原结果 → 修复后”。RMSE 逐帧计算世界坐标力向量误差，单位 N，不是仅比较大小。

| 场景 | 接触力 RMSE | 末端力 RMSE | 合力 RMSE |
|---|---:|---:|---:|
{chr(10).join(rows)}

![力向量误差对照，纵轴为对数](../out/accuracy/{run_id}/comparison.png)

## 物理与数值检查

独立后验核验使用更严格 ODE 容差 `2e-8` 和 `0.1 mm` 网格，相对**估计平面**检查杆身，并直接计算摩擦做功、法向分量和欧氏锥界。{physics_status}；最小间隙 `{final_gap:.4g} mm`，最大最终约束残差 `{final_res:.3g}`。优化本身用 `0.5 mm` 网格；有限采样不是连续无穿透的数学证明。

| 场景 | 稠密最小间隙 mm：原 → 新 | 新最终约束残差 | 新需复核帧数 |
|---|---:|---:|---:|
{chr(10).join(physics_rows)}

本轮中间连续化阶段共 `{mid_exits}` 次非正退出、拒绝力学试探共 `{rejected}` 次；最终结果与中间失败分别保留。不能将最终通过写成“每一级都收敛”。本次工程回归及旧输入重放账本见 [accuracy_full_checks.json](../out/completion/accuracy_full_checks.json)，最后的质量标志／公开接口／数值回归／语法复查见 [accuracy_supplement_checks.json](../out/completion/accuracy_supplement_checks.json)。

另外修正了质量标志：旧 `frictionDirectionResolved` 仅凭互补成立就可能置 true。现在把 `frictionKinematicsConsistent`（方程一致）、`frictionDirectionResolutionAssessed`（是否做过观测分辨检查）和方向是否可分辨分开；完整 MPCC 的含噪分支未做模式置信度评估时，会明确触发 `hasFrictionObservationWarning` 和复核，保持所有摩擦约束。

归档的原 flags 原样保留，另用[质量复核记录](../out/accuracy/{run_id}/quality_audit.json)计算上表“需复核”列。它不重算或修改任何力、形状或退出码。`forceAccuracyCertified` 和 `forceUncertaintyCoverageValidated` 仍为 false；数值通过不能代替方向分辨或真实力误差。

## 含噪问题仍在哪里

含噪力误差仍约 N 级。局部六力到 48 个弯曲观测的导数满秩，但条件数约 `{min(nd['sensitivityConditionNumber']):.1f}–{max(nd['sensitivityConditionNumber']):.1f}`；仅固定杆参数、基座与接触弧长的噪声传播诊断，多个力分量标准差约 `0.7–1.2 N`。它未计环境／接触约束，也不是完整后验或精度下界，不能拿它证明误差不可避免。

该含噪估计的弯曲预测与真值之间 RMS 偏差，以曲率似然标准差计为 `{', '.join(f'{v:.3f}' for v in nd['bendingPredictionTruthRmseInLikelihoodSigma'])}`。需要一起看形状与分力，不能仅因形状吻合就认为两类力分开估准。下一项仍是具有前驱力学平衡的时间窗口、噪声与标定失配矩阵，以及联合不确定性；现有 48 维历史隐变量只有积分运动学。

新增非线性初值／坐标尺度前，同一标定权重下也补做了历史隐变量对照：接触 RMSE `1.02801942 → 1.02484984 N`，仅约 0.31%；末端 `.596905178 → .587551061 N`，合力 `.498683022 → .499035602 N`。这是前版的单因素诊断，不能与本轮数值混合作方法排名。因此没有把 latent 路径换成默认，也没有声称联合历史已解决问题。[单因素 MAT、日志与复现说明](../out/accuracy/probes/README.md)保留所有结果。

## 分开检查三个噪声来源

同一物理场景的真实配对滑移为 `0.02 mm`。当前／历史含噪 FBG 分别积分之后，观测切向位移变成 `{', '.join(f'{v:.3f}' for v in slip['observedSlipMm'])} mm`，与真实方向的余弦为 `{', '.join(f'{v:.3f}' for v in slip['observedDirectionCosine'])}`。这一诊断使用真实弧长做后验核对，没有把它传入估计器。严格满足含噪历史导出的摩擦条件，仍可能偏离实际滑移。

下面逐项换成同一场景保存的**无噪稀疏观测**，其他观测、似然权重、先验和求解器设置保持一致。它们是仿真归因控制，不是用真实力帮助算法，也不是可部署的降噪方法。各因素相互影响，不能把改善相加或据一个随机种子概括普遍规律。

| 输入控制 | 接触力 RMSE / N | 末端力 RMSE / N | 合力 RMSE / N | 最终约束残差 |
|---|---:|---:|---:|---:|
{chr(10).join(noise_rows)}

[噪声归因原始记录](../out/accuracy/noise/{noise_id}/comparison.json)保留逐项结果、质量标志、源码与输入校验和；[隔离审计](../out/accuracy/noise/{noise_id}/isolation_audit.json)逐字段确认除指定观测和输出目录外，输入／权重／配置与基线完全相同，并保留优化告警。此处没有宣称控制组全部收敛或对它们另做连续几何认证。同目录有各控制的输入和完整 MAT。复现：`run_noise_attribution('{run_id}')`。

## 结果位置与限制

- [对照 JSON](../out/accuracy/{run_id}/comparison.json)、[逐帧力 CSV](../out/accuracy/{run_id}/forces.csv)、[物理审计](../out/accuracy/{run_id}/physics_audit.json)、[敏感度与退出诊断](../out/accuracy/{run_id}/diagnostics.json)。各场景目录有输入、完整 MAT 和逐帧检查点。
- 运行 `{run_id}`；固定基线 `{report['sourceRunId']}`。JSON 记录源代码和原输入 SHA-256。
- [中断的旧几何运行](../out/geometry/4945b2ad-21b9-4b1e-8c63-7690aeb050e3/comparison.json)保留为 `interrupted`，只有含噪一组完成；未把它计为六组成功。
- [坐标缩放修复前的失败复验](../out/accuracy/91bd9529-e1ee-426e-845a-17449b0de2bd/physics_audit.json)保留三组物理验收失败，未删除或放宽阈值。
- [原六场景播放器](../out/demos/index.html)仍展示原运行，不能用其动画冒充本轮新结果。
- 当前为模拟两路弯曲 FBG＋模拟平面观测；还依赖单独输入的环境几何，并未实现仅靠 FBG 重建任意未知环境。无真实硬件、真实相机、统计覆盖率或空间多接触验证。仍是递推约束 MAP，Cosserat/MPCC/先验方法及缺陷修复本身不能单独作为 RA-L 创新声明。
'''
    (ROOT / 'docs/accuracy.md').write_text(note, encoding='utf-8')
    print(f'Wrote docs/accuracy.md, CSV and diagnostic plots for {run_id}.')


if __name__ == '__main__':
    main()
