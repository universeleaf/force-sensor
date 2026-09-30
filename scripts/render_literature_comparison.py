"""Verify recorded MATLAB comparisons, export source tables and publication figures."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'out' / 'benchmarks' / 'literature'
FIGS = FOLDER / 'figures'
LABELS = {'EnFiRCE': 'EnFiRCE', 'point': 'Point curvature LS', 'gaussian': 'Gaussian curvature LS'}
COLORS = {'EnFiRCE': '#326D9C', 'point': '#A56B43', 'gaussian': '#7D718F'}
SCENES = {'two_contact': 'S-channel', 'tapered_two_contact': 'Tapered channel',
          'three_contact': 'Three contacts', 'rotated_two_contact': 'Rotated channel',
          'three_contact_noisy': 'Noisy three contacts', 'spatial_sliding': 'Spatial sliding',
          'spatial_sliding_noisy': 'Noisy spatial sliding'}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(fig: plt.Figure, stem: str) -> None:
    for ext in ('pdf', 'svg', 'png'):
        path = FIGS / f'{stem}.{ext}'
        fig.savefig(path, dpi=300, bbox_inches='tight', facecolor='white')
        if ext == 'svg':
            # Keep vector paths unchanged while avoiding noisy Git whitespace.
            path.write_text('\n'.join(line.rstrip() for line in path.read_text(encoding='utf-8').splitlines())+'\n', encoding='utf-8')
    plt.close(fig)


def main() -> None:
    report = json.loads((FOLDER / 'comparison.json').read_text(encoding='utf-8'))
    if report['state'] != 'complete' or report['failureCount']:
        raise RuntimeError('Do not publish a running or incomplete experiment.')
    reference_path = ROOT / report['referencePath']
    if sha(reference_path) != report['referenceComparisonSha256']:
        raise RuntimeError('Reference comparison changed after replay.')
    ref = json.loads(reference_path.read_text(encoding='utf-8'))
    FIGS.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in ref['cases']:
        base = reference_path.parent / case['artifactFolder']
        for name in ('input', 'truth', 'estimate'):
            if sha(base / f'{name}.mat') != case['artifactSha256'][name]:
                raise RuntimeError(f'Reference {case["id"]}/{name} corrupted.')
        rows.append({'sceneId': case['id'], 'method': 'EnFiRCE', **case['metrics']})
    for case in report['cases']:
        if case['completed']:
            base = FOLDER / case['artifactFolder']
            if sha(base / 'estimate.mat') != case['estimateSha256'] or sha(base / 'forces.csv') != case['csvSha256']:
                raise RuntimeError(f'Baseline {case["sceneId"]}/{case["method"]} corrupted.')
            rows.append({'sceneId': case['sceneId'], 'method': case['method'], **case['metrics']})
    fields = ['sceneId', 'method', 'frameCount', 'candidateCount', 'contactForceRmseN',
              'contactMagnitudeMaeN', 'contactArcRmseMm', 'tipForceRmseN', 'totalForceRmseN',
              'shapeRmseMm', 'reviewCount', 'optimizationSeconds']
    with (FOLDER / 'source_data.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fields, extrasaction='ignore'); writer.writeheader(); writer.writerows(rows)
    lookup = {(r['sceneId'], r['method']): r for r in rows}
    noisy = ['three_contact_noisy', 'spatial_sliding_noisy']
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'DejaVu Sans'],
                         'font.size': 8, 'pdf.fonttype': 42, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False})
    # Contract: same noisy observations expose the force/location benefit and
    # the difference between component accuracy and total resultant accuracy.
    fig, axes = plt.subplots(1, 3, figsize=(7.05, 2.35), layout='constrained')
    for ax, field, title in zip(axes, ['contactForceRmseN', 'contactArcRmseMm', 'totalForceRmseN'],
                               ['Contact vector RMSE (N)', 'Contact arc RMSE (mm)', 'Total vector RMSE (N)']):
        for j, method in enumerate(LABELS):
            values = [lookup.get((sid, method), {}).get(field, np.nan) for sid in noisy]
            ax.bar(np.arange(2) + (j-1)*0.24, values, width=0.22, color=COLORS[method], label=LABELS[method])
        ax.set_xticks([0, 1], ['Three\ncontacts', 'Spatial\nsliding']); ax.set_ylabel(title)
        ax.grid(axis='y', alpha=0.16); ax.set_axisbelow(True)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside upper center', ncol=3, fontsize=7)
    for i, ax in enumerate(axes):ax.text(-0.18, 1.05, chr(97+i), weight='bold', transform=ax.transAxes)
    save(fig, 'noisy_baseline_comparison')
    # Clean cases are numerical consistency evidence, kept separate so they
    # cannot visually dominate the practically relevant noisy errors.
    clean = [sid for sid in SCENES if sid not in noisy]
    fig, ax = plt.subplots(figsize=(3.45, 2.25), layout='constrained')
    for method in LABELS:
        values = [lookup.get((sid, method), {}).get('contactForceRmseN', np.nan) for sid in clean]
        ax.plot(range(len(clean)), values, 'o-', ms=3, label=LABELS[method], color=COLORS[method])
    ax.set_yscale('log'); ax.set_ylabel('Contact vector RMSE (N)')
    ax.set_xticks(range(len(clean)), ['S', 'Tapered', 'Three', 'Rotated', 'Spatial'], fontsize=7)
    ax.legend(fontsize=7); ax.grid(axis='y', alpha=0.16)
    save(fig, 'clean_baseline_consistency')
    geometry_path = FOLDER / 'geometry.json'
    geometry = json.loads(geometry_path.read_text(encoding='utf-8'))
    if geometry['referenceComparisonSha256'] != sha(reference_path):
        raise RuntimeError('Geometry export does not match the reference run.')
    fig = plt.figure(figsize=(7.05, 3.15), layout='constrained')
    geometry_grid = fig.add_gridspec(2, 3, height_ratios=[1, .09])
    for i, case in enumerate(geometry['cases']):
        source = next(c for c in ref['cases'] if c['id'] == case['id'])
        if case['artifactSha256'] != source['artifactSha256']:
            raise RuntimeError('Geometry source hashes changed.')
        spatial = case['id'] == 'spatial_sliding_noisy'
        ax = fig.add_subplot(geometry_grid[0, i], projection='3d' if spatial else None)
        p, pe = np.array(case['truthP']), np.array(case['estimateP'])
        pc, pce = np.array(case['truthContactP']), np.array(case['estimateContactP'])
        points, normals = np.array(case['planePoint']), np.array(case['planeNormal'])
        if spatial:
            ax.plot(*p, color='#333333', lw=1.5, label='Independent truth')
            ax.plot(*pe, color=COLORS['EnFiRCE'], lw=1, ls='--', label='EnFiRCE')
            ax.scatter(*pc, c=COLORS['point'], s=12)
            ax.scatter(*pce, c=COLORS['EnFiRCE'], s=12, marker='x')
            yy, zz = np.meshgrid(np.linspace(p[1].min()-1, p[1].max()+1, 2), np.linspace(p[2].min(), p[2].max(), 2))
            for point, n in zip(points.T, normals.T):
                xx = point[0] - (n[1]*(yy-point[1])+n[2]*(zz-point[2]))/n[0]
                ax.plot_surface(xx, yy, zz, color='#CCD0D4', alpha=.18, shade=False)
            ax.set_xlabel('x (mm)', labelpad=0, fontsize=7);ax.set_ylabel('y (mm)', labelpad=0, fontsize=7);ax.set_zlabel('z (mm)', labelpad=0, fontsize=7)
            ax.set_box_aspect((1, .75, 2));ax.view_init(elev=14, azim=-55)
            ax.tick_params(labelsize=6, pad=0)
            for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:axis.set_major_locator(MaxNLocator(3))
        else:
            ax.plot(p[0],p[2],color='#333333',lw=1.5,label='Independent truth')
            ax.plot(pe[0],pe[2],color=COLORS['EnFiRCE'],lw=1,ls='--',label='EnFiRCE')
            ax.scatter(pc[0],pc[2],c=COLORS['point'],s=12)
            ax.scatter(pce[0],pce[2],c=COLORS['EnFiRCE'],s=12,marker='x')
            zz=np.linspace(p[2].min()-2,p[2].max()+2,2)
            for point,n in zip(points.T,normals.T):
                xx=point[0]-n[2]*(zz-point[2])/n[0]
                ax.plot(xx,zz,color='#A8AFB5',lw=1)
            ax.set_xlabel('x (mm)');ax.set_ylabel('z (mm)')
            ax.set_xlim(-28,28);ax.set_ylim(p[2].min()-3,p[2].max()+3)
            ax.set_aspect('equal',adjustable='box')
        ax.set_title(chr(97+i)+'  '+['S-channel, clean','Three contacts, noisy','Spatial sliding, noisy'][i],fontsize=8)
        truth_m=np.linalg.norm(case['truthContactForce'],axis=0)
        estimate_m=np.linalg.norm(case['estimateContactForce'],axis=0)
        annotation=' / '.join(f'{a:.1f}:{b:.1f}' for a,b in zip(truth_m,estimate_m))+' N'
        note_ax=fig.add_subplot(geometry_grid[1,i]);note_ax.axis('off')
        note_ax.text(.5,.3,annotation,transform=note_ax.transAxes,ha='center',fontsize=6)
    handles,labels=fig.axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside upper center',ncol=2,fontsize=7)
    save(fig,'solved_contact_geometry')
    # Existing multi-seed experiment is explicitly the ordered planar
    # prototype, not a statistical replication of the new 3-D window solver.
    matrix_path = ROOT / 'out' / 'benchmarks' / 'multi_contact' / 'comparison.json'
    matrix = json.loads(matrix_path.read_text(encoding='utf-8'))
    if matrix['state'] != 'complete' or matrix['failureCount']:
        raise RuntimeError('Planar ablation reference incomplete.')
    methods = ['EnFiRCE_environment', 'shape_only_point_loads', 'ablation_no_gap',
               'ablation_no_tangency', 'ablation_no_geometry_penalties']
    fig, axes = plt.subplots(1, 2, figsize=(7.05, 2.4), layout='constrained')
    for method, color, label in zip(methods[:2], [COLORS['EnFiRCE'],COLORS['point']], ['Planar environment', 'Planar shape only']):
        means, lower, upper = [], [], []
        for density in [8,16,24]:
            seed_rmse = []
            for seed in matrix['seeds']:
                group = [r['contactForceRmseN'] for r in matrix['cases'] if r['completed'] and
                         r['method']==method and r['sensorCount']==density and r['noiseStdPerMm']==2.5e-5 and r['seed']==seed]
                seed_rmse.append(float(np.sqrt(np.mean(np.square(group)))))
            means.append(np.mean(seed_rmse));lower.append(min(seed_rmse));upper.append(max(seed_rmse))
        axes[0].errorbar([8,16,24],means,yerr=[np.array(means)-lower,np.array(upper)-means],
                         fmt='o-', color=color,label=label,capsize=3,ms=4)
    axes[0].set_xticks([8,16,24]); axes[0].set_xlabel('Curvature sensor positions')
    axes[0].set_ylabel('Contact vector RMSE (N)');axes[0].legend(fontsize=7)
    data=[]
    for method in methods:
        values=[r['contactForceRmseN'] for r in matrix['cases'] if r['completed'] and r['method']==method and
                r['sensorCount']==24 and r['noiseStdPerMm']==2.5e-5]
        data.append(values)
    axes[1].boxplot(data, tick_labels=['Env.', 'Shape', 'No gap', 'No tangent', 'Neither'], showfliers=True,
                    medianprops={'color':COLORS['EnFiRCE']}, widths=0.55)
    axes[1].set_ylabel('Per-state contact vector RMSE (N)');axes[1].tick_params(axis='x', labelsize=7)
    for i, ax in enumerate(axes):
        ax.grid(axis='y', alpha=0.16);ax.text(-0.16,1.05,chr(97+i),weight='bold',transform=ax.transAxes)
    save(fig, 'planar_density_ablation')
    tex = [r'\begin{table}[t]', r'\caption{Same-curvature noisy comparison; two states per scene, one noise seed. Baselines are disclosed literature adaptations.}',
           r'\label{tab:baseline}', r'\centering\small', r'\begin{tabular}{llrr}', r'\toprule',
           r'Scene & Method & Force (N) & Arc (mm)\\', r'\midrule']
    summary = ['# 同观测文献适配基线比较', '', '更新：2026-10-01。', '',
               '这是一组已实际完成的 MATLAB 比较。七个相同输入窗口，各有两个输出状态；两个含噪场景各只有一个噪声种子。不能据此宣布 SOTA。', '',
               '完整曲率输入、校准、独立真值和文件 SHA-256 与原窗口实验一致。基线只共享 EnFiRCE 从观测生成的候选数；不读取真值接触数、位置或载荷。基线位置初始化来自自身曲率重构。这个比较因此以相同候选数为条件，不能评价未知接触数的识别。', '',
               '| 场景 | 方法 | 接触向量 RMSE / N | 接触大小 MAE / N | 位置 RMSE / mm | 末端 RMSE / N | 合力 RMSE / N | 复核 / 2 |',
               '|---|---|---:|---:|---:|---:|---:|---:|']
    for sid in SCENES:
        for method in LABELS:
            r=lookup.get((sid,method))
            if not r:
                summary.append(f'| {SCENES[sid]} | {LABELS[method]} | 运行失败 | — | — | — | — | — |');continue
            summary.append(f"| {SCENES[sid]} | {LABELS[method]} | {r['contactForceRmseN']:.6g} | {r['contactMagnitudeMaeN']:.6g} | {r['contactArcRmseMm']:.6g} | {r['tipForceRmseN']:.6g} | {r['totalForceRmseN']:.6g} | {r['reviewCount']}/2 |")
            if sid in noisy:
                scene='Three contacts' if sid==noisy[0] else 'Spatial sliding'
                short={'EnFiRCE':'EnFiRCE','point':'Point LS','gaussian':'Gaussian LS'}[method]
                tex.append(f"{scene} & {short} & {r['contactForceRmseN']:.3f} & {r['contactArcRmseMm']:.3f}\\\\")
    tex += [r'\bottomrule',r'\end{tabular}',r'\end{table}']
    (FIGS/'baseline_table.tex').write_text('\n'.join(tex)+'\n',encoding='utf-8')
    summary += ['', '点载荷方法借鉴 Xiao–Chen 的曲率最小二乘，使用一般局部坐标 Cosserat 方程而不是只适用直杆的简化式；Gaussian 方法保留 Aloi 的局部横向 Gaussian 载荷参数化，把原位置似然改为实际可用的两通道曲率似然。两者都允许未知三维末端力，以完整非线性平衡预测观测；独立按帧求解。它们是本项目的可核实适配实现，不能写成原作者官方实现或完整原论文复现。', '',
                'EnFiRCE 的参考是原始完整两时刻 MAP，含共享几何、过程因子和摩擦互补；基线没有环境观测与时间过程因子。因此性能差异是完整方法间比较，不单独归因于某一约束。宽度下界为 0.25 mm；局部最优和拟合复核保留。', '',
                '无噪声点载荷基线也达到了数值一致，有些指标比 EnFiRCE 更低。含噪两组中 EnFiRCE 的接触向量和位置误差更低，但这只支持这些输入下的局部结论。三接触合力 RMSE 为 EnFiRCE 0.820 N、点载荷 0.632 N：我们的接触分力更准，但合力并非所有条件下最优。历史 EnFiRCE 优化时间和新基线时间并非统一性能测量，不用于速度排名。', '',
                f"本轮运行 ID：`{report['runRecord']['runId']}`；参考窗口：`{report['referenceRunId']}`；失败数：{report['failureCount']}。", '',
                '[逐项原始指标](comparison.json) · [绘图源数据](source_data.csv) · [已求解接触场景](figures/solved_contact_geometry.png) · [论文图：含噪比较](figures/noisy_baseline_comparison.png) · [论文图：无噪声一致性](figures/clean_baseline_consistency.png) · [二维传感器密度与消融](figures/planar_density_ablation.png)', '',
                '来源：[Xiao–Chen 2021](https://arxiv.org/abs/2109.12469)；[Aloi et al. 2022](https://doi.org/10.1109/LRA.2022.3188905)。']
    (FOLDER/'summary.md').write_text('\n'.join(summary)+'\n',encoding='utf-8')
    audit={'runId':report['runRecord']['runId'],'comparisonSha256':sha(FOLDER/'comparison.json'),
           'sourceDataSha256':sha(FOLDER/'source_data.csv'),'planarComparisonSha256':sha(matrix_path),
           'geometryDataSha256':sha(geometry_path),
           'scriptSha256':sha(Path(__file__)),'backend':'Python/matplotlib',
           'figureContract':'Archived solved first-state geometry with true/estimated forces; noisy same-input force and location comparison; clean numerical consistency separate; planar density/ablation explicitly separate.',
           'statistics':'Noisy comparison: 2 states, one seed per scene, no inferential intervals. Planar density: mean and min/max of 3 seed-level pooled state RMSEs. Boxplots: 18 per-state RMSEs, descriptive quartiles, correlated states, no significance test.',
           'authorsOfficialImplementations':False,'sotaEstablished':False}
    (FIGS/'provenance.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(f'Verified {len(rows)} method/scene records; exported 4 figures and source data.')


if __name__=='__main__':main()
