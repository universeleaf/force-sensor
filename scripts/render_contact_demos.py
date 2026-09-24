"""Package actual MATLAB demo outputs into an offline viewer and animations.

No force estimates or motion interpolation are computed here. Each animation
holds the three solved states; the sensor-pair interval is not playback time.
Run with the workspace's Python after force('demos'). Requires Pillow for GIFs.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def bounds(data):
    points = [p for f in data['frames'] for key in ('pTrue', 'pEstimated', 'pObserved') for p in f[key]]
    xs, zs = [p[0] for p in points], [p[2] for p in points]
    cx, cz = (min(xs)+max(xs))/2, (min(zs)+max(zs))/2
    span = max(max(xs)-min(xs), max(zs)-min(zs), 60)*1.35
    return cx-span/2, cx+span/2, cz-span/2, cz+span/2


def clip_obstacle(box, origin, normal):
    """Clip view rectangle to the obstacle half-space n.(p-p0) <= 0."""
    xmin, xmax, zmin, zmax = box
    polygon = [(xmin, zmin), (xmax, zmin), (xmax, zmax), (xmin, zmax)]
    out = []
    def gap(p):
        return normal[0]*(p[0]-origin[0])+normal[2]*(p[1]-origin[2])
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        ga, gb = gap(a), gap(b)
        if ga <= 0:
            out.append(a)
        if (ga <= 0) != (gb <= 0):
            t = ga/(ga-gb)
            out.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
    return out


def render_animation(data, folder):
    """Portable GIF + last-state PNG, real computed frames only."""
    font_path = Path('C:/Windows/Fonts/msyh.ttc')
    def font(size):
        return ImageFont.truetype(str(font_path), size) if font_path.exists() else ImageFont.load_default(size=size)
    box = bounds(data)
    xmin, xmax, zmin, zmax = box
    left, top, size = 48, 104, 560
    scale = size/(xmax-xmin)
    def point(p):
        return left+(p[0]-xmin)*scale, top+(zmax-p[2])*scale
    def point2(p):
        return point((p[0], 0, p[1]))
    truth_color, estimate_color = '#177c69', '#dc7136'
    frames = []
    for f in data['frames']:
        im = Image.new('RGB', (1100, 760), '#f5f6f2')
        d = ImageDraw.Draw(im)
        d.text((36, 22), data['scene']['title'], font=font(28), fill='#162a35')
        d.text((36, 65), f"离散求解状态 {f['index']} / {len(data['frames'])} · 推进 {f['pushMm']:g} mm", font=font(17), fill='#52656c')
        poly = clip_obstacle(box, data['planePoint'], data['planeNormal'])
        if len(poly) >= 3:
            d.polygon([point2(p) for p in poly], fill='#dce1de')
        d.rectangle((left, top, left+size, top+size), outline='#c4cfcd', width=1)
        d.line([point(p) for p in f['pObserved']], fill='#9ba6ae', width=2)
        d.line([point(p) for p in f['pTrue']], fill=truth_color, width=6)
        for i, (a, b) in enumerate(zip(f['pEstimated'], f['pEstimated'][1:])):
            if i % 5 < 3:
                d.line((point(a), point(b)), fill=estimate_color, width=4)
        def arrow(p, force, color):
            if norm(force) < .05:
                return
            a, b = point(p), point([p[i]+4*force[i] for i in range(3)])
            d.line((a, b), fill=color, width=3)
            theta = math.atan2(b[1]-a[1], b[0]-a[0])
            d.polygon([b, (b[0]-10*math.cos(theta-.45), b[1]-10*math.sin(theta-.45)),
                       (b[0]-10*math.cos(theta+.45), b[1]-10*math.sin(theta+.45))], fill=color)
        for suffix, color in [('True', truth_color), ('Estimated', estimate_color)]:
            arrow(f['contact'+suffix], f['fc'+suffix], color)
            arrow(f['p'+suffix][-1], f['fe'+suffix], color)
        d.text((left, top+size+12), f"X ∈ [{xmin:.0f}, {xmax:.0f}], Z ∈ [{zmin:.0f}, {zmax:.0f}] mm", font=font(16), fill='#52656c')
        x, y = 652, 119
        d.text((x,y), '力作用在杆上 · N', font=font(22), fill='#162a35')
        y += 52
        d.text((x,y), '绿色：真值    橙色：估计', font=font(19), fill='#52656c')
        for title, key, err in [('接触力', 'fc', 'contactErrorN'), ('末端力', 'fe', 'tipErrorN'), ('合力', 'total', 'totalErrorN')]:
            y += 57
            d.text((x,y), title, font=font(19), fill='#162a35')
            y += 30
            d.text((x,y), f"{norm(f[key+'True']):.3f}  →  {norm(f[key+'Estimated']):.3f}", font=font(28), fill='#162a35')
            y += 38
            d.text((x,y), f"向量误差 {f[err]:.4f} N", font=font(17), fill='#52656c')
        d.text((652, 621), '数值诊断：'+('需复核' if f['requiresReview'] else '当前检查通过'), font=font(18), fill='#9b501e' if f['requiresReview'] else '#177c69')
        d.text((36, 711), '独立平面仿真 / 三维逆估计 · 每帧停留 1.4 秒 · 无插值 · 力箭头 4 mm/N', font=font(17), fill='#52656c')
        frames.append(im)
    frames[-1].save(folder/'preview.png')
    frames[0].save(folder/'demo.gif', save_all=True, append_images=frames[1:], duration=1400, loop=0, optimize=False)


def load_cases(folder, report):
    cases = []
    for entry in report['cases']:
        data = None
        case_folder = (folder/entry['artifactFolder']).resolve()
        if not case_folder.is_relative_to(folder.resolve()):
            raise ValueError('Artifact path must stay inside the demo output folder.')
        if entry['completed']:
            data = json.loads((case_folder/'data.json').read_text(encoding='utf-8'))
            if data['scene']['id'] != entry['id'] or len(data['frames']) != entry['frames']:
                raise ValueError('Scene identity or frame count does not match the report.')
            # Presentation should never silently use magnitudes as vector errors.
            for f in data['frames']:
                error = norm([a-b for a,b in zip(f['fcTrue'], f['fcEstimated'])])
                if abs(error-f['contactErrorN']) > 1e-8:
                    raise ValueError('Contact vector error does not match the stored vectors.')
            render_animation(data, case_folder)
        cases.append({'entry':entry, 'data':data})
    return cases


def write_report(report, cases):
    """Keep the technical note tied to the same run as the viewer."""
    def n(value, digits=4):
        return '—' if value is None else f'{value:.{digits}f}'
    rows, last_rows, diagnostics, assets = [], [], [], []
    for case in cases:
        e, d = case['entry'], case['data']
        if d is None:
            rows.append(f"| {e['title']} | 失败 | — | — | — | {e['failureStage']}：{e['error']} |")
            continue
        review = sum(f['requiresReview'] for f in d['frames'])
        rows.append(f"| {e['title']} | {e['frames']} | {n(e['contactRmseN'])} | {n(e['tipRmseN'])} | {n(e['totalRmseN'])} | {review}/{e['frames']} 帧需复核 |")
        f = d['frames'][-1]
        last_rows.append(f"| {e['title']} | {f['pushMm']:g} | {n(norm(f['fcTrue']))} | {n(norm(f['fcEstimated']))} | {n(f['contactErrorN'])} |")
        diagnostics.append(f"| {e['title']} | {n(e['trueShapeRmseMm'])} | {n(e['contactLocationRmseMm'])} | {e['truthMaxPenetrationMm']:.2e} | {e['estimatedMaxPenetrationMm']:.4f} | {min(e['frameSeconds']):.1f}–{max(e['frameSeconds']):.1f} |")
        base = '../out/demos/'+e['artifactFolder']
        assets.append(f"- **{e['title']}**：[GIF]({base}/demo.gif) · [末帧图]({base}/preview.png) · [逐帧力 CSV]({base}/forces.csv) · [几何与力 JSON]({base}/data.json) · [完整结果 MAT]({base}/results.mat)。")
    by_id = {c['entry']['id']:c['data'] for c in cases if c['data'] is not None}
    conclusion = ''
    frictionless = [d for d in by_id.values() if d['scene']['frictionMu'] == 0]
    if frictionless:
        errors = [d['summary']['contactRmseN'] for d in frictionless]
        conclusion = (f"本次已完成的 {len(frictionless)} 组无摩擦场景，接触力向量 RMSE 为 "
                      f"**{min(errors):.4f}–{max(errors):.4f} N**。这支持当前算法在这些已标定、无注入噪声的单接触场景中取得较小误差。")
    paired = ''
    if 'sliding_clean' in by_id and 'sliding_noisy' in by_id:
        clean, noisy = by_id['sliding_clean'], by_id['sliding_noisy']
        for a,b in zip(clean['frames'], noisy['frames']):
            for key in ('pTrue','fcTrue','feTrue'):
                if a[key] != b[key]:
                    raise ValueError('Clean/noisy truth differs; not a controlled noise comparison.')
        paired = (f"同一滑动真值下，无噪声／含噪的接触力向量 RMSE 为 **{n(clean['summary']['contactRmseN'])} / {n(noisy['summary']['contactRmseN'])} N**，"
                  f"末端力为 **{n(clean['summary']['tipRmseN'])} / {n(noisy['summary']['tipRmseN'])} N**。"
                  "这只是一组固定随机种子的配对比较，不是噪声鲁棒性的统计结论。")
        last = noisy['frames'][-1]
        relative = last['contactErrorN']/norm(last['fcTrue'])*100
        conclusion += (f" 含噪声滑动的最后一帧真实／估计接触力大小为 "
                       f"**{norm(last['fcTrue']):.4f} / {norm(last['fcEstimated']):.4f} N**，"
                       f"向量相对误差 **{relative:.1f}%**；该场景的分力精度仍不足。"
                       "其数值检查仍全部通过，直接说明当前数值质量标志不能筛出所有力估计错误。")
    content = f'''# 六组连续体杆—环境接触 demo：真值、估计力与误差

更新：2026-09-19。本次运行状态：`{report['state']}`；运行编号：`{report['runRecord']['runId']}`。本页数字由 `scripts/render_contact_demos.py` 从本次保存的 JSON 生成，未把旧输出混入新结果。

**先打开[可播放的离线演示](../out/demos/index.html)**，切换六个场景，用滑块查看三帧真实杆形、估计杆形、接触位置、真实／估计的接触力、末端力和合力。每组还提供 GIF、PNG、CSV 与 MAT。播放只重复已算出的离散状态，没有添加插值估计帧。

{conclusion}

## 项目仍在解决什么

本项目用稀疏形状信息和环境信息估计力，贴合 `papers/Formulation.pdf` 的状态与接触约束。未知量包括环境平面修正、杆身接触弧长、法向力、摩擦生成系数和独立末端载荷。当前算法保留三维非线性 Cosserat 平衡、完整法向／切向／摩擦锥互补与非线性 MAP；这些 demo 没有把未知末端力设为真值，也没有把接触位置送入估计器。

本轮主要补上**物理合法且可观察的多场景演示**，没有把六个场景称为六项新算法，也没有修改材料参数来追求某个目标误差。核心建模和研究缺口见[算法与 Formulation 对照](algorithm_formulation_2026-09-19.md)。

## 六组场景如何构造

| 场景 ID | 环境和杆 | 推进量 mm | 真值生成器的末端力（局部 XZ，N） | 噪声／摩擦 |
|---|---|---|---|---|
| ceiling_hook | 200 mm 弯钩杆，直段 120 mm，弯段半径 30 mm；向上顶 z=160 平面 | 10, 16, 20 | [1, −1] | 无摩擦，无注入噪声 |
| side_wall | 上述几何整体绕 y 旋转 90°，侧向压墙；另换末端载荷 | 12, 17, 21 | [0.4, −0.6] | 无摩擦，无注入噪声 |
| inclined_plane | 基座不旋转，平面经过局部 [0,145] mm，法向倾斜 12° | 12, 17, 21 | [0.5, −0.6] | 无摩擦，无注入噪声 |
| long_soft_rod | 240 mm 杆，直段 144 mm，半径 36 mm，刚度为参考 65%；旋转 180°向下压面 | 12, 20, 26 | [0.4, −0.4] | 无摩擦，无注入噪声 |
| sliding_clean | 参考弯杆沿顶面滑动，配对样本沿 −x 相差 0.02 mm | 18, 20, 22 | [1, −1] | μ=0.3，无注入噪声 |
| sliding_noisy | 与上组相同几何、真值和滑移 | 18, 20, 22 | [1, −1] | μ=0.3，曲率 σ=5×10⁻⁵/mm，平面偏移 σ=0.1 mm |

参考弯曲刚度为 200700 N·mm²，扭转刚度为弯曲刚度/1.3；长软杆二者均乘 0.65。视频场景只近似杆形，视频没有提供可用于毫米标定和材料辨识的全部信息，不能称为视频实验的精确数字孪生。

传感器为 24 个位置的两个弯曲曲率通道；固有曲率、基座姿态和刚度作为已知标定。第三个曲率为标定扭转假设，不是实测通道。采用 `intrinsic-delta` 重建，当前与前驱样本间隔 20 ms；三帧推进量是抽样工况，并不代表只相隔 20 ms 的连续运动。随机种子固定为 93。

环境输入为模拟平面观测，本轮没有走合成深度图前端。平面点的各向同性似然标准差为 max(0.03 mm, 注入平面噪声)，法向分量标准差为 0.001；后者是建模下限，没有注入法向噪声。当前传感器噪声与 MAP 的似然权重并非全部逐项匹配，估计器其余权重沿用公开默认配置，完整配置保存在 MAT。

## 独立真值与估计之间的边界

`build_contact_demo_truth` 调用独立二维连续射击求解器，联立自由端零力矩、接触零间隙和杆与平面相切，求反力与连续接触弧长。真值检查整杆 1601 个采样位置、固有曲率分段点和接触点的非穿透；容许穿透 <10⁻⁶ mm、末端／相切残差 <10⁻⁵ N·mm。它不调用逆估计器的三维平衡映射。

真值生成器只支持单个光滑杆身接触或无接触，属于局部静态平衡解，未证明唯一性或全局稳定性。滑动真值预先指定切向／法向力比 0.3，再将配对平衡形状沿无限平面平移，形成合法的耗散方向；没有独立求解从粘着到滑动的转换。

`simulate_sensor_packet` 只输出曲率样点、基座位姿、环境观测、摩擦系数和时间戳。`estimate_sensor_forces` 接收 `tube + packet + config`，不接收力真值、接触弧长或稠密真实杆形。真值仅在生成传感器观测和事后评分时使用。二维真值与三维逆模型共享物理假设和标定参数，因此仍是理想参数下的独立实现验证，不是材料失配验证。

## 准不准：三帧整体误差

下表是三维**向量** RMSE，即 √mean(‖估计 − 真值‖²)，不是只比较幅值。无接触帧同样计入力误差；优化或诊断警告不会被删掉。

| 场景 | 帧数 | 接触力 RMSE N | 末端力 RMSE N | 合力 RMSE N | 数值诊断 |
|---|---:|---:|---:|---:|---|
{chr(10).join(rows)}

### 各场景最后一帧的接触力大小

| 场景 | 推进 mm | 真实大小 N | 估计大小 N | 向量误差 N |
|---|---:|---:|---:|---:|
{chr(10).join(last_rows)}

{paired}

### 形状、接触位置、穿透与耗时

| 场景 | 真实形状 RMSE mm | 接触弧长 RMSE mm | 真值最大穿透 mm | 估计形状相对真实平面最大穿透 mm | 单帧耗时 s |
|---|---:|---:|---:|---:|---:|
{chr(10).join(diagnostics)}

弧长误差只统计真实接触力 >0.05 N 的帧；无接触时接触弧长没有物理定义。估计穿透按输出杆节点相对**真实平面**计算；噪声下估计平面可能偏移，所以该值不能单独解释为违反估计器内部约束。逆问题目前只对一个候选接触点施加接触约束，未对整杆加入连续碰撞约束。真值的密集非穿透检查和逆结果的采样穿透诊断是两件事。

质量字段只反映数值与观测一致性，不认证力的精度。完整互补可能成立而分力仍不准确，因而页面始终把接触力、末端力和合力分开显示。`requiresReview` 反映最终接受解；连续化中间阶段可能出现非正退出，随后收紧约束并恢复，完整各级退出记录在 MAT 的 `output.ours.solverTrace`，不能把最终通过解释为每级都无告警。耗时来自实际求解；离线动画不构成实时性证明。

## 本轮修正的错误

1. **撤回旧 demo 的接触真值资格。** 旧 `out/formulation/demo_suite/` 先给任意点载荷求杆形，再把平面放在该点，没有保证整杆非穿透或接触相切。几何审计发现 floor_midbody / inclined_sticking / tip_contact_degenerate / nonplanar_contact_stress 最大穿透分别为 65.3992 / 11.6615 / 20.6475 / 16.3294 mm；旧侧墙生成还因基座落入障碍物失败。原文件保留并[标记无效](../out/formulation/demo_suite/INVALID_GEOMETRY.md)，未当作新结果使用。
2. **重建场景生成和输出。** `contact_demo_scenes` 定义明确参数，`build_contact_demo_truth` 先解独立合法平衡，再生成传感器数据。`run_contact_demo_suite` 每次使用独立运行目录，逐场景保存结果和失败阶段，输出评分、原始向量和杆形。新接口 `sensor-config` 从当前默认标定配置建立输入，不再读取旧实验模板或负载标签。
3. **纠正历史不确定性标志。** 有历史曲率噪声标定不代表 MAP 已对历史隐状态建模。`historyUncertaintyModeled=false`；单独报告 `historyNoiseCalibrationAvailable`。旧 active-set 路径的噪声模式分辨与完整 MPCC 的模式优化分开标记；完整 MPCC 仍以重建的历史形状为条件，没有把历史噪声纳入联合似然。
4. **交付可复查的演示。** 离线 HTML 嵌入本次真实 JSON；GIF 每帧与真实保存结果一一对应，末帧 PNG 可直接用于组会。所有场景保留，未通过数值检查的帧也显示，未用动画插值增加“实验帧数”。

## 如何运行和重放

```matlab
% MATLAB Current Folder 设为项目根目录
report = force('demos');

% 只重算一组；每次都会建立新运行目录
addpath('rod');
report = run_contact_demo_suite('inclined_plane');

% 从某个 results.mat 单独重放估计，真值不传给估计器
a = load('out/demos/<run-id>/<scene-id>/results.mat');
output = estimate_sensor_forces(a.sensorInput);
```

```powershell
python scripts/render_contact_demos.py
python scripts/render_notes.py docs/contact_demos_2026-09-19.md
```

渲染脚本只读已算结果，不启动 MATLAB。`out/demos/comparison.json` 指向最近一次完整或部分运行；各次 `<run-id>/comparison.json` 和 MAT 持久保存，记录 MATLAB 版本、源码哈希和场景配置。只跑一个子集时，首页也只显示该次子集，不悄悄拼接旧成功案例。

## 原始数据和动画

{chr(10).join(assets)}

## 接下来要补的算法能力

1. 在保持完整接触互补的前提下，把当前／前驱两帧的潜在真实形状和 FBG 似然放进同一个窗口。当前完整 MPCC 仍把有噪声的历史重建固定下来，优化变量无法正确吸收这部分误差。
2. 在这六组可复现场景上增加噪声种子、刚度／固有曲率失配和环境偏差；区分大小、方向、两种分力、接触位置和失败率，不能只汇总合力。
3. 扩展独立非共面三维接触、粘滑转换真值及整杆非穿透约束。当前所有真值是平面静力学，不能拿旋转场景代替三维载荷验证。
4. 依照[算法报告](algorithm_formulation_2026-09-19.md)对齐公开论文的观测信息和适用载荷，再做公平基线。现有 demo 没有运行文献方法，不能从这些数字声称优于它们，也不足以宣布 RA-L 投稿就绪。
'''
    (ROOT/'docs/contact_demos_2026-09-19.md').write_text(content, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, default=ROOT/'out/demos')
    args = parser.parse_args()
    folder = args.folder.resolve()
    report = json.loads((folder/'comparison.json').read_text(encoding='utf-8'))
    cases = load_cases(folder, report)
    bundle = {'report':report, 'cases':cases}
    payload = json.dumps(bundle, ensure_ascii=False, separators=(',',':'), allow_nan=False).replace('<','\\u003c')
    template = (ROOT/'scripts/contact_demo_viewer.html').read_text(encoding='utf-8')
    target = folder/'index.html'
    target.write_text(template.replace('__DEMO_DATA__', payload), encoding='utf-8')
    write_report(report, cases)
    print(f"Wrote {target}; {sum(c['data'] is not None for c in cases)} scenes; state={report['state']}")


if __name__ == '__main__':
    main()
