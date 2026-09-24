# 六组连续体杆—环境接触 demo：真值、估计力与误差

更新：2026-09-24。本次运行状态：`complete`；运行编号：`a3244666-2bd8-4d80-ae95-5f6ac5e00d42`。本页数字由 `scripts/render_contact_demos.py` 从本次保存的 JSON 生成，未把旧输出混入新结果。

**先打开[可播放的离线演示](../out/demos/index.html)**，切换六个场景，用滑块查看三帧真实杆形、估计杆形、接触位置、真实／估计的接触力、末端力和合力。每组提供由同一批已保存状态导出的 MP4、GIF、PNG、CSV 与 MAT。播放只重复已算出的离散状态，没有添加插值估计帧；MP4/GIF 都是仿真结果的可视化，不是硬件录像。

本次已完成的 4 组无摩擦场景，接触力向量 RMSE 为 **5.598e-07–7.171e-06 N**。这支持当前算法在这些已标定、无注入噪声的单接触场景中取得较小误差。 含噪声滑动的最后一帧真实／估计接触力大小为 **6.2409 / 7.7667 N**，向量相对误差 **25.2%**；该场景的分力精度仍不足。其数值检查仍全部通过，直接说明当前数值质量标志不能筛出所有力估计错误。

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
| 弯钩杆向上顶面 | 3 | 5.598e-07 | 5.108e-07 | 6.622e-08 | 0/3 帧需复核 |
| 侧向墙面接触 | 3 | 1.614e-06 | 9.744e-07 | 8.152e-07 | 0/3 帧需复核 |
| 倾斜环境接触 | 3 | 7.171e-06 | 4.002e-06 | 3.950e-06 | 0/3 帧需复核 |
| 较长较软弯杆 | 3 | 5.806e-07 | 3.258e-07 | 2.918e-07 | 0/3 帧需复核 |
| 沿顶面滑动 | 3 | 1.018e-05 | 6.930e-06 | 4.151e-06 | 0/3 帧需复核 |
| 相同滑动＋测量噪声 | 3 | 1.0280 | 0.5968 | 0.4987 | 3/3 帧需复核 |

### 各场景最后一帧的接触力大小

| 场景 | 推进 mm | 真实大小 N | 估计大小 N | 向量误差 N |
|---|---:|---:|---:|---:|
| 弯钩杆向上顶面 | 20 | 7.8769 | 7.8769 | 8.219e-07 |
| 侧向墙面接触 | 21 | 10.2593 | 10.2593 | 7.886e-07 |
| 倾斜环境接触 | 21 | 10.7532 | 10.7532 | 4.455e-06 |
| 较长较软弯杆 | 26 | 4.3319 | 4.3319 | 8.920e-07 |
| 沿顶面滑动 | 22 | 6.2409 | 6.2409 | 1.948e-06 |
| 相同滑动＋测量噪声 | 22 | 6.2409 | 7.7667 | 1.5704 |

同一滑动真值下，无噪声／含噪的接触力向量 RMSE 为 **1.018e-05 / 1.0280 N**，末端力为 **6.930e-06 / 0.5968 N**。这只是一组固定随机种子的配对比较，不是噪声鲁棒性的统计结论。

### 形状、接触位置、穿透与耗时

| 场景 | 真实形状 RMSE mm | 接触弧长 RMSE mm | 真值最大穿透 mm | 估计形状相对真实平面最大穿透 mm | 单帧耗时 s |
|---|---:|---:|---:|---:|---:|
| 弯钩杆向上顶面 | 3.341e-07 | 3.252e-06 | 8.53e-14 | 0.0000 | 9.7–20.5 |
| 侧向墙面接触 | 3.143e-07 | 3.055e-06 | 4.83e-13 | 0.0000 | 15.7–25.6 |
| 倾斜环境接触 | 2.494e-07 | 1.788e-06 | 0.00e+00 | 0.0000 | 16.4–22.9 |
| 较长较软弯杆 | 4.038e-07 | 3.730e-06 | 0.00e+00 | 0.0000 | 5.1–26.1 |
| 沿顶面滑动 | 3.374e-07 | 3.747e-06 | 0.00e+00 | 0.0000 | 81.6–193.9 |
| 相同滑动＋测量噪声 | 0.2284 | 0.0534 | 0.00e+00 | 0.1323 | 100.9–325.2 |

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

- **弯钩杆向上顶面**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/ceiling_hook/results.mat)。
- **侧向墙面接触**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/side_wall/results.mat)。
- **倾斜环境接触**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/inclined_plane/results.mat)。
- **较长较软弯杆**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/long_soft_rod/results.mat)。
- **沿顶面滑动**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_clean/results.mat)。
- **相同滑动＋测量噪声**：[MP4](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/demo.mp4) · [GIF](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/demo.gif) · [末帧图](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/preview.png) · [逐帧力 CSV](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/forces.csv) · [几何与力 JSON](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/data.json) · [完整结果 MAT](../out/demos/a3244666-2bd8-4d80-ae95-5f6ac5e00d42/sliding_noisy/results.mat)。

## 接下来要补的算法能力

1. 在保持完整接触互补的前提下，把当前／前驱两帧的潜在真实形状和 FBG 似然放进同一个窗口。当前完整 MPCC 仍把有噪声的历史重建固定下来，优化变量无法正确吸收这部分误差。
2. 在这六组可复现场景上增加噪声种子、刚度／固有曲率失配和环境偏差；区分大小、方向、两种分力、接触位置和失败率，不能只汇总合力。
3. 扩展独立非共面三维接触、粘滑转换真值及整杆非穿透约束。当前所有真值是平面静力学，不能拿旋转场景代替三维载荷验证。
4. 依照[算法报告](algorithm_formulation_2026-09-19.md)对齐公开论文的观测信息和适用载荷，再做公平基线。现有 demo 没有运行文献方法，不能从这些数字声称优于它们，也不足以宣布 RA-L 投稿就绪。
