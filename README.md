# EnFiRCE: Environment- and Friction-informed Rod Contact Estimation

本项目研究连续体机器人与环境接触时的力分解问题：利用稀疏形状观测和环境几何，同时估计杆身接触力与独立末端载荷，并在力分量不可可靠分离时输出质量标志。

当前实现是 MATLAB 仿真研究代码，核心求解器与项目 Formulation 对齐，包括三维 Cosserat 平衡、平面接触、离散 Coulomb 摩擦锥、互补约束和 MAP 估计。代码不会把数值可行性自动当成力精度保证。

## 当前结果

最新完整三维实验及逐种子表见[完整软件实验报告](docs/PUBLICATION_RESULTS.md)。下面来自同一个冻结版本；历史二维原型、单种子窗口和连续视频分别保留自己的来源。

| 协议 | 本版本结果 |
|---|---|
| 完整三维因素矩阵 | 108 次：2 场景 × 3 种子 × 3 噪声 × 6 方法；推断异常 0 |
| 同观测文献适配 | 36 次点载荷/Gaussian 基线，18 个字节相同的输入/真值包；推断异常 0 |
| 三接触，标称噪声 | 三种子接触向量 RMSE 均值：EnFiRCE 0.215 N，Point 3.142 N，Gaussian 0.780 N |
| 空间摩擦双接触，标称噪声 | EnFiRCE 0.288 N，Point 2.491 N，Gaussian 0.607 N |
| 局部 95% 区间 | 含噪完整法 252/252 个可计分力分量覆盖；平均全宽 1.551 N；条件诊断，未作全局校准声明 |
| 工程检查与归档重放 | 36/36 通过；包括 JSON 恢复、搜索域、导数和原始传感器重放 |
| 求导对照 | 2 次顺序冷启动；ODE 6817 → 6060，最大力差 8.28e-15 N |

接触力、末端力与总合力分别计分；各方法的非正退出、review 和不利结果在报告中完整列出。三个种子重复传感器噪声，而非独立机器人轨迹。因素计时共享并行资源，不能作为独立速度排名。对比是文献思想适配，未运行官方完整因子图系统，当前不宣称 SOTA。

完整三维路径从形状与环境生成候选，联合优化各帧 Cosserat 平衡、未知接触弧长、力、环境参数和摩擦历史；见[完整 formulation 工作流](docs/FORMULATION_WORKFLOW.md)。它是直接非线性窗口 MAP，递归预测后验/迭代 EKF 的原式仍单独列为差异。真实传感器尚未接入。

历史 396 次二维多接触及 54 次平面偏移实验见[二维软件报告](docs/SOFTWARE_BENCHMARK_2026-09-30.md)；历史七窗口与缓存结果见[文献比较说明](docs/LITERATURE_COMPARISON.md)。这些历史记录未改写成当前源码结果。

## 公开视频与演示

- [六个接触场景离线播放器](out/demos/index.html)：包含顶面弯钩、侧墙、斜面和滑动场景；每个场景现在用 12 个完整求解状态生成 MATLAB `forces.mp4`。
- 视频文件按场景放在 [`out/demos/latest/`](out/demos/latest/) 下；目录名保持稳定，运行 UUID 只保留在结果 JSON 中作为复现实验记录。
- [90° 旋转后向上推的力图视频](out/upward/forces.mp4)：沿 `+z` 推 45 mm，随后沿 `-x` 滑 1 mm。
- [视频几何回放](out/video/forces.mp4)：先沿 `+z` 推 20 mm，再沿 `-x` 滑 12 mm；这是 `fail.mp4` 的图像几何近似，不是原视频参数恢复。
- [视频种子回放](out/stage1/video_seeded/forces.mp4)和[壁面种子回放](out/stage1/wall_tip_seeded/forces.mp4)。
- [固定环境多接触 demos](docs/MULTI_CONTACT_DEMOS.md)：S 形双侧通道、收窄通道双接触、蛇形杆三接触及带曲率噪声的三接触，视频位于 `out/demos/multi_contact/<scene-id>/`。

完整的算法、数据流、文件职责、实验结果和限制见[技术总说明](docs/TECHNICAL_OVERVIEW.md)；滑动阶段、视频说明、文献场景和实验矩阵见[场景矩阵](docs/SCENARIO_MATRIX.md)；旧视频与经典文献问题的对应关系见[文献场景视频对照](docs/LITERATURE_SCENARIO_MAP.md)；当前单接触边界和双接触扩展路径见[多接触扩展说明](docs/MULTI_CONTACT_EXTENSION.md)；当前实验状态见[状态页](docs/STATUS.md)。

多接触的配对基线、传感器密度/噪声消融、几何约束消融和标定误差回放见[软件实验报告](docs/SOFTWARE_BENCHMARK_2026-09-30.md)。这些内部仿真比较还不能证明 SOTA；原始逐次结果与观测包可用下述入口在本地重建。

新增完整窗口的[逐接触真值与估计结果](out/formulation_window/summary.md)、[模型失配报告](out/model_mismatch/summary.md)和[无杆身接触结果](out/formulation_window/free_space/metrics.json)分别记录。完整公式、代码对应和每个场景的具体参数见[完整 formulation 工作流](docs/FORMULATION_WORKFLOW.md)。

历史版本的[文献基线与缓存技术报告](docs/LITERATURE_COMPARISON.md)解释适配公式、公平输入、逐项结果、代码实现和剩余缺口；[真实对比图](out/benchmarks/literature/figures/noisy_baseline_comparison.png)与[绘图源数据](out/benchmarks/literature/source_data.csv)随仓库发布。它们不是原作者官方实现，当前证据不足以宣布 SOTA。

新的[完整三维因素与修复说明](docs/FORMULATION_FACTORS.md)介绍跨帧候选关联、全长有序接触、共享导数、退化分支精化，以及完整窗口的多种子/多噪声、逐因素消融和局部覆盖率接口。各阶段与代码对应也列入技术总说明第 25 节。

## 快速运行

在 MATLAB 中将 Current Folder 设置为仓库根目录：

```matlab
addpath(genpath(pwd));
force('demos');            % 六个独立连续体接触场景
force('formulation');     % 三维非线性 Cosserat + 完整 MPCC
force('depth');           % 合成深度图到平面估计再到力估计
force('statistics');      % 多种子、多噪声统计协议
force('model-mismatch');  % 双接触、非平面、摩擦失配压力测试
force('fair-baselines');  % 相同观测输入的基线比较
force('multi-benchmark'); % 双/三接触，396 次配对基线及消融
force('multi-geometry');  % 重放 1 mm 平面误差及潜在偏移先验
force('multi-formulation'); % 自动候选、三维多接触、完整摩擦时间窗口
force('literature-baselines'); % 同一归档曲率：文献点载荷/Gaussian 适配
run_formulation_cache_benchmark(); % 相同冷启动的精确缓存开关消融
force('formulation-factors'); % 完整三维：多种子/噪声/六因素/局部覆盖率
force('publication');        % 消融 → 字节相同的文献对照 → 性能 → 工程检查
run_formulation_derivative_benchmark(); % 相同冷启动，共享/单独求导对照
force('realtime');        % 墙钟性能回放
force('temporal-window'); % 短窗口 Cosserat MAP
force('check');           % 工程回归与传感器重放
```

严格路径需要 MATLAB R2024a 或兼容版本、Optimization Toolbox，以及本地的 `LCP-Continuum/` 依赖。完整 MPCC 运行时间较长；各协议会在 `out/` 下写出本地结果，完整投稿协议的精选输入/真值/估计 MAT、CSV、完成记录与图源随仓库发布；其他中间数据和逐帧日志保留在本地。

首次克隆本项目时，外部依赖需单独获取。本轮环境使用版本 `56bfd089665efc95bba3e0e49ad11557cf206524`，可在仓库根目录运行：

```text
git clone https://github.com/Jia0Shen/LCP-Continuum.git LCP-Continuum
git -C LCP-Continuum checkout 56bfd089665efc95bba3e0e49ad11557cf206524
```

依赖检查确认杆构造器及力学函数来自这个本地目录，避免同名函数遮蔽。新的运行记录同时保存所需依赖文件的 SHA-256；仅有 Git 版本号不能证明依赖未被本地修改。新窗口和失配协议的精选原始输入/真值/估计 MAT 随结果发布；其他大量归档输出仍在本地。

## 代码结构

- `force.m`：统一入口和场景分派。
- `rod/estimate_sensor_forces.m`：传感器数据包的通用估计入口。
- `rod/estimate_formulation_forces.m`：论文主路径的配置入口。
- `rod/estimate_formulation_window.m`、`integrate_cosserat_load_state.m`：完整三维多接触窗口 MAP、逐时刻非线性平衡和摩擦 MPCC。
- `rod/formulation_window_observations.m`、`formulation_contact_candidates.m`：实际观测通道、协方差与真实前驱时间，形状和环境驱动的候选搜索。
- `rod/track_formulation_candidates.m`、`formulation_contact_arc_domain.m`、`formulation_derivative_bundle.m`：跨帧候选关联、全长有序接触域与共享导数。
- `rod/run_formulation_factor_protocol.m`、`resample_formulation_observations.m`、`score_formulation_coverage.m`：完整三维窗口的独立噪声实现、因素消融和可辨识分量覆盖率；`scripts/render_formulation_factors.py`：校验原始数据后绘图。
- `rod/run_formulation_workflow.m`、`write_formulation_window_csv.m`：全部给定观测到 MAT/CSV/JSON 的可复现离线流程。
- `rod/estimate_literature_curvature_baseline.m`、`integrate_body_load_curvature.m`：完整局部坐标 Cosserat 点载荷/Gaussian 曲率基线；`run_literature_baseline_protocol.m` 与 `scripts/render_literature_comparison.py` 生成同输入记录和矢量图。
- `rod/solve_cosserat_force_map.m`、`rod/solve_contact_mpcc.m`：单接触三维平衡和互补优化。
- `rod/solve_cosserat_multi_contact_map.m`：独立多接触正向压力真值。
- `rod/solve_planar_multi_contact.m`、`integrate_planar_multi_contact.m`、`audit_planar_multi_contact.m`：固定平面环境中的多接触连续 Cosserat 求解和整杆非穿透审计。
- `rod/estimate_planar_multi_contact.m`：带可选平面偏移先验的稀疏形状多接触逆解；`estimate_planar_shape_only_point_loads.m`：只用形状的同输入点力基线。
- `rod/run_multi_contact_benchmark.m`、`run_multi_contact_plane_uncertainty.m`：多接触配对实验、消融与标定误差回放；`scripts/summarize_multi_contact_benchmark.py`：生成可核对的结果表。
- `rod/run_multi_contact_demo_suite.m`、`render_multi_contact_demo_video.m`：按可读场景名生成多接触结果和 MP4。
- `rod/multi_contact_state_spec.m`、`decode_multi_contact_state.m`、`evaluate_multi_contact_state.m`：早期三维多接触候选评估层；完整窗口使用 `formulation_window_spec.m` 和 `estimate_formulation_window.m`。
- `rod/estimate_temporal_window_forces.m`：前一时刻平衡和过程先验的短窗口 MAP。
- `rod/run_submission_statistics.m`、`run_model_mismatch_protocol.m`、`run_fair_baseline_protocol.m`、`run_realtime_benchmark.m`：投稿评估协议。
- `rod/test_*.m`、`rod/validate_*.m`：输入契约、物理约束、摩擦和结果完整性检查。
- `docs/`：公开技术说明、实验报告和验证记录。
- `legacy/`：历史的 Rucker、Ferguson 和早期 Cosserat 示例。

## 研究边界

当前没有真实 FBG、相机同步、接触力传感器标定或实时硬件结果。完整三维摩擦窗口已连接；接触位置现在可沿整根杆迁移，只施加顺序与最小间距约束，旧首帧分区保留为显式消融。历史二维 benchmark 和新三维实验分别记录。有限候选覆盖、模式边缘化、材料误差、全局覆盖率和实时性能属于后续研究验证范围，内部方法比较不作为 SOTA 证明。
