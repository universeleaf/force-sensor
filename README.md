# EnFiRCE: Environment- and Friction-informed Rod Contact Estimation

本项目研究连续体机器人与环境接触时的力分解问题：利用稀疏形状观测和环境几何，同时估计杆身接触力与独立末端载荷，并在力分量不可可靠分离时输出质量标志。

当前实现是 MATLAB 仿真研究代码，核心求解器与项目 Formulation 对齐，包括三维 Cosserat 平衡、平面接触、离散 Coulomb 摩擦锥、互补约束和 MAP 估计。代码不会把数值可行性自动当成力精度保证。

## 当前结果

| 协议 | 结果 |
|---|---|
| 基础工程检查 | 本轮 31/31 通过；另有 2 项可选归档重放 |
| 3 个随机种子 × 3 个曲率噪声等级 | 9 cases、27 frames 完成；平均总力 RMSE 0.3076 N |
| 旧双接触 / 非平面 / 摩擦失配压力测试 | 多接触前向符号和标定分段已修正；旧数字撤出有效结果 |
| 固定环境多接触 | 双接触 9.668e-11–7.598e-11 N；三接触 1.267e-10 N；带噪声三接触 0.6202 N |
| 多接触配对 benchmark | 396 次估计；24 个曲率点、标称噪声下接触力 RMSE：环境方法 0.490 N，shape-only 2.967 N；8 个点时环境方法 13.576 N |
| 平面标定误差 | 第一个平面错位 1 mm 时接触力 RMSE 31.092 N；允许 1 mm 标准差的潜在平面偏移后为 1.341 N |
| 相同观测输入基线 | EnFiRCE 接触力 RMSE 0.0190 N；shape-only 1.069 N；Gaussian 1.689 N |
| 墙钟性能 | p95 119.14 s/frame，尚未达到 20 ms |

多种子和噪声区间目前是条件局部一阶诊断，不是校准后的全局置信区间。历史多接触 benchmark 使用已知数量/顺序的平面原型；新增完整三维窗口路径从形状和环境生成接触候选，联合求解各帧的 Cosserat 平衡、摩擦、接触力和末端力，见[完整 formulation 工作流](docs/FORMULATION_WORKFLOW.md)。真实传感器尚未接入。

## 公开视频与演示

- [六个接触场景离线播放器](out/demos/index.html)：包含顶面弯钩、侧墙、斜面和滑动场景；每个场景现在用 12 个完整求解状态生成 MATLAB `forces.mp4`。
- 视频文件按场景放在 [`out/demos/latest/`](out/demos/latest/) 下；目录名保持稳定，运行 UUID 只保留在结果 JSON 中作为复现实验记录。
- [90° 旋转后向上推的力图视频](out/upward/forces.mp4)：沿 `+z` 推 45 mm，随后沿 `-x` 滑 1 mm。
- [视频几何回放](out/video/forces.mp4)：先沿 `+z` 推 20 mm，再沿 `-x` 滑 12 mm；这是 `fail.mp4` 的图像几何近似，不是原视频参数恢复。
- [视频种子回放](out/stage1/video_seeded/forces.mp4)和[壁面种子回放](out/stage1/wall_tip_seeded/forces.mp4)。
- [固定环境多接触 demos](docs/MULTI_CONTACT_DEMOS.md)：S 形双侧通道、收窄通道双接触、蛇形杆三接触及带曲率噪声的三接触，视频位于 `out/demos/multi_contact/<scene-id>/`。

完整的算法、数据流、文件职责、实验结果和限制见[技术总说明](docs/TECHNICAL_OVERVIEW.md)；滑动阶段、视频说明、文献场景和实验矩阵见[场景矩阵](docs/SCENARIO_MATRIX.md)；旧视频与经典文献问题的对应关系见[文献场景视频对照](docs/LITERATURE_SCENARIO_MAP.md)；当前单接触边界和双接触扩展路径见[多接触扩展说明](docs/MULTI_CONTACT_EXTENSION.md)；当前实验状态见[状态页](docs/STATUS.md)。

多接触的配对基线、传感器密度/噪声消融、几何约束消融和标定误差回放见[软件实验报告](docs/SOFTWARE_BENCHMARK_2026-09-30.md)。这些内部仿真比较还不能证明 SOTA；原始逐次结果与观测包可用下述入口在本地重建。

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
force('realtime');        % 墙钟性能回放
force('temporal-window'); % 短窗口 Cosserat MAP
force('check');           % 工程回归与传感器重放
```

严格路径需要 MATLAB R2024a 或兼容版本、Optimization Toolbox，以及本地的 `LCP-Continuum/` 依赖。完整 MPCC 运行时间较长；各协议会在 `out/` 下写出本地结果，但大体积 MAT 和逐帧日志保留在本地；精选视频与结果摘要随仓库发布。

## 代码结构

- `force.m`：统一入口和场景分派。
- `rod/estimate_sensor_forces.m`：传感器数据包的通用估计入口。
- `rod/estimate_formulation_forces.m`：论文主路径的配置入口。
- `rod/estimate_formulation_window.m`、`integrate_cosserat_load_state.m`：完整三维多接触窗口 MAP、逐时刻非线性平衡和摩擦 MPCC。
- `rod/formulation_window_observations.m`、`formulation_contact_candidates.m`：实际观测通道、协方差与真实前驱时间，形状和环境驱动的候选搜索。
- `rod/run_formulation_workflow.m`、`write_formulation_window_csv.m`：全部给定观测到 MAT/CSV/JSON 的可复现离线流程。
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

当前没有真实 FBG、相机同步、接触力传感器标定或实时硬件结果。完整三维摩擦窗口已连接，历史二维 benchmark 和新窗口实验分别记录；候选覆盖、跨分区接触迁移、模式边缘化、材料误差、全局覆盖率和实时加速仍需优化。这些边界会在结果文件的 `quality` 和报告中显式保留，内部方法比较不作为 SOTA 证明。
