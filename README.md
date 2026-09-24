# EnFiRCE: environment- and shape-informed force estimation

本项目研究连续体机器人与环境接触时的力分解问题：利用稀疏形状观测和环境几何，同时估计杆身接触力与独立末端载荷，并在力分量不可可靠分离时输出质量标志。

当前实现是 MATLAB 仿真研究代码，核心求解器与项目 Formulation 对齐，包括三维 Cosserat 平衡、平面接触、离散 Coulomb 摩擦锥、互补约束和 MAP 估计。代码不会把数值可行性自动当成力精度保证。

## 当前结果

| 协议 | 结果 |
|---|---|
| 31 项工程检查 | 31/31 通过 |
| 3 个随机种子 × 3 个曲率噪声等级 | 9 cases、27 frames 完成；平均总力 RMSE 0.3076 N |
| 双接触 / 非平面 / 摩擦失配 | 总力 RMSE 1.584 / 1.649 / 1.630 N；均触发 review |
| 相同观测输入基线 | EnFiRCE 接触力 RMSE 0.0190 N；shape-only 1.069 N；Gaussian 1.689 N |
| 墙钟性能 | p95 119.14 s/frame，尚未达到 20 ms |

多种子和噪声区间目前是条件局部一阶诊断，不是校准后的全局置信区间。当前研究范围仍是仿真、单平面名义模型和有限的多接触压力测试。

## 公开视频与演示

- [六个接触场景离线播放器](out/demos/index.html)：包含顶面弯钩、侧墙、斜面和滑动场景。
- [90° 旋转后向上推的力图视频](out/upward/forces.mp4)。
- [视频几何回放](out/video/forces.mp4)。
- [视频种子回放](out/stage1/video_seeded/forces.mp4)和[壁面种子回放](out/stage1/wall_tip_seeded/forces.mp4)。

完整的算法、数据流、文件职责、实验结果和限制见[技术总说明](docs/TECHNICAL_OVERVIEW.md)，当前实验状态见[状态页](docs/STATUS.md)。

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
force('realtime');        % 墙钟性能回放
force('temporal-window'); % 短窗口 Cosserat MAP
force('check');           % 工程回归与传感器重放
```

严格路径需要 MATLAB R2024a 或兼容版本、Optimization Toolbox，以及本地的 `LCP-Continuum/` 依赖。完整 MPCC 运行时间较长；各协议会在 `out/` 下写出本地结果，但大体积 MAT 和逐帧日志保留在本地；精选视频与结果摘要随仓库发布。

## 代码结构

- `force.m`：统一入口和场景分派。
- `rod/estimate_sensor_forces.m`：传感器数据包的通用估计入口。
- `rod/estimate_formulation_forces.m`：论文主路径的配置入口。
- `rod/solve_cosserat_force_map.m`、`rod/solve_contact_mpcc.m`：单接触三维平衡和互补优化。
- `rod/solve_cosserat_multi_contact_map.m`：独立多接触正向压力真值。
- `rod/estimate_temporal_window_forces.m`：前一时刻平衡和过程先验的短窗口 MAP。
- `rod/run_submission_statistics.m`、`run_model_mismatch_protocol.m`、`run_fair_baseline_protocol.m`、`run_realtime_benchmark.m`：投稿评估协议。
- `rod/test_*.m`、`rod/validate_*.m`：输入契约、物理约束、摩擦和结果完整性检查。
- `docs/`：公开技术说明、实验报告和验证记录。
- `legacy/`：历史的 Rucker、Ferguson 和早期 Cosserat 示例。

## 研究边界

当前没有真实 FBG、相机同步、接触力传感器标定或实时硬件结果。短窗口 MAP 已实现 W=1 smoke test，但 W=2/3 统计、接触模式边缘化、全局不确定性校准、多接触逆解和实时加速仍需继续完成。这些边界会在结果文件的 `quality` 和报告中显式保留。
