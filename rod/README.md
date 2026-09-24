# MATLAB implementation

从仓库根目录运行 `force()` 或 `force('formulation')`。所有入口、输入契约和实验边界见[根目录 README](../README.md)与[技术总说明](../docs/TECHNICAL_OVERVIEW.md)。

## 主要入口

- `estimate_sensor_forces(sensorInput)`：通用传感器数据包入口。
- `estimate_formulation_forces(sensorInput)`：Formulation 对齐的三维非线性路径。
- `solve_cosserat_force_map`：给定杆、接触和末端载荷的 Cosserat 射击映射。
- `solve_contact_mpcc`：带平面接触、摩擦锥和互补约束的 MAP 优化。
- `solve_cosserat_multi_contact_map`：多接触独立正向压力真值。
- `estimate_temporal_window_forces`：包含前一时刻平衡和过程先验的短窗口 MAP。

## 评估协议

- `run_submission_statistics`：随机种子、曲率噪声和条件局部区间诊断。
- `run_model_mismatch_protocol`：双接触、非平面和摩擦失配压力测试。
- `run_fair_baseline_protocol`：相同观测输入下的 shape-only 与 Gaussian 基线。
- `run_realtime_benchmark`：实际 MATLAB 墙钟耗时。
- `run_project_checks`：输入、摩擦、几何、结果完整性和重放回归。

## 目录职责

| 目录/文件 | 职责 |
|---|---|
| `rod/solve_*.m` | Cosserat 平衡、接触和互补求解 |
| `rod/estimate_*.m` | 传感器似然、MAP 估计和时间窗口 |
| `rod/build_*_truth.m` | 独立连续体真值和传感器数据包 |
| `rod/test_*.m` | 单元和回归检查 |
| `rod/run_*.m` | 场景、基线和投稿评估协议 |
| `rod/validate_*.m` | 物理约束和结果验收 |

完整运行需要 MATLAB R2024a 或兼容版本、Optimization Toolbox，以及本地 `LCP-Continuum/` 依赖。各协议的完整 MAT 和详细日志写入本地 `out/`；精选视频与小型结果摘要可以直接查看。
