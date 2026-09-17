# 当前 MATLAB 代码

从仓库根目录运行 `force()` 或 `force('senior')`。用法和结果见 [根目录说明](../README.md)，接手背景见 [MEMORY.md](../MEMORY.md)。

| 文件 | 用途 |
|---|---|
| `run_rod_plane_force_sensing_experiment.m` | 正向 LCP、模拟传感、约束 EKF/MAP、出图及视频 |
| `simu_rod_plane_*_force_sensing.m` | 两个正式场景的固定参数 |
| `contact_tangent.m` | 当前轴向平面附近连续的摩擦基底 |
| `validate_friction.m` | 独立检查方向、圆锥/多面体锥、互补不等式 |
| `validate_rod_plane_displacement_results.m` | 形状、力及物理约束验收 |
| `test_friction_*.m`, `test_reverse_slide.m` | 这次修复的回归检查 |
| `plot_friction.m` | 带符号摩擦、锥边界、接触位移和摩擦功 |
| `estimate_aloi_gaussian_baseline.m` | 原有非线性 Gaussian 位置拟合对照 |
| `compute_aloi_comparison_metrics.m` | Aloi 误差及载荷假设适用性 |
| `rerun_aloi_saved_result.m` | 仅重算保存数据的 Aloi 对照 |
| `validate_aloi_gaussian_baseline.m` | Aloi 自身模型内验证 |

保留原有公开函数名，缩短目录和输出名，避免破坏 MATLAB 的函数名约定。旧长文档已归档到 `archive/old-notes.zip`。
