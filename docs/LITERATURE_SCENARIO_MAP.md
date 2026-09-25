# 文献场景与视频对应关系

这里把仓库中的完整视频和常见连续体机器人力感知文献逐一对应。**“适配”表示复用了文献的载荷或观测设置，但没有声称复现论文的硬件、材料或算法。** 所有新 `out/demos/*/forces.mp4` 都是 EnFiRCE 的三维 Cosserat + 接触 MPCC/MAP 结果；论文方法的公平基线仍由单独的 baseline 协议完成。

## 已有的旧格式完整视频

这些视频直接由 `run_rod_plane_force_sensing_experiment` 的 MATLAB `VideoWriter` 生成，包含连续推进状态、真实/估计杆形、环境平面、力箭头和右侧力历史：

| 文献问题 | 本仓库视频 | 代码入口 | 适配范围 |
|---|---|---|---|
| Rucker & Webster (IROS 2011) 的形状/位姿到点载荷 | [`out/stage1/wall_tip_seeded/forces.mp4`](../out/stage1/wall_tip_seeded/forces.mp4) | `force('sensor-tip')` / `run_sensor_stage1` | 同时含杆身接触和末端点载荷；估计器是 EnFiRCE MAP，不是原论文 EKF。 |
| Aloi et al. (RA-L 2022) 的点载荷/高斯载荷比较 | [`out/wall/aloi.mp4`](../out/wall/aloi.mp4) | `force('wall')` | 右侧同时显示真实合力与 Aloi 高斯基线；当前场景是平面接触适配。 |
| 环境几何与单接触力分解 | [`out/wall/forces.mp4`](../out/wall/forces.mp4) | `force('wall')` | EnFiRCE 主路径的名义 wall 场景。 |
| 学长视频几何和旋转等价性 | [`out/video/forces.mp4`](../out/video/forces.mp4)、[`out/upward/forces.mp4`](../out/upward/forces.mp4) | `force('video')`、`force('upward')` | 图像几何近似和 90° 旋转；不是视频的精确标定恢复。 |

## 新 demo 的完整求解链路

`force('demos')` 对六个环境场景各生成 12 个真实推进状态。每个状态依次经过：

1. `build_contact_demo_truth` 的独立二维连续射击平衡，检查整杆非穿透、相切和自由端力矩；
2. `simulate_sensor_packet` 只导出稀疏曲率、基座姿态、平面观测和前驱样本；
3. `estimate_sensor_forces` 的三维 Cosserat shooting、Coulomb 摩擦锥和 Scholtes MPCC/MAP；
4. 事后评分和 `render_contact_demo_video` 的 MATLAB `VideoWriter` 导出。

因此 `forces.mp4` 不是把三帧复制成动画：它由 12 个真实求解状态采样成 60 个播放帧。`demo.mp4` 只是同一文件的兼容副本，Python 脚本只生成 HTML、GIF 和 PNG。

六个场景属于文献问题的参数化适配：

| 场景 | 可对照的文献问题 | 尚未声称的内容 |
|---|---|---|
| `ceiling_hook` | 视频几何下的单接触、独立末端载荷 | 不是某篇论文的材料和硬件复现。 |
| `side_wall` | 单接触姿态旋转和点载荷分解 | 90° 旋转仍是平面内力学，不是完整空间扭转验证。 |
| `inclined_plane` | 环境法向变化、稀疏形状 + 环境观测 | 只有局部斜平面，尚未覆盖非平面曲面。 |
| `long_soft_rod` | 形状/刚度变化下的外力估计 | 刚度是设定失配，不是材料标定实验。 |
| `sliding_clean` | 摩擦接触和切向力方向 | 滑移分支是预设的准静态配对，不是动力学粘滑转变。 |
| `sliding_noisy` | 噪声下的摩擦方向可观测性 | 当前结果用于暴露分力退化，不能写成已解决的噪声鲁棒性。 |

## 论文引用

- Rucker & Webster, “Deflection-based force sensing for continuum robots: A probabilistic approach,” IROS 2011, [DOI](https://doi.org/10.1109/IROS.2011.6094526)。
- Aloi et al., “Estimating Forces Along Continuum Robots,” IEEE RA-L 2022, [DOI](https://doi.org/10.1109/LRA.2022.3188905)。
- Ferguson, Rucker & Webster, “Unified Shape and External Load State Estimation for Continuum Robots,” IEEE T-RO 2024, [DOI](https://doi.org/10.1109/TRO.2024.3360950)。
- Xiao & Chen, “Efficient Force Estimation for Continuum Robot,” [arXiv:2109.12469](https://arxiv.org/abs/2109.12469)。

要把这些适配升级为论文级复现，还需要把相同观测输入、载荷参数、材料、传感器布局和评价指标写成可公开的基线协议；不能只凭视频外观或单一 wall 场景声称优于文献方法。
