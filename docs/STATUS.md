# 当前状态

更新：2026-09-25。

本仓库当前是 EnFiRCE 的 MATLAB 仿真研究实现，目标是从稀疏形状和环境几何中估计连续体机器人杆身接触力与独立末端载荷。完整算法、代码结构、实验协议和限制见[技术总说明](TECHNICAL_OVERVIEW.md)。

本页的六场景结果来自当前代码重新运行的 `5140876c-8e87-4da6-ac4d-c28113e12b88`。运行记录保存了参与计算的源码 SHA-256；每个场景完成 12 个 MATLAB 真实求解状态，视频是由这些状态生成的连续播放可视化，不是硬件录像。

## 已完成

- 三维 Cosserat 平衡、平面接触、离散 Coulomb 摩擦锥和互补 MAP 路径。
- 当前 FBG、环境平面和历史观测的似然与质量标志。
- 六个连续体接触场景及独立连续平衡真值。
- 3 seed × 3 noise 统计协议，共 9 cases、27 frames。
- 双接触、非平面和摩擦失配压力测试。
- 多接触公共 mechanics 层：K 接触状态布局、力解码、Cosserat 候选评估和统一物理审计；当前不冒充双接触逆解。
- 相同观测输入的 shape-only 和 Gaussian 基线。
- 短窗口 Cosserat MAP 入口和墙钟性能测量。
- 31/31 项工程和重放检查通过。

## 关键结果

| 项目 | 当前结果 | 解释 |
|---|---:|---|
| 多种子/多噪声平均总力 RMSE | 0.3076 N | 单个 `ceiling_hook` 轨迹，局部统计 |
| 条件局部区间命中 | 27/27 | 未校准的局部一阶诊断 |
| 双接触压力测试总力 RMSE | 1.584 N | 当前单接触逆解的模型外结果 |
| 非平面压力测试总力 RMSE | 1.649 N | 第二局部平面未提供给逆解 |
| 实时回放 p95 | 119.14 s/frame | 未达到 20 ms |
| 当前五个近乎无噪声接触力 RMSE | `3.870e-7–5.212e-6 N` | 当前运行的五个模型一致场景 |
| 当前无噪声滑动接触力 RMSE | `5.212e-6 N` | 指定摩擦滑动分支 |
| 当前含噪声滑动接触力 RMSE | `0.8793 N` | 12/12 帧触发 review，仍未解决的分力场景 |

## 目前需要确认或继续完成

1. 真实 FBG、相机/深度相机和接触力传感器标定。
2. W=2/3 短窗口统计、接触模式边缘化和联合后验校准。
3. 在已完成的多接触 mechanics 层之上，实现带多平面观测的正式 MAP/MPCC 逆解。
4. 多轨迹、多载荷和独立实验集上的全局覆盖率。
5. MPCC/Cosserat 求解加速和实时预算验证。

## 常用入口

```matlab
addpath(genpath(pwd));
force('demos');
force('formulation');
force('depth');
force('statistics');
force('model-mismatch');
force('fair-baselines');
force('realtime');
force('temporal-window');
force('check');
```

详细结果摘要写入 `out/*/comparison.json`。精选视频和演示动图已纳入版本库；完整 MAT 与逐帧日志仍留在本地。
