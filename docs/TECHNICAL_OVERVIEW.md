# EnFiRCE 技术总说明

本文档是仓库中面向研究协作者的总技术说明。它描述项目要解决的问题、当前的数学模型、代码数据流、可复现实验和已知边界。结果以代码和 `out/*/comparison.json` 为准。

场景选择、滑动阶段的物理原因、公开视频索引和相关工作对照见[场景矩阵](SCENARIO_MATRIX.md)。

## 1. 项目目标

连续体机器人在环境中运动时，稀疏形状观测通常只能看到杆的形状，不能直接区分杆身接触反力和末端任务载荷。本项目利用两路稀疏弯曲观测、杆的本征几何、基座状态和环境平面信息，估计

- 杆身接触位置与接触力；
- 独立末端力；
- 接触模式、摩擦方向和数值可辨识性质量标志。

当前实现是仿真研究代码。真实 FBG、相机和接触力传感器尚未接入，因此所有数值结果都应理解为算法和数值流程验证。

## 2. 数学模型

状态由环境、接触和末端载荷组成，主要变量为环境平面点与法向、接触弧长、接触力、摩擦系数方向变量、法向乘子以及末端力。杆的中心线采用三维 Cosserat 表示：

\[
\mathbf r'(s)=\mathbf R(s)\mathbf e_3,\qquad
\mathbf n'(s)+\mathbf f(s)=0,\qquad
\mathbf m'(s)+\mathbf r'(s)\times\mathbf n(s)+\boldsymbol\ell(s)=0.
\]

接触约束包括平面间隙、杆身非穿透、法向互补、切向摩擦锥和接触位移条件。优化目标由以下项组成：

1. 当前 FBG 弯曲观测似然；
2. 环境平面点和法向观测似然；
3. 杆参数、接触和末端力的先验；
4. 当前 Cosserat 平衡残差；
5. 摩擦和互补约束的连续化惩罚。

`estimate_sensor_forces` 是通用入口，`estimate_formulation_forces` 使用论文主路径的配置。`estimate_temporal_window_forces` 在多个时刻上联合优化状态，并增加过程先验与前一时刻接触点运动学。

## 3. 数据流

```text
sensor packet
    ├── sparse bending observations
    ├── base pose and insertion
    ├── plane/depth observation and covariance
    └── rod/material configuration
            ↓
measurements_from_sensor_packet
            ↓
shape seed + Cosserat balance map
            ↓
contact MPCC / MAP estimator
            ↓
force estimate + physics residuals + quality flags
            ↓
JSON/MAT/CSV result record
```

输入数据包不应包含真实力、truth 或 forward 字段。真值只在独立模拟器和评估脚本中用于评分，不能进入估计器。

## 4. 代码结构

| 路径 | 作用 |
|---|---|
| `force.m` | 根目录统一入口和场景分派 |
| `rod/estimate_sensor_forces.m` | 通用传感器估计入口 |
| `rod/estimate_formulation_forces.m` | Formulation 对齐配置入口 |
| `rod/solve_cosserat_force_map.m` | 单接触三维 Cosserat 射击映射 |
| `rod/solve_contact_mpcc.m` | 平面、摩擦锥和互补约束的非线性 MAP |
| `rod/solve_cosserat_multi_contact_map.m` | 独立多接触正向平衡真值 |
| `rod/estimate_temporal_window_forces.m` | 短窗口物理 MAP |
| `rod/build_contact_demo_truth.m` | 单接触连续体真值与传感器包 |
| `rod/build_multi_contact_truth.m` | 双接触、曲面和摩擦失配真值 |
| `rod/run_submission_statistics.m` | 多种子、多噪声统计协议 |
| `rod/run_model_mismatch_protocol.m` | 模型外压力测试 |
| `rod/run_fair_baseline_protocol.m` | 同输入公平基线 |
| `rod/run_realtime_benchmark.m` | 墙钟性能测量 |
| `rod/test_*.m`、`rod/validate_*.m` | 输入、物理和结果回归 |
| `docs/` | 公开技术报告和结果说明 |
| `legacy/` | 历史实现和文献示例 |

## 5. 当前验证结果

### 视频场景的运动定义

仓库中有两个不同的 `forces.mp4`，文件名相同但场景不同：

| 文件 | 场景 | 运动定义 |
|---|---|---|
| `out/upward/forces.mp4` | 将原 wall 场景刚体旋转 90° 后向上推 | `+z` 推 45 mm，随后沿 `-x` 滑 1 mm |
| `out/video/forces.mp4` | 根据 `fail.mp4` 外观构造的近似顶面弯钩场景 | `+z` 推 20 mm，随后沿 `-x` 滑 12 mm |

因此 `out/video/forces.mp4` 后半段向左移动是预先设定的顶面滑动阶段，不是相机漂移。该场景的几何和材料参数是图像驱动的近似值，不能当作对原视频参数的精确恢复。具体阶段、累计路径和接触模式见对应目录的 `trajectory.csv`；`phase=push` 后切换到 `phase=slide` 的帧就是运动方向改变的位置。

### 工程回归

最新 `force('check')` 共 31 项，全部通过。检查覆盖输入契约、摩擦方向、平面几何、Cosserat 工作量、结果完整性、旋转一致性、深度环境和两组传感器重放。

### 多种子和噪声

`ceiling_hook` 使用 3 个随机种子和 3 个曲率噪声等级（0、`5e-5`、`1e-4` /mm），共 9 个 case、27 帧：

- 平均总力 RMSE：`0.3076 N`；
- 平均接触力 RMSE：`0.5198 N`；
- 平均末端力 RMSE：`0.2661 N`；
- 条件局部一阶区间命中：`27/27`。

区间定义为 `|error| <= 1.96*sigma_local`，结果保留 `coverageCertified=false`。它不是完整后验、模式混合边缘化或校准后的全局置信区间。

### 模型失配

| 场景 | 总力 RMSE | 接触力 RMSE | 说明 |
|---|---:|---:|---|
| two-contact | 1.584 N | 0.851 N | 真值有两个接触，当前逆解仍为单接触 |
| curved-surface | 1.649 N | 0.838 N | 两局部平面夹角 24.58°，第二平面未提供给逆解 |
| friction-mismatch | 1.630 N | 0.853 N | 真值摩擦设置与估计模型不一致 |

三类压力测试均触发 `review`。它们用于确定方法边界，不是当前多接触逆解已经成功的证据。

### 公平基线

所有方法读取同一稀疏 FBG、基座和历史数据包；shape-only 和 Gaussian 基线不读取环境或真值。

| 方法 | 总力 RMSE | 接触力 RMSE |
|---|---:|---:|
| EnFiRCE | 0.0456 N | 0.0190 N |
| shape-only point-load | 0.0271 N | 1.069 N |
| Aloi-style Gaussian | 0.6025 N | 1.689 N |

合力误差不能代替分力误差。shape-only 的合力较小主要来自分量补偿，而不是正确分离两类力。

### 计算性能

`sliding_clean` 两帧实际 MATLAB 回放耗时 119.23 s 和 117.47 s，p95 为 119.14 s/frame，有效频率 0.00839 Hz，未达到 20 ms 标称周期。当前实现没有实时能力声明。

## 6. 已修复的工程问题

- Cosserat 输入现在显式拒绝非有限弧长、曲率、刚度、接触位置和力向量。
- 非法接触弧长不再被静默裁剪到杆端。
- 杆身非穿透约束贯穿形状种子、MPCC 和最终验收。
- ODE 工作量有明确上限，超限试探会被拒绝，不返回截断杆形。
- FBG 似然使用传感器标准差和模型误差底噪的合成值。
- 多接触真值使用独立 Cosserat 平衡，不复用逆解结果。
- 结果目录写入运行状态、源文件校验和、退出码、约束残差和质量标志。

## 7. 尚未完成的工作

1. 接入真实 FBG、相机/深度相机、同步和接触力传感器标定。
2. 将短窗口 MAP 从 W=1 smoke test 扩展到 W=2/3，并验证联合后验。
3. 完成接触模式混合边缘化和校准后的全局区间覆盖率。
4. 将双接触和非平面几何真正纳入逆解，而不只是压力测试。
5. 使用 profiler、warm-start、稀疏导数和降阶模型解决当前两分钟级单帧耗时。

## 8. 复现命令

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

完整 MAT 和逐帧日志默认保存在本地 `out/`；六个演示动图、关键视频和小型结果摘要随仓库发布。公开结果摘要位于 `out/*/comparison.json`；完整本地交接文档不在版本库中。
