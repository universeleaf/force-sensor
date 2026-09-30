# 固定环境多接触 demo

这组 demo 用来验证 EnFiRCE 的“环境几何 + 稀疏形状”力分解在多接触情况下的可运行边界。每个状态都由连续的平面 Cosserat 杆模型生成，然后用独立的稀疏曲率逆解估计各接触反力、接触弧长和末端外力。视频只是把已经求解并保存的 12 个状态连续播放，不会从播放帧重新估计力。

## 运行范围

当前实现是一个**平面、无摩擦、接触数量和接触顺序已知**的多接触原型：

- 环境由若干条固定直线表示，直线法向和位置是输入；
- 杆不可伸长、不可剪切，刚度和本征曲率由模型给定；
- 每个接触只有非负法向力，接触点满足闭合和切触；
- FBG 输入是 24 个弧长位置的稀疏平面曲率，带噪声场景使用已声明的曲率标准差；
- 逆解变量是基座弯矩、每个接触的法向力、每个接触的弧长以及末端二维力；
- 求解后统一审计整根杆的非穿透、接触间隙、接触顺序、自由端力矩和数值秩。

这条路径已经读取形状观测并独立估计多接触力，但还没有把它并入三维摩擦锥 MPCC，也没有做未知接触模式的边缘化。因此它可以作为多接触仿真结果和算法原型，不能替代真实 FBG/相机实验或完整三维摩擦 MAP。

## 视频和结果

结果目录使用可读的场景名，不使用 UUID：[`out/demos/multi_contact/`](../out/demos/multi_contact/)。每个场景包含 `forces.mp4`、`forces.png`、`forces.csv`、`data.json`、`input.mat` 和 `results.mat`。CSV/JSON 记录真值与估计值，MAT 文件保留完整曲线、审计和求解器信息。

| 场景 | 接触数 | 说明 | MP4 |
|---|---:|---|---|
| `s_channel_two_contact` | 2 | S 形杆同时接触左右侧壁 | [forces.mp4](../out/demos/multi_contact/s_channel_two_contact/forces.mp4) |
| `tapered_channel_two_contact` | 2 | 两侧法向略有变化的收窄通道 | [forces.mp4](../out/demos/multi_contact/tapered_channel_two_contact/forces.mp4) |
| `serpentine_three_contact` | 3 | 蛇形杆依次接触左、右、左三个约束面 | [forces.mp4](../out/demos/multi_contact/serpentine_three_contact/forces.mp4) |
| `serpentine_three_contact_noisy` | 3 | 与上一场景相同，加入 `2.5e-5 / mm` 曲率噪声 | [forces.mp4](../out/demos/multi_contact/serpentine_three_contact_noisy/forces.mp4) |

每段视频左侧显示真值实线和估计虚线、接触面、各接触力箭头及数值；右侧显示每个接触力的时间曲线、合力和末端力。噪声场景中若某帧残差或审计超出阈值，会在 `forces.csv` 的 `requires_review` 和 `data.json` 中保留，而不是静默删除。

## 本次运行的结果

下面的数值来自 `out/demos/multi_contact/comparison.json`，每个场景 12 个状态。前三个场景是模型一致的无噪声回放；最后一个场景用于展示噪声下的质量标志。

| 场景 | 接触力 RMSE (N) | 接触弧长 RMSE (mm) | 末端力 RMSE (N) | 合力 RMSE (N) | 形状 RMSE (mm) | review |
|---|---:|---:|---:|---:|---:|---:|
| `s_channel_two_contact` | 9.668e-11 | 1.866e-11 | 1.501e-10 | 1.546e-10 | 6.383e-13 | 0/12 |
| `tapered_channel_two_contact` | 7.598e-11 | 5.640e-11 | 1.383e-10 | 1.514e-10 | 1.344e-12 | 0/12 |
| `serpentine_three_contact` | 1.267e-10 | 1.414e-11 | 9.800e-11 | 1.179e-10 | 6.361e-12 | 0/12 |
| `serpentine_three_contact_noisy` | 0.510 | 0.00713 | 0.670 | 0.620 | 0.00383 | 1/12 |

真值的最大接触闭合/整杆穿透分别为 `6.995e-10`、`1.514e-9`、`1.517e-8` 和 `1.517e-8 mm`；估计结果前三个场景的最大穿透与真值同量级，噪声场景的最大估计穿透为 `1.014e-3 mm`，因此该场景明确标记一帧需要复核。数值精度表示当前合成模型中的一致性，不能外推成硬件精度。

## 如何重跑

在仓库根目录运行：

```matlab
addpath('rod');
report = run_multi_contact_demo_suite();
assert(report.successCount == report.caseCount);
```

入口会把旧的同名结果移入 `out/demos/multi_contact/history/<timestamp>/`，再生成稳定的场景目录。历史目录用于追溯，不应作为论文结果目录。若只想检查某个场景，可直接读取相应目录的 `results.mat`，其中 `truth`、`estimate`、`scene` 和 `sensorInput` 都已保存。

## 对应代码

- [`multi_contact_demo_scenes.m`](../rod/multi_contact_demo_scenes.m)：四个固定环境、接触顺序、推进轨迹和噪声种子；
- [`build_multi_contact_demo_truth.m`](../rod/build_multi_contact_demo_truth.m)：独立前向求解并生成稀疏曲率输入，真值不从逆解回流；
- [`solve_planar_multi_contact.m`](../rod/solve_planar_multi_contact.m)：以能量解作初值，再用连续 Cosserat 射击满足力矩、闭合和切触；
- [`integrate_planar_multi_contact.m`](../rod/integrate_planar_multi_contact.m)：在接触弧长处施加法向力跳变并积分整根杆；
- [`estimate_planar_multi_contact.m`](../rod/estimate_planar_multi_contact.m)：已知接触数量/顺序下的稀疏曲率逆解；
- [`audit_planar_multi_contact.m`](../rod/audit_planar_multi_contact.m)：统一检查整杆非穿透、闭合、切触、顺序、非负力和端部力矩；
- [`run_multi_contact_demo_suite.m`](../rod/run_multi_contact_demo_suite.m)：批量运行、评分、写入 CSV/JSON/MAT 并调用视频渲染；
- [`render_multi_contact_demo_video.m`](../rod/render_multi_contact_demo_video.m)：用 MATLAB `VideoWriter` 生成 MP4 和诊断图。

旧的 [`build_multi_contact_truth.m`](../rod/build_multi_contact_truth.m) 和 [`run_model_mismatch_protocol.m`](../rod/run_model_mismatch_protocol.m) 仍保留，用来测试单接触估计器遇到多接触或摩擦失配时的模型边界；它们不属于本页的成功 demo。
