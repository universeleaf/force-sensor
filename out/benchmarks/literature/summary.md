# 同观测文献适配基线比较

更新：2026-10-01。

这是一组已实际完成的 MATLAB 比较。七个相同输入窗口，各有两个输出状态；两个含噪场景各只有一个噪声种子。不能据此宣布 SOTA。

完整曲率输入、校准、独立真值和文件 SHA-256 与原窗口实验一致。基线只共享 EnFiRCE 从观测生成的候选数；不读取真值接触数、位置或载荷。基线位置初始化来自自身曲率重构。这个比较因此以相同候选数为条件，不能评价未知接触数的识别。

| 场景 | 方法 | 接触向量 RMSE / N | 接触大小 MAE / N | 位置 RMSE / mm | 末端 RMSE / N | 合力 RMSE / N | 复核 / 2 |
|---|---|---:|---:|---:|---:|---:|---:|
| S-channel | EnFiRCE | 5.3812e-07 | 4.65084e-07 | 2.49009e-08 | 6.81388e-07 | 6.69622e-07 | 0/2 |
| S-channel | Point curvature LS | 4.04526e-07 | 1.00023e-07 | 1.3145e-08 | 1.29332e-09 | 5.53857e-07 | 0/2 |
| S-channel | Gaussian curvature LS | 0.0241445 | 0.017978 | 0.00236829 | 0.0215525 | 0.0243242 | 1/2 |
| Tapered channel | EnFiRCE | 1.63717e-06 | 1.28634e-06 | 1.19748e-07 | 1.8294e-06 | 1.28458e-06 | 0/2 |
| Tapered channel | Point curvature LS | 3.5056e-07 | 2.59228e-08 | 2.63649e-08 | 4.72097e-10 | 4.75343e-07 | 0/2 |
| Tapered channel | Gaussian curvature LS | 0.090682 | 0.0887875 | 0.00456468 | 0.102533 | 0.103097 | 2/2 |
| Three contacts | EnFiRCE | 9.29541e-07 | 7.26754e-07 | 1.15401e-08 | 1.19449e-06 | 1.10841e-06 | 0/2 |
| Three contacts | Point curvature LS | 2.8625e-07 | 7.02233e-08 | 1.60644e-08 | 2.28677e-10 | 4.50085e-07 | 0/2 |
| Three contacts | Gaussian curvature LS | 0.00244731 | 1.51331e-06 | 8.0013e-08 | 5.4454e-06 | 5.27224e-07 | 0/2 |
| Rotated channel | EnFiRCE | 5.59311e-07 | 4.72679e-07 | 2.69012e-08 | 7.0137e-07 | 6.86519e-07 | 0/2 |
| Rotated channel | Point curvature LS | 4.04165e-07 | 9.49548e-08 | 1.3145e-08 | 1.27273e-09 | 5.53441e-07 | 0/2 |
| Rotated channel | Gaussian curvature LS | 0.0241445 | 0.017978 | 0.00236829 | 0.0215525 | 0.0243242 | 1/2 |
| Noisy three contacts | EnFiRCE | 0.600524 | 0.538018 | 0.0235349 | 0.780736 | 0.819888 | 0/2 |
| Noisy three contacts | Point curvature LS | 1.78064 | 0.791197 | 0.556319 | 1.46932 | 0.631551 | 0/2 |
| Noisy three contacts | Gaussian curvature LS | 1.21216 | 1.00675 | 0.552396 | 1.36077 | 1.50613 | 0/2 |
| Spatial sliding | EnFiRCE | 7.54728e-07 | 4.2755e-07 | 4.42274e-08 | 2.38168e-07 | 1.23301e-06 | 1/2 |
| Spatial sliding | Point curvature LS | 4.22864e-07 | 1.86487e-08 | 2.95539e-08 | 7.09927e-10 | 6.30014e-07 | 0/2 |
| Spatial sliding | Gaussian curvature LS | 0.0449574 | 0.040726 | 0.00287494 | 0.031169 | 0.0647806 | 2/2 |
| Noisy spatial sliding | EnFiRCE | 0.330189 | 0.293602 | 0.0321973 | 0.35183 | 0.479752 | 1/2 |
| Noisy spatial sliding | Point curvature LS | 1.73184 | 1.14942 | 0.223017 | 0.613029 | 1.49934 | 0/2 |
| Noisy spatial sliding | Gaussian curvature LS | 0.993874 | 0.926519 | 0.217092 | 1.20735 | 1.32933 | 0/2 |

点载荷方法借鉴 Xiao–Chen 的曲率最小二乘，使用一般局部坐标 Cosserat 方程而不是只适用直杆的简化式；Gaussian 方法保留 Aloi 的局部横向 Gaussian 载荷参数化，把原位置似然改为实际可用的两通道曲率似然。两者都允许未知三维末端力，以完整非线性平衡预测观测；独立按帧求解。它们是本项目的可核实适配实现，不能写成原作者官方实现或完整原论文复现。

EnFiRCE 的参考是原始完整两时刻 MAP，含共享几何、过程因子和摩擦互补；基线没有环境观测与时间过程因子。因此性能差异是完整方法间比较，不单独归因于某一约束。宽度下界为 0.25 mm；局部最优和拟合复核保留。

无噪声点载荷基线也达到了数值一致，有些指标比 EnFiRCE 更低。含噪两组中 EnFiRCE 的接触向量和位置误差更低，但这只支持这些输入下的局部结论。三接触合力 RMSE 为 EnFiRCE 0.820 N、点载荷 0.632 N：我们的接触分力更准，但合力并非所有条件下最优。历史 EnFiRCE 优化时间和新基线时间并非统一性能测量，不用于速度排名。

本轮运行 ID：`6d9ee3b2-eeea-4747-a943-f374b50b71be`；参考窗口：`2530e6e5-741c-4337-8a23-221aaefb496b`；失败数：0。

[逐项原始指标](comparison.json) · [绘图源数据](source_data.csv) · [已求解接触场景](figures/solved_contact_geometry.png) · [论文图：含噪比较](figures/noisy_baseline_comparison.png) · [论文图：无噪声一致性](figures/clean_baseline_consistency.png) · [二维传感器密度与消融](figures/planar_density_ablation.png)

来源：[Xiao–Chen 2021](https://arxiv.org/abs/2109.12469)；[Aloi et al. 2022](https://doi.org/10.1109/LRA.2022.3188905)。
