# EnFiRCE：三维多接触与完整时间窗口实现

更新：2026-09-30。本文说明本轮新增的完整软件流程，以及其与 `papers/Formulation.pdf` 的关系。它补充[技术总说明](TECHNICAL_OVERVIEW.md)，没有把软件完成度等同于投稿或硬件验证完成度。实际数值以 `out/formulation_window/comparison.json` 为准。

## 1. 这条路径实际解决什么问题

输入是已标定的杆模型、稀疏弯曲测量、基座位姿、环境平面观测与协方差。算法从观测重建的形状中产生接触候选，然后联合优化窗口内各帧的环境参数、身体接触位置、法向力、摩擦力、末端力和基座力矩。每帧均满足当前载荷下的三维 Cosserat 平衡；相邻帧的真实形状共同决定摩擦位移。

二维多接触原型 `estimate_planar_multi_contact` 继续用于历史 benchmark 与独立对照。新的逆解没有读取它的接触数、接触顺序、接触位置或力标签。候选数由形状和环境决定，优化中的法向互补允许某个候选变成零反力。这里的“未知接触”是有上限的候选搜索，并非对任意接触拓扑进行全局枚举。

## 2. 从输入到输出的代码地图

| 步骤 | 实现文件 | 职责 |
|---|---|---|
| 对外入口 | [estimate_formulation_forces.m](../rod/estimate_formulation_forces.m) | schema 2 自动进入新路径；schema 1 传第二个 options 参数也进入新路径 |
| 完整文件工作流 | [run_formulation_workflow.m](../rod/run_formulation_workflow.m) | 保存输入，运行全部给定时刻，保存估计、CSV、质量、源码和数据校验和；保留失败记录 |
| 输入验证和迁移 | [formulation_window_observations.m](../rod/formulation_window_observations.m) | 验证时间、测量维度、位姿和协方差；schema 1 当前/前驱时间合并，同一观测只加入一次 |
| 接触候选 | [formulation_contact_candidates.m](../rod/formulation_contact_candidates.m) | 稀疏形状重建、各平面的间隙局部极小值、跨帧合并、排序、数量限制和搜索审计 |
| 状态布局 | [formulation_window_spec.m](../rod/formulation_window_spec.m) | 每个独立观测平面保存一份几何，多个接触共享它；末端力属于物理状态和过程先验 |
| 法向与摩擦方向 | [formulation_contact_frame.m](../rod/formulation_contact_frame.m) | 球面指数参数化，切向基随法向连续转动，生成多面体摩擦方向 |
| 联合 MAP/MPCC | [estimate_formulation_window.m](../rod/estimate_formulation_window.m) | 初始拟合、原始似然、一次初始先验、过程因子、完整互补、同伦、逐帧质量及局部协方差 |
| 当前三维力学 | [integrate_cosserat_load_state.m](../rod/integrate_cosserat_load_state.m) | 连续接触位置处分段；直接积分当前形状与载荷；返回末端力矩作为硬约束 |
| 任意材料点查询 | [cosserat_state_at_arc.m](../rod/cosserat_state_at_arc.m) | 连续 ODE 插值查询当前/前驱的同一弧长位置 |
| 时间窗口兼容入口 | [estimate_temporal_window_forces.m](../rod/estimate_temporal_window_forces.m) | 替代旧 penalty smoother，筛选连续时间后调用同一完整逆解 |
| 数据子集 | [subset_formulation_packet.m](../rod/subset_formulation_packet.m) | 同步选择曲率、基座、环境、摩擦系数和协方差 |
| 独立二维真值输入 | [build_formulation_multi_packet.m](../rod/build_formulation_multi_packet.m) | 将独立二维平衡嵌入三维坐标；只导出稀疏传感器信息给逆解 |
| 空间滑动真值 | [build_spatial_friction_packet.m](../rod/build_spatial_friction_packet.m) | 独立嵌套射击与接触根求解，生成面外摩擦的三维双接触平衡及平移历史 |
| 独立前向射击 | [solve_cosserat_multi_contact_map.m](../rod/solve_cosserat_multi_contact_map.m) | 不使用逆解的 lifted state；求基座力矩使末端力矩为零 |
| 实验和评分 | [run_formulation_window_protocol.m](../rod/run_formulation_window_protocol.m)、[score_formulation_window.m](../rod/score_formulation_window.m) | 输入、真值、估计分别存储；真值仅进入评分；失败和 review 均保留 |
| 通用力表 | [write_formulation_window_csv.m](../rod/write_formulation_window_csv.m) | 每个时刻/接触一行，报告世界坐标力分量、大小、位置和 review |

## 3. 状态与 formulation 的对应关系

设有 P 个观测平面、K 个接触候选、m 个摩擦生成方向。每帧状态为：

```text
x_k = [所有平面的 p_j(3), eta_j(2);
       所有候选的 s_i, fn_i, beta_i(m), lambda_i;
       fe_k(3); m0_k(3)]
维度 = 5P + (m+3)K + 6
```

`m0` 是计算隐式力学映射而增加的辅助变量，末端力矩等式会约束它；它不属于 formulation 的外力状态，默认不施加随机游走先验。P=2、K=2、m=4 时每帧 30 维；三接触为 37 维。多个接触作用于同一面墙时，只使用该墙的一份观测似然，避免把同一份环境测量重复当成独立证据。

每个接触仍使用 formulation 式 (1)–(3)：

```text
fc_i = n_plane(i) fn_i + D_plane(i) beta_i
beta_i >= 0, fn_i >= 0
```

法向采用相对于首帧观测法向的固定球面参数图。参考方向在窗口内保持不变，因此相邻帧 eta 的差有一致含义。切向基沿指数转动平滑传输，避免优化过程中切换参考轴造成摩擦方向跳变。m=4 是默认的多面体近似；可以配置更多方向。这沿用 formulation 的多面体摩擦设定。

## 4. 三维非线性平衡与符号

单位为 mm、N、N mm，刚度为 N mm²，曲率为 /mm：

```text
p' = R e3
R' = R hat(u)
u = u_hat + K^-1 R^T m
m' = -p' × F_downstream
F_downstream(s) = fe + sum_{s_i > s} fc_i
p(0), R(0) = 已观测基座位姿
m(L) = 0
```

下游外力的世界坐标力矩为 `sum((p_i-p(s)) × f_i)`，对 s 求导得到负号。原多接触射击函数写成正号，本轮已修正。旧单接触函数的符号正确。以前只检查多接触函数自己求得的末端残差，无法发现这一错误；现在还检查独立二维平衡及单接触退化情形。

本轮同时修正了多接触函数的固有曲率读取方式：每个标定区间使用左端样本，并在真实的曲率/刚度突变和接触弧长处分段，避免最近节点把突变位置移到邻近区间。不再在每个恒定参数采样点重启积分，减少了计算量；保留非线性三维弯曲和扭转。

新逆解直接积分待求的 `m0`，在 MPCC 中施加三条 `m(L)=0` 硬等式。它实现与射击法相同的边界问题，减少目标函数评估里的嵌套根求解。FBG 预测值在实际传感器弧长处连续查询，使用实际观测通道。未测扭转仍由力学决定，不加入“扭转为零”的测量似然。

## 5. 完整窗口目标，避免重复计算信息

窗口目标为原始观测似然、一次初始先验和相邻物理状态的随机游走因子之和：

```text
J = 1/2 sum_k ||hu(x_k)-zu_k||^2_Ru^-1
  + 1/2 sum_{k,j} ||[p_j;n_j]-[pMeasured_j;nMeasured_j]||^2_Renv^-1
  + 1/2 ||xPhysical_0-priorMean||^2_P0^-1
  + 1/2 sum_{k>0} ||xPhysical_k-xPhysical_{k-1}||^2_Qk^-1
Qk = Qreference * (t_k-t_{k-1}) / referencePeriod
```

环境协方差支持每个平面的完整 6×6 矩阵；弯曲协方差可以是逐通道标准差，也可以是实际观测向量的完整矩阵。均通过 Cholesky 白化，拒绝不对称或非正定矩阵，不悄悄改成对角矩阵。默认初始几何先验非常宽；相机观测只是初值，不再充当第二份几何似然。默认位置先验是依接触槽数均匀排列的宽 Gaussian，候选间隙极小值只是初值。法向、摩擦、lambda 和末端力的默认初始/过程标准差来自 estimator config，均须正且有限。可通过 `options.initialPrior.mean/covariance` 输入独立的完整初始物理状态先验，或通过 `options.processCovariance` 提供每个参考时间周期的完整物理状态过程协方差，二者均按 `stateSpec.physicalState` 排列，排除辅助基座力矩。完整过程矩阵保留跨几何、位置、力与末端载荷的相关性，不强制对角化。

旧窗口先运行逐帧估计，再把那些后验及同一批原始测量一起放入目标函数，会重复使用观测。新窗口不再这样做。候选搜索仍使用观测来选择有限模型，当前后验以该模型为条件，尚未对候选选择进行概率边缘化。

## 6. 历史材料点与摩擦互补

严格使用 formulation 式 (7)：

```text
v_{i,k} = (I-n n^T) [p_k(s_{i,k}) - p_{k-1}(s_{i,k})]
```

注意两项都取当前待估的同一弧长 `s_{i,k}`。不能用 `p_{k-1}(s_{i,k-1})`，否则接触位置在杆上迁移时会混入错误位移。前驱形状也来自窗口里的非线性平衡状态，不是真值或直接差分的含噪重建。

完整互补关系为：

```text
0 <= g_i = n^T(pc_i-pPlane)  ⟂ fn_i >= 0
0 <= w_i = D^T v_i + lambda_i 1  ⟂ beta_i >= 0
0 <= cone_i = mu_i fn_i - sum(beta_i)  ⟂ lambda_i >= 0
```

无摩擦时固定 beta=0，但后续帧的 lambda 仍保留，以允许接触点切向移动。固定 lambda=0 会错误地要求 `D^T v>=0`，在成对摩擦方向下强迫 v=0。

schema 2 的第一帧没有前驱，因而只进行静态摩擦锥审计；存在非零摩擦接触时明确标为需要复核，不能称为已验证的黏着。schema 1 保存的输出时刻可能隔了多个传感器周期，迁移时合并所有当前与前驱时刻，而非只添加第一个历史样本。`observations.predecessorIndex(k)` 明确指出当前样本应使用哪个平衡状态，不能盲目使用时间数组中的上一项。

例如旧 `sliding_clean` 第 10/11 个视频状态的时间是 4.26/4.32 s，前驱实际为 4.24/4.30 s。新窗口求解 `[4.24,4.26,4.30,4.32]` 四个平衡状态，返回第 2/4 个；它不会用 4.26 s 的视频帧替代 4.30 s 的真实前驱。原始曲率与基座按时间去重，共享时间的数据冲突会被拒绝。历史时刻没有独立相机观测时，复制几何只作为初值，`environmentLikelihood=false`；仅当前时刻贡献环境似然。历史缺少的摩擦系数沿用配对当前值，因此参数在未观测的一采样周期内不变仍是适配假设。

## 7. 全杆几何、候选与数值求解

每个环境平面都定义自由空间半空间。算法采样连续积分形状进行全部半空间的非穿透检查，默认间隔 1 mm；接触点间隙另外连续查询。内点光滑接触还施加力门控切触 `fn * n^T tangent=0`。默认检查中心线，不包含机器人外径；有限面片、曲面或角点需单独几何模型，不能直接套全局半空间。

候选来源是稀疏重建形状对各面的间隙局部极小值。默认允许距墙 5 mm 的候选，同面 10 mm 内的候选合并，最多四个；超限和相同弧长的角点歧义都会记录并触发 review。候选按弧长排序，相邻候选中点形成搜索分区，接触不跨越该分区。这是保证接触顺序及连续积分数值稳定的有限搜索范围，不能声称已经处理任意出生/消失或大范围接触迁移。

初始化使用重建形状的力矩关系，只拟合实际测得的弯曲分量，同时拟合法向力、非负摩擦生成系数和未知末端力；线性拟合已考虑摩擦锥。它只产生初值，最终目标和约束均重新积分当前三维力学。

无摩擦的后续帧中，初始化另将 lambda 设置为 `max(0,max(-D^T v))+1e-4 mm`（不超过配置边界）。这样 `w` 初始可行，避免数值最小二乘停在几微米以下的负松弛边界。lambda 仍是后续 MAP 的优化变量，完整互补关系没有删除或变成速度为零的约束。

初始化还按每个平面的当前非零反力候选平均间隙，沿法向修正一次平面点初值。这减少极小间隙乘以较大法向力时的互补残差；平面仍是带原始相机似然和过程先验的待估变量，求解阶段不会固定在该初值。可用 `options.initialState` 传入相同候选坐标和时间布局的既有估计作为重启初值，维度和边界必须一致；它不会变成另一份后验先验，正式七场景协议未使用真值或既有结果来初始化。

初值经过非线性最小二乘后进入 SQP/Scholtes 连续化。初始化先增加 `1e-4 /mm`、`1e-5 /mm` 的数值曲率方差，最后回到原始观测协方差；这只改善初始求解，不修改最终 MAP 的观测噪声。目标梯度先求残差雅可比再计算 `J^T r`，减少直接差分大数平方和的消减误差。默认无量纲 tau 为 `1e-2,1e-4,1e-6,1e-8`。SQP 无可接受退出且此前无原约束可行解时，允许对同一目标/约束尝试一次内点法；失败码与重试退出全部保留，不换成简化力学。

每级都保存退出码、原始与松弛约束残差、目标值、迭代和耗时。若后一级破坏可行解，保留此前原始完整约束在 `complementarityTolerance` 内的更低目标状态，记录其实际来源级数，不把失败退出改成成功。正退出且所有原始约束残差已小于 `exactContinuationTolerance=1e-8` 时结束连续化，不重复求解已满足的更紧松弛。该终止准则检查未松弛的约束，不是只看当前 tau 可行。最终逐帧审计仍分别检查物理单位的平衡与几何阈值，不以一次优化正退出代替所有检查。

当前默认长度/力尺度为 1 mm/1 N；末端力矩按 EI/L 缩放。数值优化器默认约束容差 `constraintTolerance=1e-5`，与互补审计的数量级一致；原数学约束保持为零互补。最终质量仍另外检查 N mm、mm 及原始互补量：默认末端力矩阈值 2e-4 N mm，全杆间隙允许数值误差 2e-4 mm，互补/力门控切触残差阈值 1e-5。无量纲优化器容差不会覆盖更严格的物理平衡检查，二者不相互替代，也不是传感器力精度指标。

## 8. 质量、不确定度与可辨识性

逐帧输出有限性、末端力矩残差、最小全杆间隙、完整互补残差、切触残差、白化弯曲残差 RMS、历史是否可用，以及 `requiresReview`。优化失败、候选被截断、角点歧义、物理残差超标或缺少摩擦历史都保留，不会因为得到有限力值而改判为成功。

`computeCovariance=true` 时，在推断的活跃分支上，对窗口目标和活跃约束（包括触及的状态上下界）求数值雅可比，用约束零空间计算局部协方差。平面、位置和基座力矩留在状态中，因此力的协方差包含这些扰动变量在局部线性模型中的耦合。另用“只含测量”的信息矩阵检查：有限后验是否仅由先验撑起来。观测零空间仍能改变力时输出 `forceUnresolvedByData` 并触发复核。

该结果以候选集合、活跃模式及已知刚度/固有曲率为条件。它没有接触模式混合、刚度不确定度或实物标定的覆盖保证；弱活跃互补点尤其可能产生多个分支。不写成全局置信区间已校准，更不宣称安全认证。

信息求逆直接对约束切空间中的白化雅可比 `H=J*N` 做 SVD，不对 `H^T*H` 取伪逆，避免把高精度曲率下的条件数再次平方。无法解析的方向单独通过零空间标志报告，力标准差设为 Inf；测量与含先验的信息秩阈值也随结果保存。有限局部方差不等于测量已经足够确定分力。

## 9. 使用与可复现文件

```matlab
addpath('rod');

% 新传感器包：全部给定时刻一起求解并导出文件
[estimate, manifest] = run_formulation_workflow(sensorInput, ...
    fullfile(pwd,'out','my_experiment'),struct('computeCovariance',true));

% 对旧 schema-1 包求解连续三帧及首个前驱
estimate = estimate_temporal_window_forces(sensorInput,3);

% 七组软件实验：双接触、收窄双壁、三接触、旋转、含噪、空间滑动及其含噪版本
report = force('multi-formulation');
% 独立 smoke 目录，避免覆盖正式结果
report = force('multi-formulation',true);
```

schema 2 的主要字段为 `sFbgMm`、`observedCurvatureAxes`、`curvaturePerMm(channels×sensors×T)`、`curvatureStdPerMm` 或 `curvatureCovariance`、`basePose(4×4×T)`、`timeSeconds`、`planePointMm/planeNormal(3×P×T)`、`planeCovariance(6×6×P×T)`、`frictionMu(P×T)`。完整曲率协方差的维度为 `(channels*sensors)²×T`，与 MATLAB `curvaturePerMm(:,:,k)(:)` 的按传感器展开顺序一致；提供该矩阵时不要求重复给出标准差。二维观测不会自动变成三维传感器。生成接口例子见 `build_formulation_multi_packet`。

通用工作流导出 `input.mat`、`estimate.mat`、`forces.csv`、`manifest.json`。实验协议另导出独立的 `truth.mat` 和 `comparison.json`。CSV 给出每个接触的世界坐标力分量、大小、位置、独立末端力和合力大小，质量标志一起导出。MAT 中保留连续 ODE 结果用于追踪历史材料点；CSV/JSON 用于直接阅读，不应把这些仿真输出当作真实录像或实物实验。

七组协议各取两个相邻平衡状态；双/三接触通道的独立真值生成器先求完原 12 个状态，再选择第 3/4 个形成窗口。它们是窗口级算法实验，不是新生成的 12 帧全轨迹视频。旧连续视频保持其原有求解记录，不用短窗口结果替换或伪造 MP4。

旧压力测试生成器也已替换：[build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) 现在先求固定表面的独立接触平衡，然后只对逆解隐藏一侧环境面或将摩擦系数从 0.03 改为 0.01。`curved-surface` 是保留的命令别名，对应非平行平面通道，明确不是光滑曲面的真实接触。旧规定载荷的分数标注撤回，不与新协议拼接。

## 10. 仍待优化的具体范围

完整离线逆解已经连接，但还不是任意场景都准确的成品。后续应扩大独立轨迹统计，研究候选分区失效、模式切换和材料参数失配；把有限平面/曲面几何和机器人半径明确加入碰撞模型；实现可控延迟的递归/边缘化窗口。空间真值目前采用预设连续滑动分支，没有独立黏滑转换。已有论文基线的正式复现、全局覆盖率、实物传感器和实时性也不能由本轮代码推出。

本轮修改没有偏离项目宗旨：环境信息和稀疏形状信息仍是估计力的观测来源。新增窗口和候选集合用于完善这一逆问题；求解技术与常规不确定度计算不单独当作新颖性声明。
