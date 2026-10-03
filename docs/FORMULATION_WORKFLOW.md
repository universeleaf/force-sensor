# EnFiRCE：三维多接触与完整时间窗口实现

更新：2026-10-04。本文说明完整软件流程及其与 `papers/Formulation.pdf` 的关系，补充[技术总说明](TECHNICAL_OVERVIEW.md)。历史七窗口数值以 `out/formulation_window/comparison.json` 为准；逐接触真值与估计见[历史窗口结果](../out/formulation_window/summary.md)。新的多种子/多噪声、六因素、同包基线及性能协议见[完整三维因素说明](FORMULATION_FACTORS.md)，各轮结果保留各自执行源码。

## 1. 这条路径实际解决什么问题

输入是已标定的杆模型、稀疏弯曲测量、基座位姿、环境平面观测与协方差。算法从观测重建的形状中产生接触候选，然后联合优化窗口内各帧的环境参数、身体接触位置、法向力、摩擦力、末端力和基座力矩。每帧均满足当前载荷下的三维 Cosserat 平衡；相邻帧的真实形状共同决定摩擦位移。

二维多接触原型 `estimate_planar_multi_contact` 继续用于历史 benchmark 与独立对照。新的逆解没有读取它的接触数、接触顺序、接触位置或力标签。候选数由形状和环境决定，优化中的法向互补允许某个候选变成零反力。这里的“未知接触”是有上限的候选搜索，并非对任意接触拓扑进行全局枚举。

## 2. 从输入到输出的代码地图

| 步骤 | 实现文件 | 职责 |
|---|---|---|
| 对外入口 | [estimate_formulation_forces.m](../rod/estimate_formulation_forces.m) | schema 2 自动进入新路径；schema 1 传第二个 options 参数也进入新路径 |
| 完整文件工作流 | [run_formulation_workflow.m](../rod/run_formulation_workflow.m) | 保存输入，运行全部给定时刻，保存估计、CSV、质量、源码和数据校验和；保留失败记录 |
| 输入验证和迁移 | [formulation_window_observations.m](../rod/formulation_window_observations.m) | 验证时间、测量维度、位姿和协方差；schema 1 当前/前驱时间合并，同一观测只加入一次 |
| 接触候选 | [formulation_contact_candidates.m](../rod/formulation_contact_candidates.m)、[track_formulation_candidates.m](../rod/track_formulation_candidates.m) | 稀疏形状重建、逐帧间隙极小值合并、跨帧一对一关联、排序、数量限制和搜索审计 |
| 接触搜索域 | [formulation_contact_arc_domain.m](../rod/formulation_contact_arc_domain.m) | 整根杆范围、端点保护、硬线性最小间距；旧首帧分区作为显式消融 |
| 共享导数 | [formulation_derivative_bundle.m](../rod/formulation_derivative_bundle.m) | 一次中央差分同时得到目标、非负、互补与平衡的雅可比；精确状态缓存 |
| 状态布局 | [formulation_window_spec.m](../rod/formulation_window_spec.m) | 每个独立观测平面保存一份几何，多个接触共享它；末端力属于物理状态和过程先验 |
| 法向与摩擦方向 | [formulation_contact_frame.m](../rod/formulation_contact_frame.m) | 球面指数参数化，切向基随法向连续转动，生成多面体摩擦方向 |
| 联合 MAP/MPCC | [estimate_formulation_window.m](../rod/estimate_formulation_window.m) | 初始拟合、原始似然、一次初始先验、过程因子、完整互补、同伦、逐帧质量及局部协方差 |
| 当前三维力学 | [integrate_cosserat_load_state.m](../rod/integrate_cosserat_load_state.m) | 连续接触位置处分段；直接积分当前形状与载荷；返回末端力矩作为硬约束 |
| 任意材料点查询 | [cosserat_state_at_arc.m](../rod/cosserat_state_at_arc.m) | 连续 ODE 插值查询当前/前驱的同一弧长位置 |
| 时间窗口兼容入口 | [estimate_temporal_window_forces.m](../rod/estimate_temporal_window_forces.m) | 替代旧 penalty smoother，筛选连续时间后调用同一完整逆解 |
| 数据子集 | [subset_formulation_packet.m](../rod/subset_formulation_packet.m) | 同步选择曲率、基座、环境、摩擦系数和协方差 |
| 独立二维真值输入 | [build_formulation_multi_packet.m](../rod/build_formulation_multi_packet.m) | 将独立二维平衡嵌入三维坐标；只导出稀疏传感器信息给逆解 |
| 空间滑动真值 | [build_spatial_friction_packet.m](../rod/build_spatial_friction_packet.m) | 独立嵌套射击与接触根求解，生成面外摩擦的三维双接触平衡及平移历史 |
| 无杆身接触真值 | [build_formulation_free_packet.m](../rod/build_formulation_free_packet.m) | 独立射击，仅有未知末端载荷，观测平面远离杆形，逆解须自动产生零接触候选 |
| 独立前向射击 | [solve_cosserat_multi_contact_map.m](../rod/solve_cosserat_multi_contact_map.m) | 不使用逆解的 lifted state；求基座力矩使末端力矩为零 |
| 实验和评分 | [run_formulation_window_protocol.m](../rod/run_formulation_window_protocol.m)、[score_formulation_window.m](../rod/score_formulation_window.m) | 输入、真值、估计分别存储；真值仅进入评分；失败和 review 均保留 |
| 通用力表 | [write_formulation_window_csv.m](../rod/write_formulation_window_csv.m) | 每个时刻/接触一行，报告世界坐标力分量、大小、位置和 review |
| 噪声尺度的拟合诊断 | [audit_formulation_window_fit.m](../rod/audit_formulation_window_fit.m) | 只使用观测与预测，检查噪声不能解释的模型/观测张力 |
| 既有结果复核 | [review_formulation_artifacts.m](../rod/review_formulation_artifacts.m) | 保留原始力、质量和源码记录，另外导出带源码记录与估计文件校验和的派生诊断 |

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

候选来源是稀疏重建形状对各面的间隙局部极小值。默认允许距墙 5 mm 的候选，只在同一帧、同一面内合并 10 mm 范围的极小值，然后跨帧进行一对一轨迹关联，最多四个候选；超限、角点与关联歧义都进入审计。默认搜索域为整根杆，仅施加端点保护和相邻槽位至少 0.1 mm 的硬线性间距；接触不再被首帧候选中点锁住。旧分区可显式设置为 `contactArcMode='partitioned'` 进行消融。有限候选及固定槽位仍不等于对任意接触拓扑进行全局枚举。具体关联门限和每帧位置初值见[三维因素说明](FORMULATION_FACTORS.md)。

初始化使用重建形状的力矩关系，只拟合实际测得的弯曲分量，同时拟合法向力、非负摩擦生成系数和未知末端力；线性拟合已考虑摩擦锥。它只产生初值，最终目标和约束均重新积分当前三维力学。

无摩擦的后续帧中，初始化另将 lambda 设置为 `max(0,max(-D^T v))+1e-4 mm`（不超过配置边界）。这样 `w` 初始可行，避免数值最小二乘停在几微米以下的负松弛边界。lambda 仍是后续 MAP 的优化变量，完整互补关系没有删除或变成速度为零的约束。

初始化还按每个平面的当前非零反力候选平均间隙，沿法向修正一次平面点初值。这减少极小间隙乘以较大法向力时的互补残差；平面仍是带原始相机似然和过程先验的待估变量，求解阶段不会固定在该初值。可用 `options.initialState` 传入相同候选坐标和时间布局的既有估计作为重启初值，维度和边界必须一致；它不会变成另一份后验先验，正式七场景协议未使用真值或既有结果来初始化。

重启入口允许优化器自身产生的边界舍入误差：每个状态分量最多 `1e-9*max(1,abs(x))`，超过这一范围仍拒绝。仅将这个范围内的数值投回原始边界，并记录 `solver.warmStateBoundCorrection`，不会改变实质上不满足边界的输入。这修复了已保存估计的 beta 为极小负数、反而不能作为下一次初值的问题。候选布局必须由调用者确认一致；仅有相同矩阵尺寸不足以证明两个不同场景的接触槽对应。

初值经过非线性最小二乘后进入 SQP/Scholtes 连续化。初始化先增加 `1e-4 /mm`、`1e-5 /mm` 的数值曲率方差，最后回到原始观测协方差；这只改善初始求解，不修改最终 MAP 的观测噪声。目标梯度先求残差雅可比再计算 `J^T r`，减少直接差分大数平方和的消减误差。默认无量纲 tau 为 `1e-2,1e-4,1e-6,1e-8`。SQP 无可接受退出且此前无原约束可行解时，允许对同一目标/约束尝试一次内点法；失败码与重试退出全部保留，不换成简化力学。

每级都保存退出码、原始与松弛约束残差、目标值、迭代和耗时。若后一级破坏可行解，保留此前原始完整约束在 `complementarityTolerance` 内的更低目标状态，记录其实际来源级数，不把失败退出改成成功。正退出且所有原始约束残差已小于 `exactContinuationTolerance=1e-8` 时结束连续化，不重复求解已满足的更紧松弛。该终止准则检查未松弛的约束，不是只看当前 tau 可行。最终逐帧审计仍分别检查物理单位的平衡与几何阈值，不以一次优化正退出代替所有检查。

当前默认长度/力尺度为 1 mm/1 N；末端力矩按 EI/L 缩放。数值优化器默认约束容差 `constraintTolerance=1e-5`，与互补审计的数量级一致；原数学约束保持为零互补。最终质量仍另外检查 N mm、mm 及原始互补量：默认末端力矩阈值 2e-4 N mm，全杆间隙允许数值误差 2e-4 mm，互补/力门控切触残差阈值 1e-5。无量纲优化器容差不会覆盖更严格的物理平衡检查，二者不相互替代，也不是传感器力精度指标。

## 8. 质量、不确定度与可辨识性

逐帧输出有限性、末端力矩残差、最小全杆间隙、完整互补残差、切触残差、白化弯曲残差 RMS、历史是否可用，以及 `requiresReview`。优化失败、候选被截断、角点歧义、物理残差超标或缺少摩擦历史都保留，不会因为得到有限力值而改判为成功。

`computeCovariance=true` 时，在推断的活跃分支上，对窗口目标和活跃约束（包括触及的状态上下界）求数值雅可比，用约束零空间计算局部协方差。平面、位置和基座力矩留在状态中，因此力的协方差包含这些扰动变量在局部线性模型中的耦合。另用“只含测量”的信息矩阵检查：有限后验是否仅由先验撑起来。观测零空间仍能改变力时输出 `forceUnresolvedByData` 并触发复核。

该结果以候选集合、活跃模式及已知刚度/固有曲率为条件。它没有接触模式混合、刚度不确定度或实物标定的覆盖保证；弱活跃互补点尤其可能产生多个分支。不写成全局置信区间已校准，更不宣称安全认证。

信息求逆直接对约束切空间中的白化雅可比 `H=J*N` 做 SVD，不对 `H^T*H` 取伪逆，避免把高精度曲率下的条件数再次平方。无法解析的方向单独通过零空间标志报告，力标准差设为 Inf；测量与含先验的信息秩阈值也随结果保存。有限局部方差不等于测量已经足够确定分力。

另有噪声尺度的观测拟合诊断。设一个输出时刻有 nu 个实际曲率观测，白化残差为 e，则参考量是 `E=e'*e`。使用完整观测维数 nu 的 chi-square 上界；默认窗口总尾概率为 0.001，按输出时刻数作 Bonferroni 分配，阈值计算为 `2*gammaincinv(1-0.001/T,nu/2)`，不需要 Statistics Toolbox。48 维观测、两个输出时刻时，等效 RMS 阈值约 1.3455。原固定 RMS 阈值为 4，可能漏掉已严重失配但仍满足局部模型的结果。

这只是观测/模型张力诊断，没有宣称拟合后的自由度、候选选择及先验影响已经校准；它不能断定失配的唯一原因，也不能证明每个遗漏障碍物都会被发现。复核既有文件时另写 `observation_fit.json`，其中保存原始复核标志、新拟合诊断及二者合并值；原始估计和原始 quality 不被追改。

当前新求解会自动保存 `result.observationFit`，将其布尔结果写入 `quality.hasObservationFitWarning` 并合并到 `requiresReview`。七组正式窗口及三个失配实验在这次集成之前已完成，因此原始复核标志仍照实保留，报告分列原始与派生合并值。无接触工作流是在集成之后运行的，结果直接包含新字段。

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

% 已完成文件的拟合诊断，不重复优化、不改动原始力
fitReport = review_formulation_artifacts(fullfile(pwd,'out','formulation_window'));

% 无杆身接触、仅末端载荷：独立生成传感器包，然后按同一工作流求解
[sensorInput, truth] = build_formulation_free_packet();
[estimate, manifest] = run_formulation_workflow(sensorInput, ...
    fullfile(pwd,'out','free_space_replay'),struct('computeCovariance',true));

% Python 生成逐接触结果表；该 Git 版本对应本轮七场景的求解代码
% python scripts/summarize_formulation_window.py --source-revision 813cdfb
```

schema 2 的主要字段为 `sFbgMm`、`observedCurvatureAxes`、`curvaturePerMm(channels×sensors×T)`、`curvatureStdPerMm` 或 `curvatureCovariance`、`basePose(4×4×T)`、`timeSeconds`、`planePointMm/planeNormal(3×P×T)`、`planeCovariance(6×6×P×T)`、`frictionMu(P×T)`。完整曲率协方差的维度为 `(channels*sensors)²×T`，与 MATLAB `curvaturePerMm(:,:,k)(:)` 的按传感器展开顺序一致；提供该矩阵时不要求重复给出标准差。二维观测不会自动变成三维传感器。生成接口例子见 `build_formulation_multi_packet`。

通用工作流导出 `input.mat`、`estimate.mat`、`forces.csv`、`manifest.json`。实验协议另导出独立的 `truth.mat` 和 `comparison.json`。CSV 给出每个接触的世界坐标力分量、大小、位置、独立末端力和合力大小，质量标志一起导出。MAT 中保留连续 ODE 结果用于追踪历史材料点；CSV/JSON 用于直接阅读，不应把这些仿真输出当作真实录像或实物实验。

七组协议各取两个相邻平衡状态；双/三接触通道的独立真值生成器先求完原 12 个状态，再选择第 3/4 个形成窗口。它们是窗口级算法实验，不是新生成的 12 帧全轨迹视频。旧连续视频保持其原有求解记录，不用短窗口结果替换或伪造 MP4。

旧压力测试生成器也已替换：[build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) 现在先求固定表面的独立接触平衡，然后只对逆解隐藏一侧环境面或将摩擦系数从 0.03 改为 0.01。`curved-surface` 是保留的命令别名，对应非平行平面通道，明确不是光滑曲面的真实接触。旧规定载荷的分数标注撤回，不与新协议拼接。

## 10. 当前模型范围与后续研究

完整离线逆解已经连接。默认固定位置分区、跨帧重复候选和退化初始化问题已修复，所有原始约束仍在最终接受时审计。现有模型是中心线对平面半空间的接触；有限表面、光滑曲面与机器人半径并未偷偷加入当前结果。后续研究包括独立轨迹、模式切换、材料失配及有边缘化的递归窗口。空间真值目前采用预设连续滑动分支；已有论文原作者实现的正式复现、全局覆盖率、实物传感器与实时性不能由软件回归检查推出。

本轮修改没有偏离项目宗旨：环境信息和稀疏形状信息仍是估计力的观测来源。新增窗口和候选集合用于完善这一逆问题；求解技术与常规不确定度计算不单独当作新颖性声明。

## 11. 本轮场景与参数的具体含义

双接触的杆长 140 mm，由两个 70 mm 的固有曲率区段组成，曲率幅值为 0.02 /mm，基座初始角为 -0.7 rad；两壁位于 x=-10 和 x=10 mm。三接触的杆长为 210 mm，增加一个固有曲率区段，形成“左壁—右壁—左壁”的蛇形接触。收窄通道的两面法向为归一化后的 `[1,0,-0.05]` 和 `[-1,0,-0.05]`，是两个不平行的平面，不是光滑曲面。

参数来自 [multi_contact_demo_scenes.m](../rod/multi_contact_demo_scenes.m)。12 个原始状态的基座 x 平移为 -0.6 到 +0.6 mm；新窗口取其中第 3/4 个，时间间隔 0.02 s。每个时刻有 24 个位置的两个实际弯曲通道；含噪条件各通道的 Gaussian 标准差为 2.5e-5 /mm，无噪声条件仍保留 1e-7 /mm 的数值协方差。平面观测的默认位置标准差为 0.05 mm、法向分量标准差为 0.001。

默认杆标定沿用外部依赖 `CreatTube`：弯曲刚度 EI=200700 N mm²（0.2007 N m²），泊松比 0.3、扭转刚度 EI/1.3，内外半径分别为 0.455/0.66 mm。逆解适配器断言真值与逆解的 EI 相同；目前半径不参与半空间碰撞约束。这些力数值对应该仿真刚度和形变，不是对硬件机器人量程的实测声明。若更换刚度、长度或固有曲率，接触反力也应由独立接触平衡重新生成，而非只缩放现有视频标签。

“整体旋转”使用两个旋转组成的刚体变换，基座、环境点、法向和所有世界坐标真值力一起变换；杆的局部曲率保持不变。它检查三维坐标处理，不冒充本来在二维平面中的真值具有面外变形。

真正面外加载的场景由 [build_spatial_friction_packet.m](../rod/build_spatial_friction_packet.m) 生成：mu=0.03，末端力 `[0.1,0.3,-0.08]` N，两个接触的摩擦沿 -y，独立求解法向反力与接触弧长。两个状态的基座与形状沿 +y 平移 0.5 mm，产生明确的切向相对位移；逆解不知道该滑动模式、接触数或真值力。第一状态没有更早的平衡，只验证静态锥；第二状态才拥有完整历史摩擦条件。尚未生成独立的黏着转滑动事件真值。

## 12. 为什么离线完整求解仍然慢

每个新的力学状态都需要三维 Cosserat 积分。当前目标、约束及局部协方差复用同一精确中央差分矩阵，单帧力学状态还经过 LRU 缓存；仅几何变化不会重积分。导数维数仍随平面数、候选数和窗口长度增长。初值有三档曲率尺度，每档最多 50 次迭代；正式同伦每级最多 100 次迭代，另有可行性恢复和活动分支精化。力向量的协方差雅可比直接从参数构造，不再额外积分杆形。

`optimizationSeconds` 计入初始化和 MAP/MPCC 优化，在局部协方差计算之前取值；`endToEndSeconds` 包含该估计调用的协方差工作，但不包含 MATLAB 启动、独立真值生成和文件输出。历史运行曾与其他求解任务并行。当前因素实验与导数对照在同一 MATLAB 批处理中顺序执行；固定运行顺序和单次对照仍不足以证明延迟显著性或实时性。

可按需要选择短窗口、关闭协方差，或复用相同坐标布局的初值；这些是公开 options，并须在结果中说明。论文级性能改进应进一步实现力学与约束的灵敏度、稀疏雅可比、递归窗口和合理的边缘化，而不是把完整模型偷偷替换成小挠度公式。

## 13. 已完成的实际结果与版本

七场景运行 ID 为 `2530e6e5-741c-4337-8a23-221aaefb496b`，全部完成，无运行失败。求解时源码内容经逐文件核对对应 Git `813cdfb`；之后新增入口舍入处理、拟合诊断、空接触真值与依赖记录。因此，原始记录与最新工作区的 SHA 不同是有解释的版本差异，不覆盖或伪造原始源码记录。

| 条件 | 逐接触力向量 RMSE / N | 接触位置 RMSE / mm | 总合力 RMSE / N | 原始复核 / 两状态 |
|---|---:|---:|---:|---:|
| S 通道双接触 | 5.3812e-7 | 2.4901e-8 | 6.6962e-7 | 0/2 |
| 收窄通道双接触 | 1.6372e-6 | 1.1975e-7 | 1.2846e-6 | 0/2 |
| 蛇形三接触 | 9.2954e-7 | 1.1540e-8 | 1.1084e-6 | 0/2 |
| 刚体旋转双接触 | 5.5931e-7 | 2.6901e-8 | 6.8652e-7 | 0/2 |
| 三接触，曲率噪声 2.5e-5 /mm | 0.600524 | 0.0235349 | 0.819888 | 0/2 |
| 空间摩擦双接触 | 7.5473e-7 | 4.4227e-8 | 1.2330e-6 | 1/2 |
| 空间摩擦双接触，相同噪声 | 0.330189 | 0.0321973 | 0.479752 | 1/2 |

例如含噪三接触的第一状态，三个力的真值分别为 134.477452、44.760219、13.072686 N，估计为 135.384216、45.439857、13.823778 N。含噪空间双接触第一状态的真值为 107.975766、16.252634 N，估计为 107.762958、16.076764 N。完整两时刻、每个接触的力大小、向量误差、末端力误差、位置和逐场景 CSV 链接均见[窗口结果表](../out/formulation_window/summary.md)。第一空间状态仍因历史缺失复核，不因数值接近真值而改判。

无杆身接触在最终集成代码下完成两个状态，候选数为 0，真实末端力为 `[0.12,0.08,-0.06]` N，大小 0.156205 N；末端力 RMSE 为 6.7844e-10 N，无复核标志。[原始力表](../out/formulation_window/free_space/forces.csv)、[指标](../out/formulation_window/free_space/metrics.json)和[运行记录](../out/formulation_window/free_space/manifest.json)保留。CSV 仍需显示末端力，因此输出一条 `active=0, planeIndex=NaN, arcMm=NaN` 的零接触占位行，不表示推断了一个接触；逐接触误差和位置指标不适用，JSON 中为 null。

旧单接触 `sliding_clean` 的第 10/11 个输出时刻，经新工作流求解四个平衡状态，接触力 RMSE 为 2.0774e-6 N、末端力 RMSE 2.2817e-6 N、总合力 RMSE 3.0188e-7 N。它复用了前一次纯观测求解的状态作初值，本次优化耗时 1239.01 s，不能作为冷启动速度。边界舍入修正量为 3.5995e-24。[评分记录](../out/formulation_window/schema1_sliding_replay/validation.json)和[输出力表](../out/formulation_window/schema1_sliding_replay/forces.csv)均保留。

修正后的三组失配运行 ID 为 `7f9e8ef0-e13e-4530-b4b6-87a0ca2236ac`，总合力 RMSE 分别为 17.8341、15.9639、1.64746 N。前两组的单接触候选无法表达未观测墙面的反力；这不能靠优化器退出成功证明正确。新增拟合诊断均在两时刻触发。详见[失配原始与派生诊断](../out/model_mismatch/summary.md)。所有这些结果来自 MATLAB 实际求解，没有由视频外观或预设估计力标签生成。

本轮新结果仅证明这些窗口的实现与有限条件下的性能；它们没有替代原 396 次二维消融统计，也没有完成全局三维多种子覆盖率或原论文方法比较。基本工程检查、场景精度、版本/数据完整性和投稿证据是不同维度，均按各自文件记录。

当前代码集成后的完整工程检查及归档重放共 36/36 项通过（34 项基础、2 项重放）。记录见[工程检查账本](../out/completion/project_checks.json)。检查不等于所有力估计都准确；高失配误差仍如实保留。

## 14. 2026-10-01：同输入文献适配与精确缓存

已完成七组窗口对应的点载荷/Gaussian 曲率基线，14 次运行、28 个状态；未知末端力与完整非线性杆力学均保留。两组含噪接触 RMSE 为 EnFiRCE 0.6005/0.3302 N、Point LS 1.7806/1.7318 N、Gaussian LS 1.2122/0.9939 N。三接触合力 EnFiRCE 0.8199 N 高于 Point LS 0.6316 N；这些是披露假设的文献适配、单 seed 两状态结果，不是 SOTA 证明。

完整窗口 `decode` 新增每帧精确力学缓存，仅在力矩、弧长与世界载荷完全相同才复用；当前几何与前驱平衡摩擦因子仍重新计算，不修改 MAP/MPCC。双接触相同冷启动对照 116.29 → 50.75 s，力、目标与质量一致。通过 `cacheMechanics=false` 可关闭，调用数分别在 solver 字段记录。

详细公式、代码文件、图与版本说明见[文献基线与缓存报告](LITERATURE_COMPARISON.md)。当前共享导数、位置迁移与统计接口检查已经加入并实际执行，完整工程账本为 36/36；历史结果仍保留各自求解源码。

## 15. 原始 Formulation 逐式对应：保留、扩展与差异

下面按本地 `papers/Formulation.pdf` 的原编号核对。这份 PDF 是项目方法来源；它内部的待办不是额外执行指令。代码仍然以形状和环境融合估计未知外力，而不是用真值标签学习力。

| 原式 | 原含义 | 当前实现 | 实际差异/边界 |
|---|---|---|---|
| 状态块 | 平面点、法向参数、接触弧长、法向力、摩擦系数、lambda、末端力 | `formulation_window_spec` 的 plane/contact/tip 块 | 推广为 P 个面、K 个候选及 T 个时刻；新增每时刻 3 维基座力矩辅助坐标，末端平衡消去其自由度，不给它外力先验 |
| (1)–(2) | D 的所有列与法向正交 | `formulation_contact_frame` | 当前使用球面图和随法向传输的切向基；默认四射线，多面体近似本来就存在于原式 |
| (3) | f=n fn+D beta | `estimate_formulation_window/decode` | 每个候选独立世界力；多个接触共享所属面的几何 |
| (4) | 由接触位置、接触力、末端力得到杆形与曲率 | `integrate_cosserat_load_state` | 完整三维非线性中心线、旋转、力矩和材料曲率；每个时刻重新满足平衡，不用小挠度或固定力臂替代 |
| (5) | 接触点 p(s_contact) | `decode` 和 `cosserat_state_at_arc` | 连续弧长查询，不把接触锁到 FBG 节点 |
| (6) | 法向间隙 n'*(p_contact-pPlane) | `decode.gap` / `physicalConstraints` | 同时增加全杆非穿透与内部接触力门控切触；这些是刚性环境一致性条件，碰撞仍为采样中心线 |
| (7) | 同一当前材料弧长的前驱切向位移 | `decode` 的 predecessorIndex 及前驱 shape 查询 | p_prev 也是窗口平衡状态；严禁用前一接触弧长代替当前弧长。无真实前驱时只保留静态锥并复核 |
| (8)–(11) | 曲率、平面点和法向的观测/预测及噪声 | `formulation_window_observations`、`residual` | 只选实际测得的两个弯曲通道；支持完整曲率/环境协方差白化。每面每时刻只计一次环境似然 |
| (12) | 独立高斯先验 | `initialPrior` 或默认物理先验 | 可给完整正定 P0；初始几何默认弱先验，避免把首帧相机观测再算一遍 |
| (13) | 状态随机游走 | `residual` 的 processW 行 | 保留完整物理状态的时间因子，时间协方差按实际 dt 缩放；辅助基座力矩不属于物理状态 |
| (14) | 递归后验均值/协方差的预测 | 旧单帧更新保留；新窗口用联合随机游走 MAP | 当前主路径没有实现已边缘化的递归滤波器，不把 warm start 当作递归后验；需要另行设计滑动窗口边缘化 |
| (15)–(18) | 法向、逐射线、锥饱和三类互补 | `physicalConstraints` / `constraints` | 完整非负与互补对均保留；先解 Scholtes 乘积松弛，再按推断分支精化；最终验原始未松弛条件 |
| (19) | 先验与观测白化二次目标 | `residual` / `objective` | 同一 MAP 思想推广为联合时间窗口；基座力矩只用于约束力学，不引入虚构测量 |
| (20)–(22) | 原 MAP 中三类互补约束 | `constraints`、`polishBranch` 和 `audit` | 分支精化只去数值冗余/恒等零行；原同伦和最终原约束审核保留全部条件，未删掉摩擦模型 |
| (23) | 接触弧长范围 0 至 L | `formulation_contact_arc_domain` | 当前模型保护两个端点并施加 0.1 mm 最小顺序间隔，因内部平面切触不能表示端点/角点接触。全长有序范围替代首帧固定分区 |
| (24)–(27) | 迭代线性化的 constrained EKF 子问题 | 旧单接触保留；当前直接优化完整非线性残差 | 当前窗口使用 SQP/内点法求原非线性 MAP，不逐字复现该线性化 EKF 更新。目标与物理约束一致，数值算法扩展明确披露 |
| (28)–(29) | 后验一阶协方差 | `localCovariance` / `derivatives` | 当前对原始因子雅可比在活动约束切空间求协方差，比原无约束近似多考虑活动边界；局部模式条件成立，未校准全局后验 |

论文主路径与原文的关系是“同一个观测/接触/摩擦 MAP 问题的完整非线性多接触时间窗口扩展”。不能写成“已逐字实现原递归 EKF”或“已完成所有硬件计划”。两份原项目文档都把杆身载荷归因于环境接触，把任务末端力独立建模；当前保留这一点。环境是无限平面半空间，项目文档提到的球面仍未进入这里的物理结果。

局部协方差的后验矩阵包含初始/过程先验；`measurementInformationRank` 和 `forceUnresolvedByData` 单独用实际测量行检验。`score_formulation_coverage` 要求可辨识且有限的力标准差才计覆盖，并报告未辨识分母。这使“可行、有有限后验、由观测确定、实际误差小”成为四项不同事实。
