# 多接触扩展：从当前单接触状态到可复现的双接触实验

当前公开逆解确实是**单接触状态**。它把一个环境平面、一个接触弧长、一个法向力、一个摩擦锥和一个独立末端力放进 MAP 状态：

\[
x_1=[p_\pi,\eta,s_1,f_{n,1},\beta_1,\lambda_1,f_e].
\]

其中 `pπ` 是平面点，`η` 是二维法向参数，`s1` 是接触弧长，`β1` 是 16 条摩擦锥生成方向的非负系数，`λ1` 是锥饱和互补变量，`fe` 是末端三维力。因此默认状态维度是 `11+m=27`。代码中的对应位置是：

- 状态维度和索引：[formulationStateSize.m](../rod/run_rod_plane_force_sensing_experiment.m)；
- 状态解码和三维 Cosserat 射击：[decodeMapState](../rod/run_rod_plane_force_sensing_experiment.m)；
- 单接触约束：[fullMapConstraints](../rod/run_rod_plane_force_sensing_experiment.m)；
- 单接触输出字段：[estimate_sensor_forces.m](../rod/estimate_sensor_forces.m)；
- 单平面观测合同：[measurements_from_sensor_packet.m](../rod/measurements_from_sensor_packet.m)。

### 当前已有的“双接触”代码是什么

[solve_cosserat_multi_contact_map.m](../rod/solve_cosserat_multi_contact_map.m) 已经可以对多个点载荷做独立的 Cosserat 正向射击；[build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) 用它生成两个接触反力、接触点和末端力，[run_model_mismatch_protocol.m](../rod/run_model_mismatch_protocol.m) 再把**只有一个平面输入**交给当前单接触逆解。这一协议的目的，是测量“模型外时会失败多少”，不是声称已经完成双接触估计。

因此现有 `two-contact`、`curved-surface` 和 `friction-mismatch` 结果都必须标为 stress/mismatch：真值有两个反力，但估计输出仍只有 `contactForceResultant(:,t)` 和 `contactArcLength(t)`。如果只把 `truth.contactForce` 的两列相加后与单接触输出比较，会掩盖接触分解失败，不能作为双接触结果。

本阶段已经把多接触的公共 mechanics 接口补齐，但没有把它包装成“已经完成的双接触逆解”：

- `multi_contact_state_spec.m`：集中生成独立平面和共面两种状态布局。`K=1` 时状态长度仍是 `11+m=27`，与旧 API 完全一致；`K=2` 独立平面是 `19+2m=51`。
- `decode_multi_contact_state.m`：把每个接触的平面点、法向参数、弧长、法向力、摩擦锥系数和 `lambda` 解码为 `contactForce(3,K)`，并保留每个接触的法向/摩擦分量。
- `evaluate_multi_contact_state.m`：调用已有的 `solve_cosserat_multi_contact_map`，返回同一候选状态的 Cosserat 形状、稀疏曲率预测、接触点、总力和约束审计；它明确标注为 forward candidate，不会冒充 MAP 估计。
- `multi_contact_constraints.m`：统一检查每个接触的闭合间隙、整杆采样非穿透、法向力、摩擦锥、互补残差和弧长有序性。
- `test_multi_contact_state_spec.m`：检查 `K=1` 兼容性、`K=2` 独立/共面布局和三维力矩阵输出；`formulationStateSize` 现在直接复用该布局。

因此当前代码已经可以对一个 K 接触候选做真实多接触 forward 和物理审计；尚未完成的是“用 FBG + 多平面观测自动优化这个 K 接触状态”的逆向 MAP 包装器。这个边界在代码和结果中都保留，避免把 forward truth 或单接触 mismatch 误写成多接触成功。

### 真正双接触状态的定义

第一版应先做两个**已观测平面**上的点接触。每个接触具有独立的平面点和法向，状态可以写为：

\[
x_2=[p_{\pi,1},\eta_1,s_1,f_{n,1},\beta_1,\lambda_1,
      p_{\pi,2},\eta_2,s_2,f_{n,2},\beta_2,\lambda_2,f_e].
\]

当每个接触使用 `m` 条摩擦锥边时，维度是 `2(8+m)+3=19+2m`，默认 `m=16` 时为 51。若两个接触明确共面，可以共享一组 `[pπ,η]`，维度降为 `5+2(3+m)+3=14+2m`，但代码应把“共面”作为显式场景配置，不要用隐含的数组形状猜测。

每个接触的几何力学约束必须同时存在：

1. 弧长有序且不重合：`s1 + s_min < s2`；
2. 接触点由同一个 Cosserat 中心线计算：`pc,j = p(sj)`；
3. 非穿透和接触闭合：`g_j=n_jᵀ(pc,j-pπ,j)=0`，并对整根杆的采样间隙施加 `g(s)≥0`；
4. 法向力非负：`fn,j≥0`；
5. 每个接触各自满足摩擦锥互补：`β_j≥0`、`w_j≥0`、`β_j⊙w_j=0`，以及 `λ_j≥0`、`κ_j≥0`、`λ_jκ_j=0`；
6. 末端边界条件：无外加末端力矩时 `m(L)=0`；
7. 多个力跳变按弧长顺序进入平衡积分。`solve_cosserat_multi_contact_map` 已经实现了这一条正向积分逻辑，可作为逆解 mechanics kernel 的基础。

逆解目标应把观测、先验和所有接触约束放在同一个 MAP：

\[
J(x_2)=\|z^{fbg}-h^{fbg}(x_2)\|^2_{R_{fbg}^{-1}}
 +\|z^{env}-h^{env}(x_2)\|^2_{R_{env}^{-1}}
 +\|x_2-x_{2,k-1}\|^2_{Q^{-1}}
 +\rho_{\rm mpcc} \Phi(x_2).
\]

不能先用单接触估计器求一个接触，再把剩余力分给第二个接触；这样会把同一个形状解释两次，破坏力矩平衡和不确定性含义。

### 代码实施顺序

1. **先抽取通用状态布局。** 新建 `multi_contact_state_spec.m`，集中返回 `K`、`m`、每个接触的索引、末端力索引和共享/独立平面模式。单接触 `formulationStateSize` 可以调用该布局，避免两套索引漂移。
2. **扩展解码和 Cosserat map。** 新建 `decodeMultiContactState.m` 和 `solve_cosserat_multi_contact_map` 的逆解包装器，返回 `contactForce(3,K)`、`normalForce(1,K)`、`frictionForce(3,K)`、`contactPoint(3,K)` 和每个接触的 mechanics residual。原有单接触字段继续保留为兼容 API，但双接触路径不能只覆盖写入 `contactForceResultant`。
3. **扩展观测合同。** `packet` 增加 `environmentPlanes` 或 `environmentPlanePointMm(3,K)`、`environmentPlaneNormal(3,K)`、`planeCovariance(6,6,K)` 和同步时间；`measurements_from_sensor_packet` 必须检查每个平面协方差正定、法向归一化、时间戳一致以及平面数量与 `K` 相等。
4. **扩展约束。** 新建 `multi_contact_constraints.m`，逐接触生成 gap、法向、摩擦锥和接触运动互补，再追加 `s` 的有序约束和整杆 sampled collision constraints。MPCC 连续化、精修和最终质量检查都要调用同一约束函数。
5. **扩展初始化。** 从环境距离场找到两个候选弧长，使用 shape-only 载荷 seed 分解成两组接触力；若候选间距不足或局部法向不可分辨，明确返回 `requiresReview`，不能随机分配一半力。
6. **先做双平面静力学，再做粘滑。** 第一版实验固定两个平面、无噪声、无摩擦，验证每个接触力和接触位置；第二版加入独立 `μ_j` 和摩擦方向；第三版再加入平面观测噪声、历史窗口和模式切换。
7. **公平基线和可辨识性。** shape-only、Gaussian 和单接触 EnFiRCE 都使用同一稀疏 FBG 与环境输入；双接触方法额外报告每个接触的 force/location RMSE、接触漏检/误检率、总力误差、力矩残差、整杆最小间隙、review rate 和单帧时间。两个接触的局部灵敏度矩阵应分别做 rank/condition 检查。

### 最小可复现实验矩阵

| 场景 | 接触几何 | 观测 | 目的 |
|---|---|---|---|
| `parallel_two_contact` | 同一平面，`s1≈120 mm, s2≈168 mm` | 两个平面观测或一个共面观测 | 检查两个反力是否能从同一形状分开 |
| `corner_two_contact` | 两个法向不平行的平面 | 两个平面点、法向和协方差 | 检查几何信息是否消除分解退化 |
| `two_contact_sliding` | 两个接触各自有 `μ_j` | 连续时间窗口 | 检查粘滑和接触模式切换 |
| `two_contact_mismatch` | 真值两接触，输入只给一个平面 | 单接触 API | 作为明确的 out-of-model 对照，不能算成功结果 |

其中第一项应复用当前 [build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) 的独立正向真值，但在发布双接触数字前必须增加每个平面的 gap 检查、接触法向检查和两接触力矩残差检查。当前 `run_model_mismatch_protocol` 只完成最后一项压力测试，尚未实现前三项的逆解。

### 何时可以把它写进论文

只有当双接触逆解输出每个接触的独立力和位置、在独立真值上通过整杆非穿透与平衡审计，并在相同观测输入下完成单接触/shape-only/Gaussian 基线后，才能在论文中写“multi-contact estimation”。在此之前，准确表述是：**当前 EnFiRCE 是单接触环境反力与末端力分解器，并包含独立双接触真值的模型失配压力测试。**
