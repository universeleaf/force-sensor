# EnFiRCE：Environment- and Friction-informed Rod Contact Estimation 技术总说明

这份文档是当前仓库的实现手册，按“研究问题 → 数据边界 → 正向真值 → 逆问题 → 数值求解 → 结果审计”的顺序解释代码。每一节给出相应函数，便于直接跳到实现。最新的完整三维多接触/时间窗口路径及其全部代码对应见[完整 formulation 工作流](FORMULATION_WORKFLOW.md)。场景和视频另见[场景矩阵](SCENARIO_MATRIX.md)，多接触配对实验见[软件实验报告](SOFTWARE_BENCHMARK_2026-09-30.md)，实验状态另见[状态页](STATUS.md)。若历史文字与代码冲突，以当前代码及相应运行目录中的 comparison.json 为准。

**2026-09-30 实现更正：** 本轮发现旧三维多接触射击函数的力矩导数符号错误，以及最近节点读取固有曲率可能移动突变位置的问题，均已修复。因此旧 `model_mismatch` 三个结果暂不作为正确物理实验引用。独立二维多接触 demo/396 次 benchmark 和既有单接触射击不受这两个错误影响。旧时间窗口 penalty smoother 已被每帧完整 Cosserat/MPCC 的联合窗口替代；下文旧版本描述作为历史解释保留，以新工作流文档为准。

**研究目标**：利用连续体机器人的稀疏形状信息和环境几何信息，估计杆身接触位置、接触力与独立末端外力，并识别观测不足或模型失配。当前是 MATLAB 仿真研究原型；真实 FBG、相机、力传感器和同步系统尚未接入，数值结果不能解释为实物精度。

## 1. 阅读地图与职责边界

| 层 | 主要代码 | 输入 → 输出 |
|---|---|---|
| 场景调度 | [force.m](../force.m) | 场景名 → 对应实验协议 |
| 公共逆解 API | [estimate_sensor_forces.m](../rod/estimate_sensor_forces.m) | sensorInput → 力估计与质量标志 |
| 论文主路径配置 | [estimate_formulation_forces.m](../rod/estimate_formulation_forces.m)、[formulation_solver_config.m](../rod/formulation_solver_config.m) | 同一 sensorInput → 非线性 Cosserat + 完整 MPCC |
| 数据边界 | [measurements_from_sensor_packet.m](../rod/measurements_from_sensor_packet.m) | 稀疏传感器包 → 重建形状、环境观测、历史 |
| 运动学重建 | [reconstruct_sensor_curvature.m](../rod/reconstruct_sensor_curvature.m)、[integrate_curvature_field.m](../rod/integrate_curvature_field.m) | 稀疏曲率 + 本征曲率 + 基座 → 中心线 |
| 当前力学 | [solve_cosserat_force_map.m](../rod/solve_cosserat_force_map.m) | 接触位置、接触力、末端力 → 平衡杆形 |
| 单帧 MAP | [run_rod_plane_force_sensing_experiment.m](../rod/run_rod_plane_force_sensing_experiment.m) 中的 estimateForcesWithShapeAndEnvironment、solveNonlinearFormulationMap | 观测和先验 → 状态向量 |
| 接触优化 | [solve_contact_mpcc.m](../rod/solve_contact_mpcc.m)、[solve_contact_mode_map.m](../rod/solve_contact_mode_map.m) | MAP 目标 + 接触约束 → 可行解和同伦轨迹 |
| 全杆几何 | [plane_contact_constraints.m](../rod/plane_contact_constraints.m) | 连续力学采样形状 → 非穿透与切触残差 |
| 时间历史 | [latent_fbg_history.m](../rod/latent_fbg_history.m)、[estimate_temporal_window_forces.m](../rod/estimate_temporal_window_forces.m) | 前一帧观测或 W 帧 → 联合估计 |
| 完整三维多接触/窗口 | [estimate_formulation_window.m](../rod/estimate_formulation_window.m)、[run_formulation_workflow.m](../rod/run_formulation_workflow.m) | 稀疏形状、多平面、协方差 → 自动候选、全窗口物理约束、逐接触力、质量和可追踪文件 |
| 独立真值/demo | [build_contact_demo_truth.m](../rod/build_contact_demo_truth.m)、[contact_demo_scenes.m](../rod/contact_demo_scenes.m)、[run_contact_demo_suite.m](../rod/run_contact_demo_suite.m)、[render_contact_demo_video.m](../rod/render_contact_demo_video.m) | 场景 → 独立平衡真值、模拟传感器包、逐状态 MAP/MPCC 评分和 MATLAB 连续视频 |
| 压力与基线 | [run_model_mismatch_protocol.m](../rod/run_model_mismatch_protocol.m)、[run_fair_baseline_protocol.m](../rod/run_fair_baseline_protocol.m) | 相同或模型外输入 → 对照结果 |
| 多接触配对实验 | [run_multi_contact_benchmark.m](../rod/run_multi_contact_benchmark.m)、[run_multi_contact_plane_uncertainty.m](../rod/run_multi_contact_plane_uncertainty.m) | 同一曲率包 → 环境/形状基线、几何消融和标定误差 |
| 工程回归 | [run_project_checks.m](../rod/run_project_checks.m) | 当前 31 项基础检查、可选 2 项归档重放 → project_checks.json |

主数据流：

~~~text
场景/真实设备（目前只有仿真）
  → 稀疏 FBG + 当前/历史基座位姿 + 环境平面及协方差
  → sensorInput.tube / packet / config
  → measurements_from_sensor_packet
  → 曲率重建与接触/末端力初值
  → 对候选状态反复求解三维 Cosserat 平衡
  → 观测 MAP + 杆身非穿透 + 摩擦互补的 MPCC
  → 接触力、末端力、合力、接触位置、残差和 review 标志
  → 只在离线评分阶段与独立真值比较
~~~

公共入口明确拒绝 sensorInput 中的 forward、truth、results 字段；评分文件可以同时保存 sensorInput 与 truth，但传给估计器的只能是 sensorInput。这个边界由 estimate_sensor_forces.m 的入口断言及 [test_sensor_contracts.m](../rod/test_sensor_contracts.m) 检查。

## 2. 坐标、单位与模型假设

所有长度用 **mm**，曲率用 **1/mm**，力用 **N**，力矩用 **N·mm**，弯曲/扭转刚度用 **N·mm²**，时间用 **s**。世界坐标是三维右手坐标；基座 T_base 是 4×4 齐次位姿，左上 3×3 是旋转，右上 3×1 是位置。杆弧长 s 从 0 到 L，p(s) 为中心线，R(s) 为截面姿态，u(s) 为局部曲率，uhat(s) 为卸载本征曲率。

环境由平面点 p1 和单位法向 n 描述。带符号间隙定义为 g=n'·(p_contact−p1)。g>0 是可行侧，g=0 是接触，g<0 是穿透；世界坐标的 x/z 方向本身没有固定的“接触法向”意义，取决于场景配置。

历史单接触逆解使用一个无限平面和一个零半径点接触；新完整窗口使用多个观测半空间和形状驱动的多个接触槽。共同假设是静态/准静态、不可伸长不可剪切杆，没有惯性、阻尼、分布载荷、有限杆半径和动态黏滑转换。原六个连续 demo 的真值来自二维 XZ 平衡再刚体旋转；新空间滑动输入另外由三维射击与接触根求解生成面外载荷，详见[新工作流](FORMULATION_WORKFLOW.md)。

## 3. 输入数据的逐字段契约

公共输入必须是一个结构体，包含 tube、packet、config，入口见 [estimate_sensor_forces.m](../rod/estimate_sensor_forces.m)。[sensor_estimator_config.m](../rod/sensor_estimator_config.m) 从场景配置中只复制估计器和传感器参数，不把仿真载荷复制到公共输入。

### 3.1 校准杆 tube

| 字段 | 维度及检查 | 物理作用 |
|---|---|---|
| tube.s | 1×N，首点为 0，严格递增、有限 | 弧长网格 |
| tube.uhat | 3×N，有限 | 卸载曲率和扭转 |
| tube.T_base | 4×4 位姿 | 默认基座姿态；逐帧由 packet.basePose 更新 |
| tube.kb、tube.kt 或 getTubeK 所支持的刚度 | 转为 3×N，严格为正 | 本构 K(s) |

[make_experiment_tube.m](../rod/make_experiment_tube.m) 优先读取自定义 cfg.rod.sMm 和 cfg.rod.intrinsicCurvaturePerMm；否则通过 CreatTube 构造默认杆。这里的 uhat/K 是标定模型，不是估计过程偷看的真值力。

### 3.2 传感器包 packet

验证和转换均集中在 [measurements_from_sensor_packet.m](../rod/measurements_from_sensor_packet.m)。必需字段：

| 字段 | 尺寸或范围 | 含义 |
|---|---|---|
| schemaVersion | 标量，当前必须为 1 | 数据协议版本 |
| fbgIdx | 有序整数，至少两点，必须含杆首尾节点 | 采样节点 |
| sFbgMm | 与 fbgIdx 等长，必须匹配 tube.s(fbgIdx) | FBG 弧长标定 |
| curvaturePerMm | 3×nFBG×nt | 当前稀疏曲率 |
| previousCurvaturePerMm | 同尺寸 | 前一采样时刻稀疏曲率 |
| basePose、previousBasePose | 4×4×nt | 当前和历史基座位姿 |
| timeSeconds、previousTimeSeconds | 各 nt 个，当前时间递增且大于对应历史时间 | 观测时钟 |
| processReferencePeriodSeconds | 正标量 | 过程先验参考周期 |
| frictionMu | nt 个非负值 | 每帧摩擦系数 |
| actuationMm | nt 个 | 执行器/推进位移 |
| planePointMm、planeNormal | 各 3×nt，法向为单位向量 | 环境观测 |

可选 planeCovariance 为 6×6×nt，对应 [平面点 3 维；法向 3 维]，必须对称正定。代码以 Cholesky 分解得到 environmentWhitening；正定的模型误差底噪应由生成包或标定环节显式提供。可选 environmentTimeSeconds 要在声明的最大偏差内与当前 FBG 时间同步，默认上限是 5 ms。若相同时间戳的曲率或基座位姿互相冲突，函数报 rod:ConflictingSensorSample，不会把它们当成两次独立观测。

### 3.3 转换后 measurements 的具体内容

每帧分别重建当前与历史曲率和中心线，并输出 m.uSparse、m.u、m.p、m.pSparse、m.previousShape、m.planePointMeasured、m.planeNormalMeasured、m.baseTraj、m.frictionMu、m.timeSeconds、m.historyIntervalSeconds、m.processStepCount。存在环境协方差时另有 m.environmentWhitening。pSparse 只提取同一 FBG 子集对应的位置，供 shape-only/Gaussian 基线公平使用。m.previousShape 来自前一时刻的**稀疏观测重建**，不是密集仿真形状。

合成 packet 的具体生成见 [simulate_sensor_packet.m](../rod/simulate_sensor_packet.m)：默认抽 24 个 FBG 节点，采用固定随机种子向前后观测加曲率噪声，抽取当前/历史基座及平面观测；密集 p/R 和真实载荷均不跨越传感器边界。当前合成 FBG 实际测量前两个弯曲分量，第三曲率分量被置为本征扭转，这是平面真值上的建模假设，不能说成有第三个真实测量通道。

## 4. 从稀疏曲率到可用杆形

[reconstruct_sensor_curvature.m](../rod/reconstruct_sensor_curvature.m) 先验证基座旋转正交、行列式为 1。主路径使用 intrinsic-delta：在每个 FBG 点计算 delta_u=u_measured−uhat，用 PCHIP 按弧长插值 delta_u 的三个分量，再把完整网格上的 uhat 加回。这样保留弯钩的已校准本征形状，不把它解释成外力。可选 shapeSmoothing>0 时使用 movmean，默认 0。历史 absolute 模式直接插值绝对曲率。

当 mechanicsModel='cosserat-shooting' 时，[integrate_curvature_field.m](../rod/integrate_curvature_field.m) 用 SE(3) 指数中点积分：

~~~text
elastic = u−uhat
omega_j = [uhat_j + (elastic_j+elastic_(j+1))/2] · ds
W = hat(omega_j), a = ||omega_j||
R_(j+1) = R_j [I + A W + B W²]
p_(j+1) = p_j + R_j [I + B W + C W²] [0,0,ds]'
A=sin(a)/a, B=(1−cos(a))/a², C=(a−sin(a))/a³
~~~

小角度时用泰勒展开；本征曲率的分段突变保持原样，只对弹性曲率取中点。输出 shape.u/R/p/T_base/sparseU/fbgIdx。这是观测的**运动学重建**，本身尚未施加该帧的 Cosserat 力平衡。旧 mechanicsModel 则使用 solveShape，保留历史比较。

## 5. 当前三维 Cosserat 力学映射

实现：[solve_cosserat_force_map.m](../rod/solve_cosserat_force_map.m)。给定 contactS、三维接触力 fc、三维末端力 fe 和 tube，它对**当前候选状态**求静态平衡杆形：

~~~text
p'(s) = R(s)e3
R'(s) = R(s)hat(u(s))
m'(s) = −p'(s) × n(s)
u(s)  = uhat(s) + K(s)⁻¹ R(s)'m(s)
n(s)  = fe+fc (接触之前)，fe (接触之后)
末端自由边界：m(L)=0
~~~

力在精确 contactS 处跳变；代码同时在本征曲率或刚度的断点分段积分。未知的三个基座力矩分量由 fsolve 射击，使末端力矩满足默认 2e-4 N·mm 容限；每一段用 ode45，默认相对容限 2e-7。初值来自卸载杆形上的两个外力力臂。射击收敛是**一个局部平衡根**，不证明唯一或稳定。

输入边界会拒绝非有限/非递增弧长、错误尺寸或非正刚度、越界接触弧长、非有限三维力。一次力学调用的 RHS 工作量上限默认 50000；超限抛 rod:CosseratEquilibrium，优化器拒绝该试探，不返回截断形状。

输出包括连续杆形 u/R/p、精确接触点 pc、接触切向 contactTangent、沿杆力矩 momentNmm、碰撞检查网格 collisionS/collisionP、末端力矩残差、基座射击力矩、RHS 调用数。固定碰撞网格还含原杆节点；间隔配置默认 0.5 mm。持久缓存使用完整 tube/基座/选项和精确 [contactS;fc;fe] 作为键，不用近似键或评分真值。

## 6. 逆解状态向量：每个变量对应什么

主循环在 [run_rod_plane_force_sensing_experiment.m](../rod/run_rod_plane_force_sensing_experiment.m) 的 estimateForcesWithShapeAndEnvironment；状态大小由 formulationStateSize 决定，解码在 decodeMapState。设 m 为摩擦锥生成方向数，默认 m=16，总维度 11+m=27：

~~~text
x = [p1(3); eta1(2); s1; fn; beta1(m); lambda1; fe(3)]
~~~

| MATLAB 索引 | 符号 | 单位 | 作用及约束 |
|---|---|---:|---|
| 1:3 | p1 | mm | 平面参考点；被环境观测和先验约束 |
| 4:5 | eta1 | rad | 相对测量法向的两个局部参数；etaToNormal 保持法向单位化 |
| 6 | s1 | mm | 单一接触弧长，限在 tube.s 首末之间 |
| 7 | fn | N | 非负法向接触反力 |
| 8:7+m | beta | N | m 个非负切向摩擦生成元 |
| 8+m | lambda1 | mm | 摩擦运动学互补辅助变量，非负 |
| 9+m:11+m | fe | N | 独立三维末端外力 |

etaToNormal 使用测量法向附近的稳定切平面基将二维参数映射到单位球；frictionDirections 则在该法向切平面上生成 m 条方向 D。当前接触合力 fc=n·fn+D·beta；总外力为 fc+fe。若 mu=0，MPCC 把 beta 固定为零。stateBounds 限制 eta、s、非负变量；useForceBounds 默认为 false，研究主路径依赖观测/先验而非硬力上界，若显式开启才应用配置的力上界。

接触位置 pc 并不是 p1：它来自当前 Cosserat 形状在 s1 处的中心线点。p1 定义环境平面，pc 是机器人上的候选接触点，两者的法向差才是 gap。

## 7. 几何、摩擦与互补约束的确切形式

### 7.1 点接触与整杆非穿透

decodeMapState 计算 gap=n'·(pc−p1)。[plane_contact_constraints.m](../rod/plane_contact_constraints.m) 在 planeContactGeometry='rod' 时还对力学输出的 collisionP 逐点计算 gap_j=n'·(collisionP_j−p1)，要求 gap_j≥0。对内部接触点加入加权切触：

~~~text
a=(s1−s_min)/(s_max−s_min)
fn · 4a(1−a) · n'·contactTangent = 0
~~~

力为零或接触位于端点时该等式不限制切向；内部有正反力的光滑接触要求中心线切向与平面相切。约束来自**连续 ODE 插值后的固定离散采样点**，不是整条曲线无穿透的解析证明，也没有杆半径补偿。

### 7.2 多面体 Coulomb 摩擦锥

frictionDirections 用稳定的平面切向基 B，按 2π/m 在切平面布置生成方向 D_i。令 beta_i≥0，则

~~~text
fc = n·fn + Σ D_i·beta_i
fn ≥ 0, beta_i ≥ 0
coneSlack = mu·fn−Σ beta_i ≥ 0.
~~~

当前/历史接触位置按同一接触弧长比较，切向位移 v_t=(I−nn')·(pc_current−pc_previous)。历史点可使用线性插值或 integrated 选项；Formulation 配置 integrated。每个摩擦生成方向的运动学变量 w_i=D_i'v_t+lambda。完整互补对为：

~~~text
0 ≤ gap       ⟂ fn        ≥ 0
0 ≤ w_i       ⟂ beta_i    ≥ 0  (每个 i)
0 ≤ lambda    ⟂ coneSlack ≥ 0
~~~

其中 a⟂b 意味着 a·b=0。对应代码：fullMapConstraints 形成不等式和乘积等式；solve_contact_mpcc 使用有物理单位缩放的 [gap;w;lambda] 与 [fn;beta;coneSlack]。摩擦运动学描述的是成对样本的**位移**，不是连续时间速度模型。

### 7.3 模式诊断而非模式真值输入

历史 active-set 路径的 selectComplementarityMode 根据可观察的接触间隙、前后切向位移、配对噪声和摩擦方向分辨率选择 no-contact、frictionless、sticking、sliding 或 unresolved。模式噪声协方差由 [paired_contact_uncertainty.m](../rod/paired_contact_uncertainty.m) 与 [friction_mode_resolution.m](../rod/friction_mode_resolution.m) 处理；后者要求候选方向的最小工作量优势超过噪声椭球的三倍标准差边界。

主路径的 Scholtes MPCC **不预先指定接触模式**。先优化全部互补变量，再根据解读出 no-contact/sticking/sliding 等标签，并允许 [solve_contact_mode_map.m](../rod/solve_contact_mode_map.m) 做有验收条件的局部 polishing。标签是结果诊断，不是真值输入。

## 8. 观测模型、MAP 目标与先验

### 8.1 观测向量和白化

measurementVectorForFrame 从 m.uSparse 选择 cfg.forceSensor.curvatureObservedAxes，按 MATLAB 列主序展平，随后拼接测量平面点和测量法向。Formulation 配置为 [1 2]，24 个 FBG 产生 48 个曲率观测，再加 6 个环境观测，共 54 维。第三曲率分量仍可用于形状重建，但不作为主路径观测似然通道。若包携带 planeCovariance，环境 6 维先通过 Cholesky 的 L⁻¹ 白化；mapMeasurementModel 对预测环境量使用相同变换，避免只白化观测。

预测 h(x) 来自当前解码状态：先调用当前三维 Cosserat 射击，抽取 FBG 节点预测曲率，再拼接状态的 p1 和 n。详细入口是 run_rod_plane_force_sensing_experiment.m 内的 mapMeasurementModel、decodeMapState、measurementVectorForFrame、measurementStdVector。曲率噪声/模型底噪由 [calibrate_fbg_likelihood.m](../rod/calibrate_fbg_likelihood.m) 与配置的 measurementStd 控制；仿真可注入零噪声，但 MAP 方差不能随意当作零。

### 8.2 完整非线性目标

内部 fullMapCost 与 solveNonlinearFormulationMap 的 nonlinearObjective 实现：

~~~text
J(x) = 1/2 (x−x_prior)' P_minus⁻¹ (x−x_prior)
     + 1/2 Σ_i [z_i−h_i(x)]² / Rdiag_i.
~~~

x_prior 为上一帧状态和随机游走过程先验；第一帧由 measured plane、测量形状的最近间隙、零接触力先验及 tip-only 力种子初始化。P_minus 来自 priorStd 和 processStd；不同帧的过程协方差按实际采样间隔相对 processReferencePeriodSeconds 缩放。先验不是“真实接触力”。默认标准差逐字段列在同文件的 defaultExperimentConfig：例如初始末端力 4 N、接触弧长 8 mm；过程末端力 0.75 N、接触弧长 2 mm；配置修改会改变 MAP 的定量结果，复现必须保存配置。

initializeMapState 先用测量杆形对平面求 signed gap，在 contactSearchBandMm 内寻找最近接触弧长；seedTipOnlyForceFromMeasuredCurvature 用 K·(u−uhat) 与 tip Jacobian 解带小 ridge 的最小二乘末端力种子。solveReducedShapeKnownMapUpdate 和 refine_cosserat_shape_seed 提供额外初值；它们只决定从何处开始优化，不替代最终完整 MAP。

### 8.3 历史线性化路径与当前主路径

默认历史配置为 predecessor-linearized、point-only、active-set、mode-reduced。solveConstrainedMapUpdate 在每轮用 finiteDifferenceMeasurementJacobian 计算 H，将 h(x) 线性化为 h0+H(x−x_lin)，调用 solveLinearizedConstrainedMapSubproblem，再通过 acceptDampedMapStep 验收非线性代价。该路径保留旧结果可复现性，但不是当前完整三维主张。

[formulation_solver_config.m](../rod/formulation_solver_config.m) 将主路径切换为：

~~~text
mechanicsModel='cosserat-shooting'
planeContactGeometry='rod', planeCollisionStepMm=0.5
historyPointInterpolation='integrated'
curvatureObservedAxes=[1 2]
complementaritySolver='scholtes'
subproblemCoordinates='full', solver='fmincon'
useNonlinearShapeSeed=true, useObservationScaling=true
allowApproximateFallback=false, useMultiStart=false
mpccRelaxations=[1e-2 1e-4 1e-6 1e-8]
mechanicsRelativeTolerance=2e-7
mechanicsMomentToleranceNmm=2e-4
~~~

主路径 solveNonlinearFormulationMap 直接优化上述非线性 J(x)。nonlinearSolverScale 用每个状态坐标的局部信息列 H'R⁻¹H 与先验精度设置对角坐标尺度；这**只改变求解坐标**，不改变物理目标。finiteDifferenceMeasurementJacobian 对一个坐标单独扰动，不顺便把其他力变量投影到锥上，避免求导方向被投影污染。

## 9. Scholtes MPCC 的数值执行细节

[solve_contact_mpcc.m](../rod/solve_contact_mpcc.m) 定义物理量缩放后的互补配对 a=[gap/lengthScale; w/lengthScale; lambda/lengthScale]、b=[fn/forceScale; beta/forceScale; coneSlack/forceScale]。每一级 tau 解：

~~~text
min J(x)
subject to a≥0, b≥0, a_i b_i≤tau
           整杆采样非穿透
           |内部切触残差|≤tau
           状态边界
~~~

默认 tau 从 1e-2、1e-4、1e-6 到 1e-8；每一级都用 MATLAB fmincon 的 SQP 求解并把上一阶段解传给下一阶段。有限 tau 是数值松弛，**不等于精确互补**。最终记录每级退出码、迭代数、目标、约束违反、耗时和被拒绝的力学试探次数。随后根据完整 MPCC 解推断活跃摩擦方向，调用 solve_contact_mode_map polishing；只有最终约束和代价符合验收条件才采用 polishing 结果，否则保留同伦解。

如果一次 ODE/射击试探抛 rod:CosseratEquilibrium，优化器把该试探记为不可行或无限目标；其他异常继续抛出，避免程序 bug 被误认为正常力学不可行。代码默认禁用近似 fallback 和多初值搜索；因此一个收敛解既不证明全局最优，也不证明全局可辨识。

## 10. 历史信息与时间窗口

固定历史模式：当前 packet 的 previousCurvaturePerMm 经 reconstruct_sensor_curvature 生成 previousShape；当前解的接触点相对上一形状的位移用于摩擦运动学，但上一形状本身不随当前优化变量变化。该条件化近似没有把历史观测误差完整传进当前力协方差。

latent-FBG 模式：[latent_fbg_history.m](../rod/latent_fbg_history.m) 把所有已观测的前一时刻 FBG 通道写成 q=(u_previous_latent−u_previous_measured)/sigma_history，并在目标中加 q'q/2；当前状态与 q 一起进入完整 MPCC。第三个未观测扭转仍由本征扭转假设给出。该模式只联立一对时刻的历史运动学，不是前一时刻力平衡的完整联合后验。

短时间窗口：[estimate_temporal_window_forces.m](../rod/estimate_temporal_window_forces.m) 现在直接调用完整联合逆解。每个实际观测时刻有独立三维平衡状态，原始 FBG/环境观测只计入一次，物理状态带时间缩放的随机游走先验；末端力矩和完整摩擦 MPCC 是非线性约束。schema 1 中输出可能跳过采样时刻，适配器会合并所有当前/前驱时间，并使用每个当前样本明确记录的前驱，不以“上一视频帧”替代它。旧逐帧后验加原始观测的 penalty smoother 已移除，避免重复计算信息。详细状态、约束和输出见[完整工作流](FORMULATION_WORKFLOW.md)。

## 11. 逐帧主循环：从观测到 output.ours

estimateForcesWithShapeAndEnvironment 是 run_rod_plane_force_sensing_experiment 的核心内部函数。每帧执行以下顺序：

1. 读取当前 measurements、上一帧的状态先验和当前基座位姿。
2. 通过 configForMeasurementFrame 选择当前摩擦系数、环境协方差、历史间隔和当前观测通道。
3. 生成 z、观测方差 Rdiag、过程先验 Pminus。
4. 调用 initializeMapState；必要时调用 shape-only seed 或 nonlinear shape seed。
5. 选择历史 active-set 路径或 formulation Scholtes 路径。
6. 由 decodeMapState 重新求 Cosserat 形状，写入 state、p、u、contactPoint、contactArcLength、contactForce、normalForce、frictionForce、tipForce 和 totalForce。
7. 计算 measurementResidual、shapeRmseMm、gap、frictionConeSlack、三类互补残差、activeConstraintResidual、力敏感度和 solverTrace。
8. 计算 posteriorCovariance；若启用了 checkpoint directory，按帧原子写入 frame_XXX.mat。
9. 保存 frameSeconds 和 showProgress 日志。

最终 output.ours 的重要字段：

| 字段 | 内容 |
|---|---|
| state | 每帧 27 维状态 |
| p、u | 估计中心线和曲率 |
| contactForceResultant、normalForceResultant、frictionForceResultant | 接触合力及法/切分量 |
| tipForce、totalForceResultant | 末端力和合力 |
| contactArcLength、contactPoint | 接触位置 |
| complementarityMode、modeResolution | 模式标签和摩擦方向诊断 |
| shapeRmseMm、measurementResidualNorm | 形状/观测误差 |
| gap、frictionConeSlack、normalComplementarity、frictionComplementarity、coneComplementarity | 物理可行性诊断 |
| solverTrace、frameSeconds | 每轮求解信息和耗时 |
| posteriorCovariance | 当前分支局部线性协方差 |
| mechanicsDiagnostics | 射击、碰撞采样、力矩残差、RHS 次数 |
| forceSensitivity | 接触与末端力局部可分离性 |
| historyEstimate | latent-FBG 历史（如启用） |
| uncertaintyScope | 协方差适用范围的文字说明 |

output.quality 再把这些字段压缩成机器可读的 review 判断：

~~~text
numericalChecksPassed
frictionKinematicsEnforced
frictionKinematicsConsistent
frictionDirectionResolutionAssessed
frictionDirectionResolved
hasFrictionObservationWarning
rodGeometryEnforced
hasRodPenetration
forceComponentsLocallySeparable
hasForceSeparationWarning
hasOptimizationWarning
requiresReview
forceUncertaintyCoverageValidated=false
forceAccuracyCertified=false
~~~

requiresReview 是数值/可辨识性警告的合取，不是“这个力一定错”。相反，requiresReview=false 也只表示当前输入、模型和局部求解检查通过，不能替代独立力传感器真值。

## 12. 正向场景、独立真值和传感器边界

### 12.1 旧 wall/video 流程

run_rod_plane_force_sensing_experiment 的 defaultExperimentConfig 设置默认 150 mm 杆、基座、平面、mu、push-slide、24 个 FBG、0.02 s 采样周期和 MAP 标准差。normalizeExperimentConfig 会单位化平面法向，把 slideDirection 投影到平面切空间，并填充 historyCurvatureStdPerMm 等字段。makeRodPlaneScenario 生成 tube/obstacle，runForwardRodPlaneSimulation 逐步推进并保存正向 p/u/R、接触、摩擦和时间相位。buildForwardBaseTrajectory 明确构造 push 阶段的法向推进和 slide 阶段的切向移动；simulateSensingMeasurements 只抽取稀疏传感器字段；estimateForcesWithShapeAndEnvironment 运行逆解。

createForceSensingVideo 使用 MATLAB VideoWriter('MPEG-4') 写旧式 forces.mp4。因此 out/video/forces.mp4 后半段向左移动不是相机漂移：该场景的顶面法向是 [0,0,-1]，先 +z 推进，再沿平面内 -x 滑动，以便激励摩擦方向和滑动互补。out/upward/forces.mp4 是 wall 场景整体旋转 90° 后的 +z 推进/短切向滑动；参数是代码定义的场景，不能宣称恢复了 fail.mp4 的真实标定。

### 12.2 六个独立 demo

[contact_demo_scenes.m](../rod/contact_demo_scenes.m) 的六个 ID 和参数：

| ID | 几何/环境 | 关键设置 |
|---|---|---|
| ceiling_hook | 200 mm、120 mm 直段、半径 30 mm 的弯钩顶面 | tipForceXZ=[1;-1]，mu=0，push=[10,16,20] |
| side_wall | ceiling_hook 绕 y 轴 90° | push=[12,17,21]，tipForceXZ=[0.4;-0.6] |
| inclined_plane | 法向相对杆倾斜 12° | 平面点和法向改变，push=[12,17,21] |
| long_soft_rod | 240 mm、直段 144 mm、半径 36 mm | 刚度 0.65，绕 y 轴 180° |
| sliding_clean | 与弯钩同类几何的平面滑动 | mu=0.3，指定每对历史样本 0.02 mm 切向位移 |
| sliding_noisy | sliding_clean 的同一真值 | 曲率噪声 5e-5 /mm，平面点噪声 0.1 mm |

[build_contact_demo_truth.m](../rod/build_contact_demo_truth.m) 不调用逆解来制造真值：它先用独立的 solve_planar_contact_shooting 在 XZ 平面求单接触平衡，检查自由端力矩、杆身非穿透、切触和单边力，再把二维 p/u/力通过世界旋转 Q 变换到三维。它生成当前状态和前一状态的平衡对，调用 simulate_sensor_packet 生成 packet，最后才调用 estimate_sensor_forces。这样 scoring 中的 truth 不会从逆解回流。

滑动 demo 是指定的静态滑动分支：无限平面上平行平移不改变准静态平衡，因此用一对平衡形状构造历史位移；它不是解决动态 stick-to-slip、惯性或速度摩擦。

### 12.3 多接触和非平面压力测试

[build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) 使用独立多接触 Cosserat 映射生成两个接触反力和一个末端力。公共 packet 只提供第一平面和单接触可表达的输入：

- two-contact：真实两个接触，逆解仍只有一个接触点；
- curved-surface：第二个局部平面与第一平面夹角约 24.6°，但逆解没有第二平面；
- friction-mismatch：真值的切向反力与 packet 的 mu 不一致。

[run_model_mismatch_protocol.m](../rod/run_model_mismatch_protocol.m) 把这些结果标成 out-of-model stress result；误差和 review 率用于界定模型边界，不能写成“多接触已解决”。

### 12.4 固定环境多接触 demo

为了区分“多接触力学是否能成立”和“通用三维摩擦 MAP 是否已经完成”，仓库另外提供了[固定环境多接触 demo](MULTI_CONTACT_DEMOS.md)。`solve_planar_multi_contact.m` 在固定平面上同时优化每个接触的弧长、法向力和基座弯矩；`integrate_planar_multi_contact.m` 按接触弧长把力跳变带入连续 Cosserat 平衡；`estimate_planar_multi_contact.m` 只读取杆参数、固定环境和 24 个稀疏曲率观测，估计每个接触的法向力/弧长及末端力；`audit_planar_multi_contact.m` 对每个平面闭合、整杆采样间隙、切触、力矩残差和接触顺序做统一检查。四个场景的目录名是 `s_channel_two_contact`、`tapered_channel_two_contact`、`serpentine_three_contact` 和 `serpentine_three_contact_noisy`，结果放在 `out/demos/multi_contact/<scene-id>/`。

这条路径是已知接触数量/顺序的平面无摩擦稀疏曲率逆解。前三个无噪声场景的接触力 RMSE 为 `9.668e-11`、`7.598e-11` 和 `1.267e-10 N`；带曲率噪声的三接触场景合力 RMSE 为 `0.6202 N`，12 帧中 1 帧被标记为 review。视频显示的是已保存状态，不会从视频重新估计力。该路径已经不是“只有 forward mechanics”，但仍不能替代三维摩擦锥 MPCC、未知接触模式边缘化或真实传感器验证。

新配对协议把上述前三种几何扩展为 3 个随机种子、3 个噪声等级、8/16/24 个曲率观测和两个基座状态。`estimate_planar_shape_only_point_loads.m` 对同一观测包拟合点力，不读环境；`run_multi_contact_benchmark.m` 记录两种方法、接触间隙/切触消融和 1 mm 平面错位，共 396 次估计。正确几何、标称噪声、24 个观测时接触力 RMSE 为 0.490 N，对照为 2.967 N；仅 8 个观测时环境方法升到 13.576 N。`run_multi_contact_plane_uncertainty.m` 用相同观测包检验标定误差，把错位平面视为精确时 RMSE 31.092 N，允许标准差 1 mm 的潜在平面偏移时为 1.341 N。完整定义和逐条件结果见[软件实验报告](SOFTWARE_BENCHMARK_2026-09-30.md)。

## 13. 结果文件、视频和 provenance

### 13.1 六 demo 的目录结构

run_contact_demo_suite 每次生成新的 UUID，并逐 case 原子更新比较文件：

~~~text
out/demos/
  comparison.json
  latest/                    # 稳定的人类可读发布别名
    <sceneId>/
      forces.mp4
  <runId>/
    comparison.json
    <sceneId>/
      input_and_truth.mat
      results.mat
      data.json
      forces.csv
      forces.mp4
      demo.mp4
~~~

`latest/<sceneId>` 是便于分享和查找的稳定路径；`<runId>/<sceneId>` 是不可变的运行归档。两者内容相同，comparison JSON 同时记录 `artifactFolder` 和 `sourceArtifactFolder`，因此改进目录命名不会牺牲结果 provenance。

input_and_truth.mat 只适合离线评分；results.mat 保存输入、truth、output、scene 和 report；data.json 是 viewer 可读的逐帧摘要；forces.csv 包含真值/估计接触力、末端力、合力、接触弧长、互补残差和 review。顶层 comparison.json 只指向最新完整运行，旧目录不会被估计器自动重用。

runRecord 保存 schemaVersion、UUID、UTC 时间、MATLAB 版本、平台和 source 文件 SHA-256。test_observation_scaled_map 现在也从当前 comparison.json 动态找到 inclined_plane artifact，不再硬编码旧 UUID。

### 13.2 MP4 的来源

旧 out/wall、out/upward、out/video、out/stage1 下的 forces.mp4 是 MATLAB 正向/逆向实验内部的 VideoWriter 输出。新六场景的 forces.mp4 也由 MATLAB VideoWriter 生成：每个场景先完成 12 个独立推进状态的真值、稀疏传感器包和三维 MAP/MPCC 逆解，再连续采样这些已保存状态写入视频。demo.mp4 是同一文件的兼容副本；Python 只生成 GIF/PNG/HTML，不再覆盖 MP4。视频不是硬件录像，也不把播放帧当成新增估计结果。

## 14. 当前运行结果与正确解读

当前六 demo 运行 ID 为 `5140876c-8e87-4da6-ac4d-c28113e12b88`，使用 MATLAB R2024a。每个场景完成 12 个独立求解状态；每个 `forces.mp4` 有 60 个播放帧，来源状态数仍为 12。JSON 中的逐场景指标：

| 场景 | 接触 RMSE (N) | 末端 RMSE (N) | 合力 RMSE (N) | 接触弧长 RMSE (mm) |
|---|---:|---:|---:|---:|
| ceiling_hook | 7.547e-7 | 6.509e-7 | 2.709e-7 | 5.144e-6 |
| side_wall | 1.320e-6 | 9.700e-7 | 6.107e-7 | 5.375e-6 |
| inclined_plane | 4.261e-6 | 2.345e-6 | 2.353e-6 | 1.487e-6 |
| long_soft_rod | 3.870e-7 | 3.356e-7 | 1.319e-7 | 5.654e-6 |
| sliding_clean | 5.212e-6 | 3.567e-6 | 2.109e-6 | 4.507e-6 |
| sliding_noisy | 8.793e-1 | 7.019e-1 | 2.992e-1 | 4.610e-2 |

前五个 case 使用与真值一致的近乎无噪声模型，主要说明数值实现能复原独立平衡；不能外推为真实传感器误差。`sliding_noisy` 的 12/12 帧都触发 `requiresReview` 和摩擦方向观测警告，虽然最终互补和整杆非穿透检查通过；其估计平面参考下的最大穿透约 0.122 mm。这是当前方法边界，不应写成噪声下成功。

旧三项 `model_mismatch` 分数撤出有效物理证据：其前向函数存在本轮已修复的力矩符号和曲率分段问题，而且原生成器只规定点载荷，没有求解所有固定表面的接触闭合与摩擦。因此不能把它称为完整双接触/曲面真值。真正的多接触平衡场景使用独立接触根求解，见[完整工作流](FORMULATION_WORKFLOW.md)与[固定环境多接触 demo](MULTI_CONTACT_DEMOS.md)。

### 14.1 多种子、多噪声

[run_submission_statistics.m](../rod/run_submission_statistics.m) 默认用场景 ceiling_hook、inclined_plane、sliding_noisy，种子 [11,22,33]，曲率噪声 [0,5e-5,1e-4] /mm，共 27 帧。它以 force sensitivity 输出的局部标准差计算

~~~text
hit = |force_error| <= 1.96 * conditionalNoiseStdN
~~~

报告中的 coverageCertified 固定为 false，因为这不是完整后验、接触模式混合、历史边缘化或真实噪声校准。

### 14.2 公平基线

[run_fair_baseline_protocol.m](../rod/run_fair_baseline_protocol.m) 从同一 demo artifact 读取相同 tube、稀疏 FBG、历史 packet、平面 packet 和噪声实现，给三种方法同一份 sensorInput：

1. EnFiRCE：读取形状、历史、环境和摩擦；
2. shape-only point-load：只用同一稀疏形状；
3. Aloi-style Gaussian：只用同一稀疏位置/形状拟合 Gaussian 载荷。

shape-only 的合力可能看起来较小，是分量互相补偿的结果，不能代替接触/末端分力指标。基线从不读取 truth；truth 只用于协议内部 scoring。

### 14.3 运行时间

[run_realtime_benchmark.m](../rod/run_realtime_benchmark.m) 重放相同 packet，统计 mean/median/p95/max 和 effectiveHz。当前公开测量约为 119 s/frame p95，而 packet 周期是 0.02 s；这不是实时系统。主要瓶颈是每个候选状态都可能触发 ode45 + fsolve + 有限差分 MAP Jacobian。

## 15. 质量、可辨识性和不确定性

output.quality 的字段在 [estimate_sensor_forces.m](../rod/estimate_sensor_forces.m) 末尾构造：

- finite、shapeConsistent、activeConstraintResidual、frictionConeViolation、gap；
- rodGeometryEnforced、minimumSampledRodGapMm、hasRodPenetration；
- frictionKinematicsEnforced、frictionKinematicsConsistent；
- frictionDirectionResolutionAssessed、frictionDirectionResolved、hasFrictionObservationWarning；
- forceComponentsLocallySeparable、hasForceSeparationWarning；
- hasOptimizationWarning、numericalChecksPassed、requiresReview；
- environmentCovarianceUsed、uncertaintyScope；
- forceUncertaintyCoverageValidated、forceAccuracyCertified。

其中 shapeConsistent 的当前阈值是 shapeRmseMm<0.5，active constraint 的统一阈值是 1e-3，gap 允许至多 −1e-5 mm 的数值误差，锥违反阈值是 1e-4。requiresReview 是这些警告的合取加上求解退出码和摩擦观测分辨率，不是准确率标签。

force_sensitivity_diagnostic / nonlinear_force_sensitivity 判断的是当前解附近接触力和末端力 Jacobian 的局部分离；不代表全局唯一。posteriorCovarianceFromLinearization 只计算当前分支的

~~~text
Pplus = pinv(Pminus^-1 + H' * R^-1 * H)
~~~

并对称化。当前没有模式混合边缘化、约束曲率后验、历史变量完整边缘化、仿真到真实的噪声校准，因此 coverageValidated 和 accuracyCertified 必须保留 false。

## 16. 工程检查、根因修复和代码边界

[run_project_checks.m](../rod/run_project_checks.m) 当前注册 31 项检查：输入契约、协方差/深度平面、曲率重建、摩擦方向、接触射击、ODE 工作量、整杆碰撞、旋转等变性、独立多接触、结果生命周期、传感器重放、语法和观测缩放 MAP。

本轮系统性调试发现的真实故障是 test_observation_scaled_map.m 仍硬编码旧 UUID 496dc953-eb47-499d-9e38-bd83a92dd98b；该目录已被新 demo 运行删除，因此 force('check') 的第 15 项失败。它是回归 fixture 失效，不是力学算法失败。修复后的测试：

1. 读取 out/demos/comparison.json；
2. 查找 completed=true 的 inclined_plane case；
3. 根据 case.artifactFolder 加载当前 results.mat；
4. 继续原有观测缩放断言。

其他已经在代码中落实的保护：

- Cosserat 输入拒绝非法弧长、曲率、刚度、接触位置和力；
- 超出 RHS 工作量预算时拒绝 trial，不返回截断杆形；
- 杆身非穿透检查贯穿 seed、MPCC 约束和最终 quality；
- packet 不允许 truth/forward 泄漏；
- planeCovariance 必须正定，且环境白化可追溯；
- 多接触真值使用独立 forward solve，不复制逆解；
- 结果目录记录状态、退出码、约束残差和源 SHA。

## 17. 当前缺口和 RA-L 路线

当前代码能支撑“仿真中的环境+形状物理力分解原型”，还不能支撑“已完成的 RA-L 系统”这一表述。缺口按优先级：

1. 真实 FBG、环境几何/深度、同步和独立接触力 ground truth；
2. 无接触、端点接触、真正三维斜平面；
3. 让逆解状态表达两个以上接触点和多个局部平面，而不是只做 mismatch 压力测试；
4. 分布载荷或 Gaussian 载荷的明确模型，并与 shape-only/Aloi/Ferguson 风格基线公平比较；
5. W=2/3 时间窗的大样本统计、失败恢复和速度；
6. 历史误差和接触模式混合后的校准区间；
7. 解析/自动微分 Jacobian、warm start、稀疏结构和降阶力学，以把两分钟级单帧降到可用频率；
8. 实物实验图、标定报告和公开数据/脚本。

推荐实施顺序：

~~~text
标定真实观测包
  → 三维斜平面/无接触/端点接触
  → 双接触状态和独立基线
  → W=2/3 时间窗统计
  → 模式混合与区间校准
  → 解析导数、warm start、降阶加速
  → 实物实验、论文图表和补充材料
~~~

## 18. 复现、审计和交接

MATLAB 根目录命令：

~~~matlab
addpath(genpath(pwd));
force('demos');
force('formulation');
force('depth');
force('statistics');
force('mismatch');
force('baselines');
force('realtime');
force('temporal-window');
force('check');
~~~

只跑修复的回归：

~~~matlab
addpath('rod');
test_observation_scaled_map;
~~~

审计当前 demo 与源代码是否匹配：

~~~matlab
report = jsondecode(fileread('out/demos/comparison.json'));
report.runRecord.runId
report.runRecord.matlabVersion
report.runRecord.source(1)
~~~

六个 demo 的 artifact 适合交接的文件是 comparison.json、每个 case 的 data.json、forces.csv 和 demo.mp4；input_and_truth.mat 只在需要复算评分时发送。不要把 MEMORY.md、tmp/、调试日志、过期 UUID 目录或只为本地浏览的 HTML 当作研究产物。

可以给学长的准确表述是：EnFiRCE 已经有稀疏形状+环境平面驱动的连续体机器人力分解仿真，主路径是三维 Cosserat 射击、杆身非穿透、离散 Coulomb 摩擦锥和 Scholtes MPCC/MAP；六个独立单接触场景、同输入基线，以及四个已知接触顺序的平面多接触场景可复现。多接触无噪声结果主要验证模型一致性，带曲率噪声场景会显式触发 review。真实传感器、未知模式/摩擦的三维多接触逆解、区间校准和实时性仍未完成。

## 19. 用最通俗的话说

FBG 只能告诉我们杆在各处弯了多少，不能单独告诉我们弯曲是“碰墙的反力”造成的，还是“末端被外力拉/推”造成的。环境观测告诉程序墙在哪里。程序猜一个接触点、一个法向力、一些摩擦力和一个末端力，把它们放进 Cosserat 杆方程，算出杆应该长成什么样；然后把算出的形状和 FBG 形状比较，同时要求杆不能穿墙、接触力不能把杆从墙上拉开、摩擦力不能超出摩擦锥、滑动方向要和摩擦方向相容。优化器不断改猜测，直到观测误差和物理约束都足够小。最后给出接触力、末端力、接触位置以及是否需要人工复核。

这就是项目的主旨：用**环境信息 + 形状信息 + 连续体力学**做可审计的接触力分解。当前已经是可复现的仿真原型；要成为完整 RA-L 工作，还必须补真实观测、多接触/三维环境、不确定性校准和可接受的计算时间。

## 20. 按文件逐项查找实现

本节把“概念”落到 MATLAB 文件、入口函数和数据边界。MATLAB 没有单独的类层；一个 `.m` 文件通常包含一个公开入口和若干私有局部函数。局部函数只能由同文件入口调用，不能从工作区直接调用。阅读代码时建议先看入口函数的第一百行，再沿下面的调用链进入局部函数。

### 20.1 调度、配置和公共入口

| 文件/函数 | 具体职责 | 关键输入/输出 |
|---|---|---|
| [force.m](../force.m) / `force` | 根目录命令分发器；给 scenario 加 `rod/` 路径，并将 `demos`、`formulation`、`check` 等名称映射到协议函数 | `scenario, quickMode` → 协议报告 |
| [run_rod_plane_force_sensing_experiment.m](../rod/run_rod_plane_force_sensing_experiment.m) / `run_rod_plane_force_sensing_experiment` | 旧 wall/video 主实验；同一文件还承载默认配置、正向模拟、逐帧逆解、统计和视频导出 | `quickMode, overrides` → `results` |
| 同文件 `defaultExperimentConfig` | 创建杆、平面、推入/滑动、FBG、噪声、先验、视频和输出目录的完整默认配置 | `rootDir, quickMode` → `cfg` |
| 同文件 `normalizeExperimentConfig` | 单位化法向、投影滑动方向、补默认字段并做范围检查 | `cfg` → 标准化 `cfg` |
| [formulation_solver_config.m](../rod/formulation_solver_config.m) / `formulation_solver_config` | 论文主路径开关：三维 Cosserat、整杆碰撞、Scholtes、完整状态、缩放和容差 | `cfg` → `cfg.forceSensor` |
| [sensor_estimator_config.m](../rod/sensor_estimator_config.m) / `sensor_estimator_config` | 从实验配置复制估计器允许读取的字段；设定 history mode、历史噪声和滑动分辨率 | `cfg` → 无 truth 的 `estimator` |
| [estimate_sensor_forces.m](../rod/estimate_sensor_forces.m) / `estimate_sensor_forces` | 公共传感器入口；拒绝 `truth/forward/results` 泄漏，调用 packet 转换和逆解，最后生成 `output.quality` | `sensorInput` → `output` |
| [estimate_formulation_forces.m](../rod/estimate_formulation_forces.m) / `estimate_formulation_forces` | 显式使用 `formulation_solver_config` 的论文主路径包装器 | `sensorInput` → `output` |
| [make_experiment_tube.m](../rod/make_experiment_tube.m) / `make_experiment_tube` | 读取 `rod.sMm`、`intrinsicCurvaturePerMm`、刚度或默认 CreatTube，生成检查过的 `tube` | `cfg` → `tube` |

`cfg` 中的字段可以按四层理解：`cfg.rod.*` 描述弧长、本征曲率和刚度；`cfg.sensing.*` 描述 FBG 抽样、插值和噪声；`cfg.forceSensor.*` 描述状态、求解器、先验和互补约束；`cfg.output/video.*` 只影响保存和展示。估计器只接收前三层中允许的传感器/模型字段，不能因为字段名存在就读取 forward truth。

### 20.2 数据包和形状重建

| 文件/函数 | 实现细节 |
|---|---|
| [measurements_from_sensor_packet.m](../rod/measurements_from_sensor_packet.m) / `measurements_from_sensor_packet` | 逐字段验证 schema、索引、时间、基座位姿、平面和协方差；对当前及 previous curvature 各调用一次重建；返回 `m`，并计算历史间隔与 process step count |
| [reconstruct_sensor_curvature.m](../rod/reconstruct_sensor_curvature.m) / `reconstruct_sensor_curvature` | 在 FBG 节点计算 `u−uhat`，按弧长 PCHIP 插值，选择 intrinsic-delta 或 absolute 模式，再调用 SE(3) 积分 |
| [integrate_curvature_field.m](../rod/integrate_curvature_field.m) / `integrate_curvature_field` | 逐段用中点指数映射更新 `R,p`；处理小角度极限；输出稠密 `R,p` 和稀疏索引位置 |
| [simulate_sensor_packet.m](../rod/simulate_sensor_packet.m) / `simulate_sensor_packet` | 只从 forward shape 抽取允许公开的 24 个 FBG 曲率、基座、平面和 frictionMu；按 seed 加噪声；可生成 previous packet |
| [subset_sensor_packet.m](../rod/subset_sensor_packet.m) | 只裁剪时间维并同步所有 current/previous 字段；供噪声协议和重放使用 |
| [attach_depth_environment.m](../rod/attach_depth_environment.m) | 将深度图估计的平面点、法向和协方差绑定到既有 packet，不改变曲率观测 |
| [plane_from_depth.m](../rod/plane_from_depth.m) | 对深度像素做相机反投影、鲁棒平面拟合和残差协方差估计；输出环境观测而非力真值 |

### 20.3 力学映射、几何和互补约束

| 文件/函数 | 实现细节 |
|---|---|
| [solve_cosserat_force_map.m](../rod/solve_cosserat_force_map.m) / `solve_cosserat_force_map` | 给定 `contactS, fc, fe`，在接触点精确切换内力，`fsolve` 求基座矩，`ode45` 分段积分；同时生成固定碰撞网格和末端力矩残差 |
| [plane_contact_constraints.m](../rod/plane_contact_constraints.m) / `plane_contact_constraints` | 将解码状态和 Cosserat collision samples 变成 `c≤0, ceq=0`；检查点接触 gap、整杆非穿透、正力内部切触 |
| [solve_contact_mpcc.m](../rod/solve_contact_mpcc.m) / `solve_contact_mpcc` | 对全状态 MAP 逐级执行 Scholtes `tau`；每级调用 `fmincon`，记录退出码、约束和耗时 |
| [solve_contact_mode_map.m](../rod/solve_contact_mode_map.m) / `solve_contact_mode_map` | 在 MPCC 解附近做可选 active-set polishing；验收不通过时返回同伦解 |
| [contact_tangent.m](../rod/contact_tangent.m) | 从法向构造确定性的切向基向量，保证摩擦方向可复现 |
| [friction_mode_resolution.m](../rod/friction_mode_resolution.m) | 用切向位移、方向矩阵、协方差和 `slipConfidenceSigma` 判断方向是否能被噪声分开 |
| [paired_contact_uncertainty.m](../rod/paired_contact_uncertainty.m) | 将 current/previous 形状、基座和曲率误差传播为同一接触弧长位移协方差 |
| [friction_direction_quality.m](../rod/friction_direction_quality.m) | 把摩擦运动学是否满足、方向是否分辨和观测警告压缩为质量字段 |
| [solve_cosserat_multi_contact_map.m](../rod/solve_cosserat_multi_contact_map.m) | 仅用于独立多接触 forward stress truth；公共单接触逆解不调用它 |
| [solve_planar_multi_contact.m](../rod/solve_planar_multi_contact.m) | 固定平面环境下同时求解多个接触位置、法向力和基座弯矩 |
| [integrate_planar_multi_contact.m](../rod/integrate_planar_multi_contact.m) | 按多个接触弧长积分连续平衡，并返回接触点与整杆形状 |
| [audit_planar_multi_contact.m](../rod/audit_planar_multi_contact.m) | 对固定环境多接触结果检查闭合、非穿透、切触、接触顺序和力矩 |
| [run_multi_contact_demo_suite.m](../rod/run_multi_contact_demo_suite.m) | 生成四个按场景名保存的多接触结果目录、CSV、JSON 和 MP4 |
| [multi_contact_state_spec.m](../rod/multi_contact_state_spec.m) | K 接触状态布局；K=1 保持 27 维旧 API，K=2 独立平面为 51 维 |
| [decode_multi_contact_state.m](../rod/decode_multi_contact_state.m) | 将 K 个接触的平面、弧长、法向/摩擦变量解码为 `contactForce(3,K)` |
| [evaluate_multi_contact_state.m](../rod/evaluate_multi_contact_state.m) | 调用多接触 Cosserat forward map 并返回候选曲率、接触点和约束审计；不执行逆向优化 |
| [multi_contact_constraints.m](../rod/multi_contact_constraints.m) | 多接触 gap、整杆 sampled collision、摩擦锥互补和弧长有序检查 |
| [solve_planar_contact_shooting.m](../rod/solve_planar_contact_shooting.m) | 六 demo 的独立二维单接触真值 shooting；将平衡、切触、单边力和端部力矩作为验收条件 |
| [solve_planar_energy_rod.m](../rod/solve_planar_energy_rod.m) | 旧二维能量杆基线；用于历史协议和与 Cosserat shooting 的交叉检查 |

### 20.4 逆解文件内部的实际调用链

论文主路径每帧的函数级调用顺序如下；这比只看文件名更接近调试时的执行栈：

~~~text
estimate_sensor_forces
  -> measurements_from_sensor_packet
      -> reconstruct_sensor_curvature (current + previous)
          -> integrate_curvature_field
  -> run_rod_plane_force_sensing_experiment / estimateForcesWithShapeAndEnvironment
      -> measurementVectorForFrame, measurementStdVector
      -> initializeMapState
          -> measuredShapeForFrame
          -> seedForcesFromMeasuredCurvature
              -> solve_cosserat_force_map / tip Jacobian seed
      -> solveNonlinearFormulationMap
          -> nonlinearSolverScale
          -> fullMapCost
          -> solve_contact_mpcc
              -> mapMeasurementModel
                  -> decodeMapState
                      -> solveShapeFromStateForces
                          -> solve_cosserat_force_map
              -> fullMapConstraints
                  -> plane_contact_constraints
      -> decodeMapState (accepted x)
      -> force_sensitivity_diagnostic / nonlinear_force_sensitivity
      -> posteriorCovarianceFromLinearization
      -> friction_mode_resolution / friction_direction_quality
      -> output.quality
~~~

旧 `predecessor-linearized` 分支在同一位置改走 `solveConstrainedMapUpdate → finiteDifferenceMeasurementJacobian → solveLinearizedConstrainedMapSubproblem → acceptDampedMapStep`。因此“历史代码存在”不代表它与 Formulation 主路径同时执行；实际分支由 `cfg.forceSensor.complementaritySolver`、`historyStateMode` 和 `subproblemCoordinates` 决定。

### 20.5 场景、协议、基线和审计文件

| 文件 | 读什么/写什么 | 研究含义 |
|---|---|---|
| [contact_demo_scenes.m](../rod/contact_demo_scenes.m) | 返回六个 scene struct：几何、刚度、旋转、平面、mu、噪声和 push/sliding 设置 | 场景定义，不运行优化 |
| [build_contact_demo_truth.m](../rod/build_contact_demo_truth.m) | 调独立 planar shooting，旋转为 3-D，再生成 packet | 防止 truth 从逆解泄漏 |
| [run_contact_demo_suite.m](../rod/run_contact_demo_suite.m) | 按 UUID 保存每个 case 的 MAT/JSON/CSV/MP4，并更新 comparison.json | 公共 demo 与 provenance |
| [render_contact_demo_video.m](../rod/render_contact_demo_video.m) | 读取已保存的 truth/output，用 MATLAB VideoWriter 绘制旧视频同款的杆形、环境、力箭头和力历史 | 连续求解结果的视频导出，不参与估计 |
| [build_multi_contact_truth.m](../rod/build_multi_contact_truth.m) | 生成双接触、曲面和 friction mismatch 的 forward truth | 只做模型边界压力测试 |
| [run_model_mismatch_protocol.m](../rod/run_model_mismatch_protocol.m) | 对上述 out-of-model 输入评分，并标 review | 不能解读成多接触成功 |
| [run_fair_baseline_protocol.m](../rod/run_fair_baseline_protocol.m) | 同一 `sensorInput` 运行 EnFiRCE、shape-only、Gaussian | 公平输入基线 |
| [estimate_planar_multi_contact.m](../rod/estimate_planar_multi_contact.m) | 读取曲率、杆参数、接触平面；可选联合估计平面法向偏移 | 已知接触顺序的平面多接触逆解 |
| [estimate_planar_shape_only_point_loads.m](../rod/estimate_planar_shape_only_point_loads.m) | 同一曲率输入，只使用接触数/顺序而不读平面点和法向 | 多接触形状基线 |
| [run_multi_contact_benchmark.m](../rod/run_multi_contact_benchmark.m) | 三种通道、噪声种子/等级/观测密度；写入配对估计、消融、运行时间 | 396 次多接触软件协议 |
| [run_multi_contact_plane_uncertainty.m](../rod/run_multi_contact_plane_uncertainty.m) | 重放 benchmark 的相同标称观测，错放一面墙并估计平面偏移 | 54 次几何标定敏感性回放 |
| [summarize_multi_contact_benchmark.py](../scripts/summarize_multi_contact_benchmark.py) | 读取逐次 JSON，按种子簇重采样并生成结果表 | 描述性统计，不输出论文间排名 |
| [estimate_shape_only_point_loads.m](../rod/estimate_shape_only_point_loads.m) | 仅由稀疏形状拟合点载荷，不读环境平面 | baseline |
| [estimate_aloi_gaussian_baseline.m](../rod/estimate_aloi_gaussian_baseline.m) | 用稀疏位置拟合弧长 Gaussian 载荷分布 | Aloi-style baseline |
| [run_submission_statistics.m](../rod/run_submission_statistics.m) | 27 组合种子/噪声/场景，输出局部误差和 coverage 标志 | 条件统计，尚不是校准置信区间 |
| [run_history_map_benchmark.m](../rod/run_history_map_benchmark.m) | 比较 fixed/latent history | 历史信息消融 |
| [estimate_temporal_window_forces.m](../rod/estimate_temporal_window_forces.m) | 合并实际观测/前驱时刻，调用每帧完整平衡和摩擦 MPCC | 完整联合离线窗口；模式边缘化与实时递归待优化 |
| [run_realtime_benchmark.m](../rod/run_realtime_benchmark.m) | 统计 frame wall time、p95 和 effective Hz | 计算性能边界 |
| [run_accuracy_benchmark.m](../rod/run_accuracy_benchmark.m) | 在保存输入上做几何/FBG 校准评分 | 复核历史结果 |
| [run_mesh_convergence.m](../rod/run_mesh_convergence.m) | 改 collision/forward mesh step 并比较结果 | 离散化误差 |
| [audit_*.m](../rod) | 不改变估计结果，读取已有 artifact 做准确性、噪声、摩擦、历史和可辨识性报告 | 结果审计 |

### 20.6 测试文件对应的断言类别

`run_project_checks` 当前注册 32 项检查，具体可按名称反查：

| 类别 | 测试文件 |
|---|---|
| 输入和结果协议 | `test_sensor_contracts`, `test_sensor_config`, `test_result_lifecycle`, `test_result_integrity` |
| 环境和几何 | `test_depth_plane`, `test_environment_covariance`, `test_plane_contact_geometry`, `test_cosserat_collision_geometry`, `test_rotation_equivariance` |
| 力学和射击 | `test_contact_shooting`, `test_planar_energy_rod`, `test_cosserat_work_budget`, `test_multi_contact_map` |
| 摩擦和互补 | `test_lcp_dependency`, `test_friction_basis`, `test_friction_regression`, `test_friction_quality`, `test_contact_separation_gate`, `test_reduced_mode_bounds` |
| 形状、历史和敏感度 | `test_intrinsic_curvature_reconstruction`, `test_latent_fbg_history`, `test_integrated_history_point`, `test_history_resolution`, `test_force_sensitivity`, `test_nonlinear_shape_seed`, `test_observation_scaled_map` |
| 噪声和重放 | `test_noise_aware_contact`, `test_sensor_history_boundary`, `test_sensor_replay` |
| 工程语法 | `run_project_checks/checkSyntax` |

`test_sensor_replay` 不是只检查文件存在：它重新读取固定 packet，重复完整逆解，比较保存结果与重放结果的状态和力；本轮两个 replay 均报告 `maxStateDifference=0`、`maxForceDifferenceN=0`。`test_observation_scaled_map` 曾因过期 UUID 失败，现改成从最新 `out/demos/comparison.json` 解析 artifact，避免下一次 demo 运行后再失效。

## 21. 结果字段到物理量的逐项对照

便于从 `results.mat` 或 `data.json` 反查：`truth.*` 只存在离线评分文件，`output.ours.*` 是估计器可输出的结果，`report.*` 是协议层统计。常用字段的实际来源如下：

| 结果字段 | 计算位置 | 解释 |
|---|---|---|
| `output.ours.contactForceResultant(:,it)` | `decodeMapState` 中 `n*fn + D*beta` | 杆对环境的接触反力估计 |
| `output.ours.normalForceResultant(it)` | 同上 `fn` | 法向标量，非负约束变量 |
| `output.ours.frictionForceResultant(:,it)` | `D*beta` | 多面体锥内的切向合力 |
| `output.ours.tipForce(:,it)` | 状态末三维分量 | 独立末端外力；不等于接触力 |
| `output.ours.totalForceResultant(:,it)` | 接触力加末端力 | 杆的总外力平衡分量 |
| `output.ours.contactArcLength(it)` | 状态 `s1` | 杆材料坐标 mm，不是世界坐标距离 |
| `output.ours.contactPoint(:,it)` | `solve_cosserat_force_map` 在 `s1` 插值 | 世界坐标接触点 |
| `output.ours.measurementResidual(:,it)` | `z-h(x)` | 观测白化前/后的残差，查看同文件的定义 |
| `output.ours.complementarityResidual(:,it)` | `complementarityResidual` | gap-force、摩擦位移-beta、lambda-cone slack 的乘积或等价残差 |
| `output.ours.posteriorCovariance(:,:,it)` | `Pplus = pinv(Pminus^-1+H'R^-1H)` | 当前求解分支的局部线性协方差 |
| `output.quality.requiresReview(it)` | `estimate_sensor_forces` 末端质量汇总 | 有数值、几何、可辨识性或观测分辨率警告时为 true |
| `report.metrics.*` | `run_contact_demo_suite` / 各 benchmark | 与独立 truth 比较得到的协议统计，不参与逆解 |

读取 `forces.csv` 时要注意列名中的 `truth_*` 和 `estimate_*`：前者只用于离线评分，后者来自 `output.ours`；CSV 是便携摘要，不能替代 `results.mat` 中的完整状态、协方差和 solverTrace。

## 22. 当前代码中哪些是“已实现”、哪些是“接口/原型”

为了避免交接时把函数名误读成论文结果，按状态明确如下：

- **已实现并有回归检查**：稀疏 FBG 包契约、intrinsic-delta 形状重建、三维 Cosserat 单接触 shooting、平面整杆非穿透采样、离散摩擦锥、Scholtes MPCC 同伦、局部 MAP 协方差、六个独立 demo、已知顺序的平面多接触稀疏曲率逆解、基线输入隔离、结果 provenance 和完整重放。
- **已实现但只用于诊断/压力测试**：双接触/曲面/摩擦失配 forward truth、depth covariance、noise attribution、mesh convergence、局部 force sensitivity、短时间窗口惩罚优化。
- **存在代码路径但没有足够实验支撑**：真实 FBG/相机数据接入、任意曲面几何、未知接触模式和摩擦的三维多接触逆解、完整多模态后验、校准后的置信区间、实时 warm-start 版本。
- **明确未声明**：没有把当前 `posteriorCovariance` 当成 95% 置信区间，没有把无噪声 demo RMSE 当成真实精度，没有把 `forces.mp4` 当成硬件录像，也没有把 mismatch case 当成算法成功。

这一区分是技术文档的一部分：一个入口函数能运行，只说明代码路径存在；只有独立输入、明确评分、重复运行和相应实验设计都完成，才可以在论文中把它写成结果。

## 23. 多接触 benchmark 的状态、目标和几何误差处理

最新协议的逐次记录和场景结果见[软件实验报告](SOFTWARE_BENCHMARK_2026-09-30.md)。这里补充逆解实际执行的数学与代码对应关系。设接触数为 K、杆长为 L、刚度为 EI、观测曲率为 κᵢ，本征曲率为 κ₀ᵢ。`estimate_planar_multi_contact.m` 的优化变量按顺序为 `x = [M₀/(EI/L), λ₁…λK, c₁/L…cK/L, Ftip,x, Ftip,z]`，其中 λ 为非负法向力，c 为杆弧长。每次 `lsqnonlin` 评估都由 `integrate_planar_multi_contact.m` 从这些变量重建预测曲率、接触点与切向量，最小化以下白化残差的平方和：

```text
(predicted_curvature - measured_curvature) / max(curvatureStdPerMm, 1e-7)
tip_moment_residual / 1e-3 Nmm
gapWeight * contact_normal_gap / 1e-3 mm
tangentWeight * normal_dot_contact_tangent / 1e-4
```

`gapWeight=0` 或 `tangentWeight=0` 只删除相应罚项；法向力的方向、接触初值以及弧长分区仍来自环境模型。独立 shape-only 基线用同一曲率观测积分中心线，按接触顺序把弧长分为 K 段，再用带 `0.5 mm` 力正则的线性最小二乘拟合各点二维力及末端力。`test_multi_contact_benchmark_protocol.m` 改动平面点和法向后验证该基线输出不变。

当 `options.planePointStdMm > 0`，在上述状态末尾额外加入每个**实际使用平面**的法向偏移 δₚ（单位 mm）；重复碰到同一平面时共享 δₚ，且每个偏移被限制在 ±3 倍先验标准差。接触间隙残差改为 `normal_dot(contact_point - supplied_plane_point) - δₚ`，并加入 `δₚ / planePointStdMm` 的零均值高斯先验残差。求解后审计使用原平面点加 `δₚ * normal` 的几何；返回 `planePointOffsetMm`、使用的平面索引及数值秩。默认标准差为 0，状态与旧算法完全相同。这是一个局部平面标定误差模型，不表示对任意曲面或相机误差的完整后验。

主 benchmark 的随机噪声在 121 个正向节点上先生成，再按 8/16/24 个 FBG 位置采样；同一场景、种子和状态下不同密度来自同一噪声场。真值、观测包、解和代码 SHA-256 分别保存在 `out/benchmarks/multi_contact/<scene-id>/truth.mat`、`trials.mat` 及根目录的 `comparison.json`。报告脚本以种子而不是逐帧为 bootstrap 抽样单位。`force('multi-geometry')` 读取这些保存的观测包，向第一面墙注入 +1 mm 误差后比较固定几何与两个预设先验，另写 `plane_uncertainty.json`。结果审计中的 `requiresReview` 与离线力误差分开记录，不能用优化器返回成功来代替物理可信度。
