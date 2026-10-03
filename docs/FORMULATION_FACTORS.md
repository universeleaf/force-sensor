# 完整三维窗口的因素实验与本轮修复

更新：2026-10-04。项目主线仍是从稀疏形状测量和环境观测推断接触力；力学、摩擦和概率模型沿用 Formulation。本文说明新的实现、实验接口和结果解释。实际实验数据生成后再填写数值，不用计划替代证据。

## 1. 已定位并修改的代码问题

### 1.1 同一接触移动后被重复计数

旧的 `formulation_contact_candidates` 将整个窗口的间隙极小值放进一个池，然后按固定 10 mm 合并。同一接触从 20 移到 35 再移到 50 mm，可能被当成三个候选。现在先在每帧、每个平面内合并极小值，再调用 [track_formulation_candidates.m](../rod/track_formulation_candidates.m) 跨帧关联。

关联只使用平面编号、观测重建形状的极小值弧长和时间。距离门限为 `candidateTrackMaxFraction * L * sqrt(max(1, dt/referencePeriod))`，默认比例 0.25。匹配一对极小值后，该轨迹和当前极小值不再参与本帧其余匹配。未匹配极小值产生新轨迹，未观测帧使用相邻观测弧长线性插值；观测区间之外使用最近端点。候选优先级先看被观测帧数，再看最小间隙误差，截断、近角点和关联近似平局均写入审计。它是有界搜索，不是拓扑覆盖定理。

`seedArcByFrameMm` 将每帧的位置送给初始力矩拟合；`seedArcMm` 保留首帧位置和兼容接口。法向互补仍允许某个槽位成为零反力，不向估计器输入真实接触数或模式。

### 1.2 固定初值分区阻止真实接触迁移

旧实现按首帧候选中点划分互不重叠区间。初始接触在 20/40 mm 时，第一个接触不能移动到 60 mm，即使第二个已经移动到 80 mm。新 [formulation_contact_arc_domain.m](../rod/formulation_contact_arc_domain.m) 默认 `contactArcMode='ordered'`，只保留端点保护与 `s_(i+1)-s_i >= minArcSeparationMm`，不限制它留在首帧分区。

这些是线性约束。物理坐标约束 `A_x*x <= b_x` 转换到优化尺度后为 `A_y=A_x(:,free)*diag(scale)`、`b_y=b_x-A_x*x0`。最终 `fmincon` 接收硬线性顺序约束。初始化采用信赖域 `lsqnonlin`，用顺序违约残差与每阶段投影产生有序初值，避免受约束最小二乘在退化状态上的病态 KKT 系统；它不改变最终 MAP 的硬约束。`partitioned` 仍可显式选择，用于旧搜索域的消融。

优化器和中央差分可以短暂产生顺序违规试探。积分器依靠排序后的分段边界计算载荷，保留原输入列的接触身份；最终结果仍要通过顺序约束和原始物理约束审计。不存在用交换接触标签来通过评分的操作。

### 1.3 相同状态的导数重复积分

旧目标梯度、约束梯度、测量信息矩阵和活动约束信息矩阵各自进行一次全状态有限差分。[formulation_derivative_bundle.m](../rod/formulation_derivative_bundle.m) 对 `[r;a;b;c;e]` 做一次中央差分，步长保持 `h=1e-5*max(1,abs(y_j))`。`estimate_formulation_window` 按完全相同的优化状态缓存整个矩阵，不对状态取整，不修改 Cosserat 方程。

目标梯度为 `J_r' * r`。同伦不等式 Jacobian 为 `[J_c;-J_a;-J_b;diag(b)*J_a+diag(a)*J_b]`。改变同伦参数只改变常数项，因此矩阵可继续复用。测量信息矩阵取残差 Jacobian 中曲率和相机的行，排除先验与过程行。活动约束使用相应物理行、活动线性顺序行以及解析上下界行。力协方差中的力向量直接由法向、摩擦方向和载荷参数计算，不再为查询力重复积分杆形。

`useSharedDerivatives=false` 可保留目标与约束分开求导的对照。`solver.derivativeBundleEvaluations/derivativeBundleCacheHits` 报告实际工作量。`endToEndSeconds` 包含协方差计算；历史 `optimizationSeconds` 保留此前的优化时间语义。

### 1.4 退化互补分支不能靠退出标志认证

零摩擦时某些互补变量固定为零，乘积约束的梯度退化。新增活动分支精化：把识别出的分支表示为 `a_i=0` 或 `b_i=0`，仍保留全部原始非负约束、平衡约束与原始目标。分支行按对偶侧大小缩放，精化使用更严格的约束容差。先用非线性最小二乘恢复该分支的等式和非负可行性，随后重新优化原 MAP；恢复步骤本身不能当成 MAP 收敛。恢复残差包含 `restorationMapWeight=1e-4` 倍的原 MAP 残差，避免无相机等弱约束组合的恢复步偏离观测形状；这只影响初值恢复，不修改最终目标、观测协方差或物理约束。记录中的 `initialMaxViolation/restoredMaxViolation` 只计物理项，另外记录恢复 MAP 权重与未加权残差平方范数。结果必须具有正的 MAP 优化器退出标志，并重新满足原始未松弛互补、间隙、平衡与顺序约束后才可接受。分支选择、恢复前后违约、退出和是否接受逐项保存在 `solver.activeBranchPolish`。这是已识别分支的局部求解，不代表枚举所有接触模式。

分支优化默认约束容差与精确同伦目标统一为 `1e-8`，不再额外除以十。此前的 `1e-9` 比 `1e-8` 步长停止条件更严格，曾把原始违约仅 `3.8e-9` 的干净三接触恢复点拒绝为不可行。修复后仍要求真实正退出和完整原始物理审计，不改写退出码；分支记录额外保存约束容差、步长容差、优化器违约、一阶最优性和完整停止消息。

恢复步骤还必须通过[formulation_restoration_merit.m](../rod/formulation_restoration_merit.m)：对同一组固定残差重新计算起点与返回点的平方范数，返回点非有限或代价变差时保留起点，再运行原 MAP。不能只信任 `lsqnonlin` 的退出标志或它报告的残差。`restoration.accepted/initialResidualSquaredNorm/candidateResidualSquaredNorm` 保存该决定；候选物理违约与求解器原始返回范数仍保留。此保护改变初值的接收逻辑，不改变目标函数、物理条件或评分。

活动分支精化还区分了三种约束身份：真正的非线性非负条件、已经由变量边界保证的条件、以及零摩擦或缺少前驱时恒为零的互补对。只在**分支精化的数值表示**中去掉重复非负行和恒等零行。原始 MPCC 同伦保留其全部行，最后的验收仍检查全部原始条件；没有删掉摩擦模型。否则，零摩擦时 beta 的零边界、同一个 beta 的非负行和分支等式会把退化行重复交给优化器，导致病态约束系统。

优化器产生非有限或离开杆内部的试探状态时，`decode` 抛出专用 `rod:InvalidOptimizerTrial`，与数值平衡失败一起作为局部试探拒绝。维数错误、格式错误及其他编程错误继续抛出，不以巨大残差吞掉。公共积分器的载荷校验也没有放宽。`solver.rejectedTrials/lastRejection` 保留试探拒绝记录。

### 1.5 非活动候选不应被当成真实接触评分

[score_formulation_window.m](../rod/score_formulation_window.m) 分别报告候选数和每帧活动接触数。在活动数与真值数一致时，按弧长顺序配对活动槽位；不根据力误差挑选最有利的排列。若任一帧活动数不一致，整个窗口的接触 RMSE 保留为空，仍报告匹配帧数、末端力和总合力误差。真值只在求解之后进入此函数。

### 1.6 连续接触位置必须参与所有平面的碰撞检查

规则碰撞网格之外，积分器现在追加每个当前连续接触弧长。坐标重合时也保留重复点，不去重或排序，以保持优化约束行的身份和数量稳定。候选对所属面的间隙已经在法向互补中施加，整杆不等式只删除这一重复行；该候选对所有其他平面的间隙仍检查。这样避免规则网格恰好漏过候选与另一平面的碰撞，也避免接触离开网格节点时约束维数改变。网格之间的连续碰撞覆盖仍由采样精度限制。

## 2. 每个因素究竟关闭什么

| 实验方法 | 参数 | 删除的因素 | 仍保留的信息 |
|---|---|---|---|
| full | 全部默认打开 | 无 | 完整三维平衡、形状与相机似然、环境与摩擦互补、时间先验 |
| no_temporal | `useTemporalPrior=false` | 物理状态随机游走因子 | 两时刻平衡和同一材料点摩擦历史 |
| cone_only | `useFrictionHistory=false` | 滑动速度/摩擦对偶互补，lambda 固定为零 | 非负摩擦系数、静态摩擦锥、环境接触 |
| no_geometry | `useContactGeometry=false` | 法向间隙互补、整杆间隙和接触切向等式 | 环境方向、相机似然、观测候选、力学；不能称纯 shape-only |
| no_camera | `useEnvironmentLikelihood=false` | 相机几何测量似然 | 潜在环境参数、接触约束与弱初始先验；可能出现数据不可辨识 |
| legacy_partitions | `contactArcMode='partitioned'` | 整根杆上的搜索域 | 旧初值分区；其他因素与 full 相同 |
| point / gaussian | 文献适配求解器 | 完整环境/摩擦/时间模型 | 同一曲率观测、标定、基座；共享观测推导的候选数 |

消融仍输出所有原始物理残差，但 `configuredConstraintViolation` 检查本方法实际施加的约束，避免把刻意删除的因素当编程错误。数据秩不足和观测拟合警告继续保留。

## 3. 实验工作流与数据边界

[run_formulation_factor_protocol.m](../rod/run_formulation_factor_protocol.m) 校验已完成的干净输入与真值 SHA；仅从干净传感器包生成独立噪声，再向每个方法传入完全相同的观测。各方法都冷启动，不读取参考估计、参考力或参考接触位置作为初值。真值文件只在求解结束后加载评分。

默认两条轨迹：三接触和真正面外滑动双接触；种子 11/23/37；曲率噪声 SD=0、2.5e-5、5e-5 /mm；六种完整窗口方法。每个窗口含两个实际平衡状态。这里的三个种子是同一固定轨迹的独立传感器实现，不是三个独立轨迹，也不能把相邻帧当作统计独立重复。可选 `sampleEnvironment=true` 会同时抽取给定环境协方差的点/法向扰动，并重新归一化法向；该归一化使噪声模型带有非线性近似，须在结果中注明。

每个条件重新创建相同种子的随机流。因而同一场景/种子在两个非零噪声等级使用同一标准化噪声、仅改变幅度；两个场景在张量维数相同时也复用该标准化噪声。这是共同随机数的配对控制，不能把 12 个含噪输入包当成 12 个全局独立噪声样本。每个固定条件内的三个种子才是独立的噪声重复。均值和最小/最大种子范围按条件计算，不跨条件套独立样本置信区间。

```matlab
force('formulation-factors', true);  % 隔离的两接触、干净六方法检查
force('formulation-factors');        % 两场景、三种子、三级噪声、六因素方法
run_formulation_factor_protocol(struct('methods',{{'full','point','gaussian'}}, ...
    'outputName','custom_literature')); % 自定义选择使用独立结果目录
run_formulation_derivative_benchmark;
force('publication'); % 完整消融、字节相同的文献对照、性能和工程检查
```

完整结果位于 `out/benchmarks/formulation_factors/`，检查结果位于其 `smoke/`，不会覆盖历史 `out/formulation_window/`。每包保存 `input.mat/truth.mat`，每方法保存 `estimate.mat/forces.csv`，每完成一项原子更新 `comparison.json`。失败保持原始异常和已保存输入；重启时只跳过源码、配置和文件 SHA 一致的完成项。配置或源码改变时拒绝混合旧结果。

[formulation_resume_matches.m](../rod/formulation_resume_matches.m) 比较恢复身份：四个协议向量和源码/依赖结构数组统一方向，结构字段按名称排序，再比较所有值。JSON 读回的数组方向差异不再误报依赖改变；改变任一源文件、依赖、参考输入或求解参数仍拒绝恢复。回归检查实际执行 JSON 往返、字段重排和三类篡改。

资源充足时可运行 `python scripts/run_publication_parallel.py --jobs 6`。它按“场景 × 种子”拆成六组，各组自行执行所有噪声和方法，只写自己的目录。汇总前检查源码/依赖/配置及每个原始文件的 SHA，并确认 108 个组合恰好出现一次。工作进程原始 JSON 保存在完成目录的 `shards/`，每个汇总案例可追溯到工作进程。程序采用独占 PID 文件拒绝两个协调进程同时运行；强制终止后应先确认该进程已退出，再清理本地 PID 文件。

因素工作进程固定单计算线程；汇总后的顺序阶段使用 MATLAB 默认线程。旧 `wall_tip_seeded` 归档是在默认线程下求解的，强制单线程会改变敏感的 SQP 数值路径，不能要求它与默认线程归档逐位相同。同一传感器输入恢复默认线程后，状态和合力差均为 0。当前调度器保持归档重放的线程条件，未放宽 `1e-8` 比较阈值。`parallel_plan.coordinatorSourceArchive` 保存因素阶段实际执行脚本的字节和 SHA；`sequentialCoordinator` 分别记录完成阶段版本、线程模式、起止时间和真实进程退出码。仅当因素已经完整完成、原执行脚本归档有效且 MATLAB/配置身份不变时，才允许修复后的调度器续接完成阶段。

因素运行的 wall time 包含局部协方差，但并行时共享机器资源，不能作为隔离性能排名。全部工作进程退出后，同输入文献基线、两次导数计时及工程检查顺序执行。只有四项步骤完成且数据/图源完整时才更新论文与网站。

[run_formulation_publication_protocol.m](../rod/run_formulation_publication_protocol.m) 将六方法的 108 项因素实验、两个适配基线的 36 项运行、两项共享求导性能运行和工程检查串成一个入口。基线树里的 `input.mat/truth.mat` 是因素树文件的逐字节复制，复制后验证 SHA；方法之间不靠“设置相同种子”声称输入相同。各步骤完成后写入 `out/benchmarks/publication/completion.json`，出现异常保留步骤与栈并停止该组合发布。无异常完成不代表每项消融都达到高精度，非正退出、违约和 review 仍必须列出。

运行全部完成后执行 `python scripts/render_formulation_publication.py`，脚本校验实验笛卡尔积、每个输入/真值/估计/CSV 的 SHA，以及三个比较方法之间的输入字节一致性。输出接触/末端/总合力的配对图、逐接触力大小图、均值和种子范围表、原始作图 CSV、图文件 SHA 和渲染器 SHA。图中种子范围不是独立轨迹的置信区间。`method_overview` 明确是算法结构示意，其余定量图来自实际求解输出。

`python scripts/update_publication_manuscript.py` 仅在完整四步状态、全部工程检查和图表 SHA 一致时，生成公开详细结果文档并更新 Overleaf 本地稿件的真实数值、表和图。`python scripts/compile_publication_manuscript.py` 使用现有 latexmk 编译提供的多文件模板，记录实际 TeX、参考文献、类文件及图表字节，输出 PDF 与编译记录。稿件输入变化后旧 PDF 不能通过同步。完成编译后，`python scripts/sync_publication_website.py` 向独立网站 checkout 复制已校验结果及真实 MP4。同步脚本本身不执行 Git 推送。

## 4. 覆盖率怎么计算、不能说明什么

[score_formulation_coverage.m](../rod/score_formulation_coverage.m) 计算世界坐标力分量 `f ± 1.95996398454005*SD` 的局部 95% 区间。只统计数据可辨识且标准差有限的分量，单独报告不可辨识分量、接触数不匹配帧、区间宽度、分子和分母。无噪声的确定性运行不作为覆盖率校准。

默认环境观测固定，重复的是曲率噪声；因此覆盖率是对固定环境和固定合成轨迹的条件评估。算法的局部协方差仍含环境参数和过程先验。不同分量、接触和相邻帧相关，分量覆盖率不是独立 Bernoulli 试验，也不能直接套二项独立置信区间。当前没有边缘化候选、接触模式与材料标定误差；`coverageValidated` 仍保持 false，直到更广泛联合后验检验提供证据。

## 5. 独立回归与阅读入口

[test_formulation_search_and_derivatives.m](../rod/test_formulation_search_and_derivatives.m) 用可行位置迁移、一个移动极小值、两个独立极小值和独立解析 Jacobian 检查新搜索/求导；[test_formulation_window_mechanics.m](../rod/test_formulation_window_mechanics.m) 检查独立真值及重排试探的力学等价；[test_formulation_factor_protocol.m](../rod/test_formulation_factor_protocol.m) 检查非活动槽位配对、不可辨识区间排除和随机种子重现。这些已注册到工程检查入口。

项目所有力学、传感器接口与代码地图仍以[技术总说明](TECHNICAL_OVERVIEW.md)为主。新数值表和图片必须由实际完成的运行账本生成；不会因为接口已经完成，就把尚未执行的组合写成完成结果。
