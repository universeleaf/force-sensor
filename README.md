# 杆—平面接触力估计

当前入口是根目录的 `force.m`。MATLAB 的 Current Folder 请设为本仓库根目录。

```matlab
r = force();                  % 主场景：竖直墙面，杆身接触
r = force('senior');          % 学长场景：水平面，杆端接触
```

需要 MATLAB 和 Optimization Toolbox（`fmincon`），以及本地 `LCP-Continuum/`。
完整运行包含逐帧非线性 Aloi 对照，可能需要数十分钟；仅重新出图无需重算。快速近似模式 `force('wall',true)` 仅用于调试流程，可能被严格验收拒绝。

## 看结果

| 场景 | 视频 | 摩擦检查 | 数据和数值摘要 |
|---|---|---|---|
| 主场景 | [forces.mp4](out/wall/forces.mp4) | [friction.png](out/wall/friction.png) | [summary.txt](out/wall/summary.txt) |
| 学长场景 | [forces.mp4](out/senior/forces.mp4) | [friction.png](out/senior/friction.png) | [summary.txt](out/senior/summary.txt) |

每个结果文件夹还包含 `results.mat`、`trajectory.csv`、`overview.png`、`forces.png`、`aloi.png` 和 `aloi.mp4`。力均指**平面对杆的力，使用世界坐标，单位 N**；长度和单步位移用 mm。

学长场景接触点在杆端，接触力与独立末端载荷不能唯一分离。修复后摩擦方向及约束通过检查，但不要用总力误差代替两个分量的识别精度。Aloi 的局部横向载荷假设也不覆盖当前完整载荷方向，详见结果摘要。

## 验证与重新出图

```matlab
addpath('rod');
test_friction_regression();                    % 两个保存结果 + 故意破坏约束
validate_rod_plane_displacement_forward();      % 独立正向物理检查
test_reverse_slide();                         % 反向滑动、内部步长减半
run_rod_plane_force_sensing_experiment( ...
    'render', fullfile(pwd,'out','wall','results.mat')); % 只重新出图/视频
```

## 文件位置

- `rod/`：当前求解器、场景入口和验证代码。
- `out/wall/`、`out/senior/`：当前正式结果。
- `legacy/`：历史 Aloi / Rucker / Ferguson 示例，原根目录 `force.m` 在此。
- `papers/`：原有论文和公式 PDF。
- `docs/`：会议材料与验证记录。
- `archive/`：旧输出、原代码及诊断归档；由 Git 忽略。
- `LCP-Continuum/`：原有外部依赖，保持原样。

**新窗口先读 [MEMORY.md](MEMORY.md)。** 它记录坐标约定、修复原因、验证方法、已知限制和接手步骤。
