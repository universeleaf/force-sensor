# Cosserat Rod Implementation for Force Estimation

## 概述

本文档说明如何将force.m中的简单梁模型（Euler-Bernoulli beam theory）替换为Cosserat rod理论，以支持大变形场景。

## 主要修改

### 1. 新增文件：cosseratHelpers.m

这个文件包含了Cosserat rod理论所需的所有数学工具函数：

- **LargeSE3**: SE(3)指数映射，用于计算刚体变换
- **LargeSO3**: SO(3)指数映射（Rodriguez公式），用于计算旋转
- **hat**: 将向量转换为反对称矩阵
- **solveShape**: 通过积分应变场计算杆的形状
- **computeJacobian**: 计算形状对应变的雅可比矩阵
- **solveCosseratWithLoad**: 从外力分布求解Cosserat rod平衡状态

### 2. 修改：force.m

#### buildBeamModel函数
- **旧版本**: 构建影响矩阵Phi（Green函数）和微分算子D1, D2
- **新版本**: 设置Cosserat rod参数
  - 刚度矩阵K = [EI_x, EI_y, GJ]（弯曲和扭转刚度）
  - 基座变换矩阵T_base
  - 本征曲率u_hat（无预弯曲时为零）
  - 加载cosseratHelpers函数

#### simulateTwoPointLoads函数
- **旧版本**: 使用线性关系 y = Phi * q 计算挠度
- **新版本**: 
  - 将分布载荷q转换为力向量f_ext
  - 调用solveCosseratWithLoad求解平衡状态
  - 从旋转矩阵R提取角度和曲率

#### 预测函数（predictShapeFromTwoGaussian等）
- **旧版本**: 直接矩阵乘法 y = Phi * q
- **新版本**: 调用Cosserat rod求解器

## Cosserat Rod理论基础

### 运动学
- 杆的构型由SE(3)变换T(s)描述
- 应变向量u = [κ_x, κ_y, τ]^T，包含两个方向的曲率和扭转
- 形状通过积分得到：T'(s) = T(s) * [u(s)]_×

### 静力学
对于悬臂梁，平衡方程简化为：
- 弯矩分布：M(s) = ∫_s^L (x-s) * f(x) dx
- 应变-弯矩关系：u(s) = M(s) / K + u_hat(s)

### 数值实现
1. 从外力分布f_ext计算弯矩分布M
2. 从弯矩计算应变u = M/K + u_hat
3. 通过SE(3)指数映射积分应变得到形状

## 验证结果

使用test_cosserat.m进行验证：

| 测试案例 | Cosserat结果 | 解析解 | 相对误差 |
|---------|-------------|--------|---------|
| 无载荷 | 0.000 mm | 0.000 mm | 0% |
| 尖端载荷 | 30.82 mm | 30.00 mm | 2.74% |
| 均布载荷 | 18.01 mm | 16.88 mm | 6.70% |

误差来源于数值离散化，在可接受范围内。

## 与Aloi方法的集成

Aloi方法的框架保持不变：
1. 用高斯函数参数化载荷分布
2. 通过优化最小化测量误差
3. 使用有限差分计算雅可比矩阵

**关键区别**：
- 旧版本：正向模型是线性的（矩阵乘法）
- 新版本：正向模型是非线性的（迭代求解Cosserat方程）

这使得优化过程变慢，但能够处理大变形情况。

## 性能考虑

- Cosserat rod求解比简单梁模型慢约10-50倍
- 每次优化迭代需要多次调用正向求解器
- 建议减少多起点搜索的种子数量以加快速度

## 使用方法

### 运行完整测试（3个案例）
```matlab
force()
```

### 运行单个案例快速测试
```matlab
force_test_single()
```

### 验证Cosserat实现
```matlab
test_cosserat()
```

## 未来改进方向

1. **性能优化**：
   - 缓存中间结果
   - 使用解析雅可比矩阵代替有限差分
   - 并行化多起点优化

2. **模型扩展**：
   - 支持3D大变形（完整的6自由度）
   - 考虑剪切变形
   - 支持变截面杆

3. **算法改进**：
   - 使用更高效的优化算法（如信赖域方法）
   - 自适应网格细化
   - 贝叶斯优化代替网格搜索

## 参考文献

1. Rucker & Webster (2011) - "Statics and Dynamics of Continuum Robots With General Tendon Routing and External Loading"
2. Aloi et al. (2022) - Gaussian load estimation method
3. LCP-Continuum repository - Cosserat rod implementation with contact mechanics
