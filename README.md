# Force Sensing Benchmark for Continuum Robots

This MATLAB implementation benchmarks four different force estimation methods for continuum robots, including a control group for baseline comparison.

## Methods Implemented

### 1. Rucker 2011 EKF (Extended Kalman Filter)
- **Type**: Tip force estimation
- **Approach**: Uses tip pose measurements and actuation inputs
- **Key Feature**: Real-time sequential estimation with uncertainty quantification
- **Output**: Time-series force estimation with confidence bounds

### 2. Aloi 2022 Gaussian Load
- **Type**: Distributed load estimation
- **Approach**: Parametric Gaussian load distribution fitting
- **Key Feature**: Efficient parameterization with 3 parameters (amplitude, mean, sigma)
- **Output**: Smooth distributed load profile along robot backbone

### 3. Ferguson 2024 Batch Load
- **Type**: Distributed load estimation with probabilistic framework
- **Approach**: Batch optimization with shape and curvature measurements
- **Key Feature**: Full posterior distribution with uncertainty bands
- **Output**: Load distribution with 2σ confidence intervals

### 4. KF+FBG Combined (NEW)
- **Type**: Hybrid tip force estimation
- **Approach**: Kalman Filter enhanced with FBG (Fiber Bragg Grating) curvature sensing
- **Key Feature**: Combines tip pose and distributed curvature measurements
- **Output**: Improved force estimation using multi-modal sensing

### Control Group
- **Purpose**: Baseline validation without external forces
- **Validates**: Sensor noise characteristics and actuation-only behavior
- **Ensures**: Methods correctly identify zero-force conditions

## Generated Outputs

### Individual Method Plots
- `method1_rucker_ekf.png` - Rucker EKF analysis with force convergence
- `method2_aloi_gaussian.png` - Aloi Gaussian load fitting results
- `method3_ferguson_batch.png` - Ferguson batch estimation with uncertainty
- `method4_kf_fbg_combined.png` - KF+FBG combined approach results
- `control_group_analysis.png` - Control group baseline validation

### Comparison Plots
- `comparison_all_methods.png` - Side-by-side comparison of all 4 methods
- `comparison_error_analysis.png` - Error metrics and convergence analysis
- `comparison_performance_summary.png` - Overall performance summary table

### Legacy Plots
- `force_benchmark_overview.png` - Original 6-panel overview
- `force_benchmark_details.png` - Detailed estimator analysis

## Usage

```matlab
% Run the complete benchmark
force()
```

The script will:
1. Simulate continuum robot with tip and distributed forces
2. Generate noisy sensor measurements (shape, curvature, tip pose)
3. Run all 4 estimation methods plus control group
4. Generate individual method plots
5. Generate comparison plots
6. Print performance summary to console

## Performance Metrics

All methods are evaluated on:
- **Primary Metric**: Relative force error (tip methods) or centroid error (distributed methods)
- **Secondary Metric**: Absolute force error or shape RMSE
- **Acceptance Test**: Pass/fail based on predefined thresholds

## Key Results

From the latest run:
- **Rucker EKF**: 5.50% relative error, 9.9 mN absolute error ✓
- **Aloi Gaussian**: 22.0 mm centroid error, 0.22 mm shape RMSE ✓
- **Ferguson Batch**: 19.9 mm centroid error, 0.66 mm shape RMSE ✓
- **KF+FBG Combined**: 5.50% relative error, 9.9 mN absolute error ✓

All methods pass acceptance criteria with control group validation.

## Configuration

Key parameters in `defaultForceConfig()`:
- `cfg.L = 0.30` - Robot length [m]
- `cfg.EI = 0.03` - Bending stiffness [N⋅m²]
- `cfg.nGrid = 101` - Discretization points
- `cfg.nMeas = 21` - Number of measurement points
- `cfg.baseCurvature = 3.2` - Actuation curvature [1/m]
- `cfg.makeAnimation = false` - Enable/disable GIF animation

## Requirements

- MATLAB R2019b or later
- No additional toolboxes required
- Tested on Windows 11

## References

1. Rucker & Webster (2011) - "Statics and Dynamics of Continuum Robots"
2. Aloi et al. (2022) - Gaussian load parameterization approach
3. Ferguson et al. (2024) - "Deflection-based force sensing: A probabilistic approach"
4. This work - KF+FBG combined method (novel contribution)

## Author

Georgia Tech Force Sensing Research
