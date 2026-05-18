# Force Sensing for Continuum Robots

This repository contains MATLAB implementations of three force estimation methods for continuum robots, based on recent research papers.

**Author:** Tongyu Wang (Georgia Tech)

## Overview

The code simulates a 2D elastic cantilever beam (continuum robot) with various force sensing scenarios and implements state-of-the-art estimation algorithms.

## Files

### Main Implementation Files

- **`force.m`** - Aloi 2022 Gaussian Load Estimation Method
  - Three test cases with two point loads each
  - Estimates distributed load using two-Gaussian parameterization
  - Generates Fig 6 style visualization with three subplots

- **`force_rucker.m`** - Rucker 2011 Extended Kalman Filter Method
  - Tip force estimation using EKF
  - Uses tip pose measurements and actuation inputs
  - Real-time sequential estimation with uncertainty quantification

- **`force_ferguson.m`** - Ferguson 2024 Batch Load Estimation Method
  - Probabilistic batch optimization framework
  - Uses shape and curvature (FBG) measurements
  - Provides full posterior distribution with uncertainty bands

## Aloi Method (force.m)

### Scenario
Three test cases, each with:
- 2D elastic rod fixed at base (L = 0.30 m, EI = 0.03 N⋅m²)
- FBG (Fiber Bragg Grating) sensors at 21 locations for shape/curvature measurement
- Two point loads with **known magnitudes** at different positions

**Case 1:**
- Load 1: 100 mN at 120 mm (body)
- Load 2: 150 mN at 280 mm (near tip)

**Case 2:**
- Load 1: 80 mN at 100 mm (body)
- Load 2: 120 mN at 220 mm (mid-body)

**Case 3:**
- Load 1: 60 mN at 150 mm (mid-body)
- Load 2: 180 mN at 300 mm (tip)

### Method
- Estimates load distribution using sum of two Gaussian functions
- Multi-start optimization (3600 initial guesses) for robustness
- Levenberg-Marquardt optimization with bounds
- Fits to FBG shape measurements

### Output
- `aloi_fig6_three_cases.png` - **Main result**: Three cases in one figure (similar to Fig 6 in paper)
  - Shows robot shape (gray curve)
  - FBG marker locations (green circles)
  - True applied forces (yellow arrows)
  - Estimated load distribution (red line)
  - Blue crosses indicating estimated load directions
- `aloi_case1_results.png`, `aloi_case2_results.png`, `aloi_case3_results.png` - Detailed analysis for each case

### Usage
```matlab
force()  % Run all three Aloi cases
```

### Results and Discussion

**Typical Performance:**
- Shape RMSE: 0.07-0.12 mm (excellent shape fitting)
- Load position errors: 3-40 mm
- Load magnitude errors: 10-67%

**Why the magnitude errors are large:**

The Aloi method uses **Gaussian parameterization** to approximate **point loads**. This is fundamentally challenging because:

1. **Mathematical mismatch**: Point loads are delta functions, while Gaussians are smooth distributions
2. **Two-Gaussian interference**: When two Gaussians are close, they interfere during optimization
3. **Shape insensitivity**: Small changes in load position/magnitude can produce similar shapes
4. **Ill-posed inverse problem**: Multiple load distributions can produce nearly identical shapes

**What the method does well:**
- Excellent shape reconstruction (RMSE < 0.12 mm)
- Identifies that there are two distinct load regions
- Approximate load locations (within 1-4 cm)
- Captures overall load distribution pattern

**Limitations:**
- Cannot precisely recover point load magnitudes (inherent to Gaussian parameterization)
- Struggles when loads are close together (Case 2)
- Tends to overestimate larger loads

This reflects the **actual capabilities and limitations** of the Aloi 2022 method when applied to point loads. The paper's experiments likely used more distributed loads or had additional constraints.

## Rucker Method (force_rucker.m)

### Features
- Extended Kalman Filter for tip force estimation
- Sequential processing of tip pose measurements
- Uncertainty quantification with confidence bounds

### Usage
```matlab
force_rucker()  % Run Rucker EKF method
```

### Output
- `rucker_method.png` - Force estimation with convergence analysis

## Ferguson Method (force_ferguson.m)

### Features
- Batch optimization with probabilistic framework
- Uses both shape and curvature (FBG) measurements
- Provides posterior distribution with 2σ uncertainty bands

### Usage
```matlab
force_ferguson()  % Run Ferguson batch method
```

### Output
- `ferguson_method.png` - Load posterior with uncertainty quantification

## Configuration

### Robot Parameters (common)
- Length: 0.30 m
- Bending stiffness (EI): 0.03 N⋅m²
- Grid points: 101
- FBG sensor locations: 21

### Measurement Noise (Aloi)
- Shape noise: 3.0×10⁻⁴ m
- Curvature noise: 0.02 m⁻¹

## Requirements

- MATLAB R2019b or later
- No additional toolboxes required
- Tested on Windows 11

## Key Differences Between Methods

| Method | Type | Measurements | Output | Uncertainty |
|--------|------|--------------|--------|-------------|
| Rucker | Tip force | Tip pose | Time series | EKF covariance |
| Aloi | Distributed | Shape (FBG) | Gaussian params | Optimization cost |
| Ferguson | Distributed | Shape + Curvature | Full distribution | Posterior bands |

## Visualization

The Aloi method produces a visualization similar to Fig 6 in the original paper, showing three test cases side-by-side:
- Robot backbone shape (gray curve)
- FBG sensor marker locations (green circles)
- True applied forces (yellow arrows)
- Estimated load distribution (red line)
- Blue crosses indicating estimated load directions

This visualization demonstrates that the method successfully identifies load regions and reconstructs the overall shape, even though precise magnitude recovery is challenging for point loads.

## References

1. Rucker & Webster (2011) - "Statics and Dynamics of Continuum Robots With General Tendon Routing and External Loading"
2. Aloi et al. (2022) - Gaussian load parameterization for continuum robots
3. Ferguson et al. (2024) - "Deflection-based force sensing for continuum robots: A probabilistic approach"

## Author

Georgia Tech Force Sensing Research
