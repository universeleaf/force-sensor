# Force Sensing for Continuum Robots

This repository contains MATLAB implementations of three force estimation methods for continuum robots, based on recent research papers.

## Overview

The code simulates a 2D elastic cantilever beam (continuum robot) with various force sensing scenarios and implements state-of-the-art estimation algorithms.

## Files

### Main Implementation Files

- **`force.m`** - Aloi 2022 Gaussian Load Estimation Method
  - Scenario: Two point loads (body + tip) with FBG sensors
  - Estimates distributed load using Gaussian parameterization
  - Generates Fig 6(b) style visualization

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
- 2D elastic rod fixed at base
- FBG (Fiber Bragg Grating) sensors along the body for curvature measurement
- Two point loads with **known magnitudes**:
  - Load 1: 80 mN at 15 cm (body)
  - Load 2: 120 mN at 30 cm (tip)

### Method
- Estimates load distribution using sum of two Gaussian functions
- Multi-start optimization for robustness
- Fits to FBG shape measurements

### Output
- `aloi_method_results.png` - Comprehensive analysis (6 subplots)
- `aloi_fig6_style.png` - Visualization similar to Fig 6(b) in Aloi paper
  - Shows robot shape with true forces (yellow arrows)
  - FBG marker locations (green circles)
  - Estimated load distribution (red markers)

### Usage
```matlab
force()  % Run Aloi method
```

### Results
- Shape RMSE: ~0.13 mm
- Load 1 position error: ~12 mm
- Load 2 position error: ~0 mm
- Successfully estimates load distribution from FBG measurements

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

### Measurement Noise
- Shape noise: 6.0×10⁻⁴ m
- Curvature noise: 0.05 m⁻¹

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

## References

1. Rucker & Webster (2011) - "Statics and Dynamics of Continuum Robots With General Tendon Routing and External Loading"
2. Aloi et al. (2022) - Gaussian load parameterization for continuum robots
3. Ferguson et al. (2024) - "Deflection-based force sensing for continuum robots: A probabilistic approach"

## Visualization

The Aloi method produces a visualization similar to Fig 6(b) in the original paper, showing:
- Robot backbone shape (black curve)
- FBG sensor marker locations (green circles)
- True applied forces (yellow arrows)
- Estimated load distribution (red markers along backbone)

This visualization helps validate that the estimated loads are consistent with the measured shape deformation.

## Author

Georgia Tech Force Sensing Research
