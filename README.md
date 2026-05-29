# Force Sensing for Continuum Robots

This repository contains MATLAB implementations of force estimation methods for continuum robots using Cosserat rod theory.

**Author:** Tongyu Wang (Georgia Tech)

## Overview

The code implements force estimation for continuum robots with large deformations using Cosserat rod theory combined with the Aloi 2022 Gaussian load estimation method.

## Main Files

### Core Implementation

- **`force.m`** - Aloi 2022 Method with Cosserat Rod Theory
  - Supports large deformations using Cosserat rod kinematics
  - Three test cases with two point loads each
  - Estimates distributed load using two-Gaussian parameterization
  - Generates visualization with three subplots

- **`cosseratHelpers.m`** - Cosserat Rod Mathematical Functions
  - SE(3) and SO(3) exponential maps
  - Shape integration from strain field
  - Equilibrium solver for external loads

- **`test_cosserat.m`** - Validation Tests
  - Verifies Cosserat implementation against analytical solutions
  - Tests unloaded, tip load, and distributed load cases

### Documentation

- **`README_COSSERAT.md`** - Detailed usage guide
- **`COSSERAT_IMPLEMENTATION.md`** - Technical documentation

## Method

### Cosserat Rod Theory

The implementation uses Cosserat rod theory for accurate modeling of large deformations:

- **Kinematics**: Rod configuration described by SE(3) transformations
- **Strain**: Curvature and twist vector u = [κ_x, κ_y, τ]^T
- **Equilibrium**: Moment balance M(s) = ∫_s^L (x-s) * f(x) dx
- **Constitutive**: Strain-moment relation u(s) = M(s) / K + u_hat(s)

### Load Estimation

- Gaussian parameterization of distributed loads
- Multi-start optimization for robustness
- Levenberg-Marquardt with bounds
- FBG shape measurements for fitting

## Test Cases

### Robot Parameters
- Length: 0.30 m
- Bending stiffness (EI): 0.03 N⋅m²
- Grid points: 101
- FBG sensor locations: 21

### Three Scenarios

**Case 1:**
- Load 1: 100 mN at 120 mm
- Load 2: 150 mN at 280 mm

**Case 2:**
- Load 1: 80 mN at 100 mm
- Load 2: 120 mN at 220 mm

**Case 3:**
- Load 1: 60 mN at 150 mm
- Load 2: 180 mN at 300 mm

## Usage

```matlab
% Run all three test cases
force()

% Quick test (single case, reduced iterations)
force(true)

% Validate Cosserat implementation
test_cosserat()
```

## Output

Generated files in `force_outputs/`:
- `aloi_fig6_three_cases.png` - Three cases comparison
- `aloi_case1_results.png` - Case 1 detailed results
- `aloi_case2_results.png` - Case 2 detailed results
- `aloi_case3_results.png` - Case 3 detailed results

## Performance

### Typical Results
- Shape RMSE: 0.07-0.12 mm (excellent)
- Total force error: 5-12%
- Convergence: All cases converge successfully

### Validation
Cosserat implementation verified against analytical beam solutions:
- Tip load: 2.74% error
- Uniform load: 6.70% error

## Requirements

- MATLAB R2019b or later
- No additional toolboxes required
- Tested on Windows 11

## Key Features

- ✅ Large deformation support via Cosserat rod theory
- ✅ Gaussian load parameterization (Aloi method)
- ✅ Multi-start optimization for robustness
- ✅ FBG sensor simulation
- ✅ Comprehensive visualization

## Performance Notes

Cosserat rod solving is iterative and slower than linear beam models:
- Beam model: ~0.001s per evaluation
- Cosserat rod: ~0.01-0.05s per evaluation
- Full optimization: 5-15 minutes

Use `force(true)` for faster testing with reduced iterations.

## References

1. Rucker & Webster (2011) - "Statics and Dynamics of Continuum Robots With General Tendon Routing and External Loading"
2. Aloi et al. (2022) - Gaussian load parameterization for continuum robots
3. Shen et al. (2024) - "Friction Modeling of Continuum Robots Through Linear Complementarity Problem"

## License

MIT License - See LICENSE file for details

## Author

Georgia Tech Force Sensing Research
