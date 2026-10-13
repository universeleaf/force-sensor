# Gaussian FBG noise with successive rod linearization

Date: 2026-10-11. All work is in our collaboration package, local and uncommitted.

## Measured outcome

All eight full trajectories completed. The selected states satisfy the implemented nonlinear mode constraints at every solved frame, but four selected states hit the outer-iteration limit. High numerical convergence and a noise-consistent FBG fit do not give reliable mode or force identification. The first-acceptable policy often accepts separation while the physical reference is in contact. No limits or physical assumptions were changed to hide these outcomes.

## Calibrated noise

| Target tip RMS | Curvature sigma [1/mm] | Independent validation: insertion / rotation [mm] | Actual noise realization: insertion / rotation [mm] | Raw FBG acceptance [1/mm] |
|---|---:|---:|---:|---:|
| 2 mm | 0.0003707256688 | 2.034 / 1.970 | 2.029 / 2.049 | 0.0004596998 |
| 5 mm | 0.0009245848422 | 5.078 / 4.933 | 5.030 / 5.113 | 0.001146485 |

Validation uses independent Gaussian draws; the actual inverse benchmark has only one paired trajectory realization per scenario. This is not a multi-seed estimate of statistical force accuracy. Calibration includes all 25 rotation states, whereas solve metrics exclude its restored first state. This explains differences between calibration RMS and Stage 1 solve-only tip RMS.

## Timing and convergence

| Noise | Scenario | mu | Converged + feasible | Fit accepted | Mean / median / max time [s] | Mean IVPs | Mean inner / outer iterations |
|---|---|---:|---:|---:|---:|---:|---:|
| 2 mm | insertion | 0.3 | 39/41 | 39/41 | 1.906 / 0.201 / 42.768 | 12.32 | 144.88 / 11.32 |
| 2 mm | insertion | 0 | 41/41 | 41/41 | 0.200 / 0.200 / 0.311 | 4.93 | 48.95 / 3.93 |
| 2 mm | rotation | 0.3 | 24/24 | 20/24 | 26.857 / 1.766 / 435.356 | 26.29 | 941.29 / 25.29 |
| 2 mm | rotation | 0 | 24/24 | 23/24 | 0.219 / 0.203 / 0.502 | 5.00 | 46.46 / 4.00 |
| 5 mm | insertion | 0.3 | 41/41 | 41/41 | 2.188 / 0.189 / 63.148 | 9.78 | 118.24 / 8.78 |
| 5 mm | insertion | 0 | 41/41 | 41/41 | 0.265 / 0.189 / 2.768 | 6.27 | 66.22 / 5.27 |
| 5 mm | rotation | 0.3 | 23/24 | 13/24 | 15.366 / 8.008 / 69.671 | 42.79 | 1553.96 / 41.79 |
| 5 mm | rotation | 0 | 23/24 | 21/24 | 1.107 / 0.213 / 17.573 | 10.71 | 276.21 / 9.71 |

Inner/outer iteration and IVP totals include all competing branches. The 40-outer limit applies per branch, so total outer iterations per frame can exceed 40. Convergence uses the existing positive-inner-exit, nonlinear-feasibility, and small-scaled-step rule, not a global optimality certificate.

## Accuracy and mode identification

| Noise | Scenario | mu | Tip-force RMSE [N] | Contact-force RMSE [N] | Shape RMS [mm] | Estimated tip-position RMS [mm] | Correct contact activation | Correct mode and activation |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 2 mm | insertion | 0.3 | 4.4950 | 5.6417 | 0.6810 | 1.3896 | 17/41 | 17/41 |
| 2 mm | insertion | 0 | 4.2756 | 6.3825 | 0.6748 | 1.3988 | 26/41 | N/A: different law |
| 2 mm | rotation | 0.3 | 6.1618 | 9.4951 | 2.1023 | 3.2436 | 22/24 | 13/24 |
| 2 mm | rotation | 0 | 2.2446 | 5.7523 | 0.9411 | 2.0330 | 24/24 | N/A: different law |
| 5 mm | insertion | 0.3 | 8.0682 | 9.7621 | 2.0157 | 3.9870 | 8/41 | 7/41 |
| 5 mm | insertion | 0 | 7.3573 | 8.6890 | 2.0636 | 4.0927 | 9/41 | N/A: different law |
| 5 mm | rotation | 0.3 | 16.5979 | 20.5307 | 5.0728 | 6.9501 | 7/24 | 3/24 |
| 5 mm | rotation | 0 | 11.1817 | 14.7263 | 2.6301 | 4.6059 | 15/24 | N/A: different law |

Contact activation uses the existing 1e-3 N normal-force diagnostic cutoff; reference activation uses resultant magnitude above 1e-3 N. A zero-force sliding branch is not counted as correct active contact. The mode plot shows selected branch labels, which can be degenerate at zero force. For mu=0, only contact activation is compared, since the frictionless active mode is not a sticking/sliding law.

The frictionless ablation sometimes has lower force error in this one noisy realization. That is not evidence that the physical friction law is unnecessary: the frictional search has wrong-mode acceptance, noisy history, and failed branch solves; both models can also exploit tip/contact load ambiguity.

## Failure and runtime diagnosis

Selected outer-limit failures: 2 mm insertion, mu=0.3, frames 17 and 20; 5 mm rotation, mu=0.3, frame 14; 5 mm rotation, mu=0, frame 20. All four selected states are feasible under the implemented 1e-6 constraint tolerance, but lack convergence. Their costs and errors are retained, and the existing history propagation policy is unchanged. No IVP exceptions occurred.

| Noise / scenario | mu | Frame | Mode | Selected? | Branch time [s] | Inner iterations | IVPs | Outcome |
|---|---:|---:|---|---|---:|---:|---:|---|
| 2 mm / rotation | 0.3 | 3 | 2 | 0 | 430.468 | 6047 | 41 | outer-iteration-limit |
| 2 mm / rotation | 0.3 | 23 | 2 | 0 | 74.803 | 1031 | 41 | outer-iteration-limit |
| 5 mm / rotation | 0.3 | 15 | 2 | 1 | 69.560 | 868 | 10 | linearized-step-converged |
| 5 mm / insertion | 0.3 | 17 | 1 | 0 | 62.109 | 257 | 40 | outer-iteration-limit |
| 2 mm / insertion | 0.3 | 20 | 1 | 0 | 41.599 | 156 | 40 | outer-iteration-limit |
| 2 mm / rotation | 0.3 | 15 | 2 | 0 | 38.627 | 1177 | 41 | outer-iteration-limit |
| 2 mm / rotation | 0.3 | 24 | 1 | 0 | 31.186 | 2966 | 40 | outer-iteration-limit |
| 5 mm / rotation | 0.3 | 9 | 2 | 0 | 29.379 | 1109 | 40 | outer-iteration-limit |

At 2 mm rotation frame 3 (7.5 degrees), the preferred sliding attempt consumes 430.47 s and finishes infeasible after 40 outer iterations. Separation converges but has normalized FBG RMS 1.306 > 1.24; sticking is also infeasible. The chosen no-contact fallback therefore converges without passing the gate. Total frame time is 435.36 s. At frame 23, another rejected sliding search drives a 93.40 s total step.

At 5 mm rotation frame 15, even the selected sliding branch takes 69.56 s despite only ten nonlinear IVPs. Removing repeated rod integration has not removed expensive inner SQP solves. These noisy branches were not separately profiled; their saved inner-iteration and IVP counts identify the remaining optimization workload but are not an exclusive CPU-time attribution.

## Geometry and physics checks

| Noise / scenario, mu=0.3 | Stage 1 tip RMS [mm] | Contact arclength RMS error [mm] | Max normal error [deg] | Max implemented mode violation |
|---|---:|---:|---:|---:|
| 2 mm / insertion | 2.036 | 0.374 | 0.01360 | 4.44e-07 |
| 2 mm / rotation | 1.788 | 0.514 | 0.01073 | 5.55e-07 |
| 5 mm / insertion | 5.038 | 0.911 | 0.00288 | 5.2e-10 |
| 5 mm / rotation | 4.460 | 1.328 | 0.00370 | 2.67e-07 |

Stage 1 reports positive exits on all solved frames. At 5 mm noise, the fixed 2 mm candidate-distance cutoff misses the contact in insertion frame 21 and rotation frames 1 and 22. The first rotation state restores that missed-contact insertion estimate. Additional candidate misses and false detections are visible in all_frames.csv. No candidate threshold was enlarged for these tests.

Across all variants the maximum implemented selected-state violation is 9.6e-07, below 1e-6. Dense diagnostic penetration into the estimated plane reaches 0.000388916 mm; penetration relative to the true plane reaches 0.0111417 mm. Active-contact tangency warnings remain. Thus discrete feasibility is not the same as exact continuous collision or Stage 1/Stage 2 tangency consistency.

The normal estimates remain close to the strongly constrained environmental prior. More influential errors include contact arclength, candidate detection, and the displacement history used in the friction law. For example, at 2 mm rotation frame 2 the estimated tangential displacement is much larger than the true physical slip; the CSV exports both. They are evaluated at their respective estimated/reference contact locations, so their difference also includes contact-location error.

## Interpretation

The normalized 1.24 gate is a noise-aware goodness-of-fit screen, not a certificate of the correct contact mode or force decomposition. A plausible noisy shape can be fitted by a large tip load and small/zero contact force. Carrying the first passing mode forward can then perpetuate the wrong explanation. Searching additional modes may improve the objective but cannot guarantee identification when their predicted measurements are indistinguishable at the noise level.

This experiment does not support using the present first-acceptable policy as a reliable noisy force estimator without further work on mode selection, uncertainty and history consistency. It also shows that changing only the FBG gate does not cure the nonlinear full-cone subsolver outliers. No proposed remedy is implemented here.

## Validation

MATLAB completed with exit code 0. The Python analysis independently checked all 264 saved nonlinear states, force/shape errors, normalization, branch counts and costs, previous-mode-first and early-exit semantics, and noisy insertion-to-rotation checkpoints. It exported 3140 outer iterations. The estimator core files match their previous comparison hashes; only the experiment runner covariance override and two reporting fields changed. Original clean experiments remain untouched.

## Experiment definition

Two noise amplitudes target 2 mm and 5 mm RMS three-dimensional tip displacement of the geometric reconstruction relative to its noise-free counterpart. These are sensor shape errors, not force errors and not final mechanics-estimator tip errors. Independent zero-mean Gaussian noise is added to both Bishop channels at all 24 FBG stations. Base pose and environmental observations remain unchanged; no environmental noise, friction uncertainty, terminal-moment uncertainty, or force priors are introduced.

The calibration reproduces the current segmented PCHIP and SE(3) midpoint integration, with the intrinsic-material break at 80 mm and 200 one-millimetre integration intervals. All 41 insertion and 25 rotation states contribute. Calibration uses 256 Gaussian realizations per frame, weights the two scenarios equally, and uses independent validation with 512 realizations per frame. A fixed amplitude is used throughout both scenarios at each level. Python-generated endpoints are checked against the actual MATLAB reconstruction for every noisy frame (tolerance 1e-8 mm).

The actual benchmark uses one independent trajectory realization, RNG seed 33001, shared between amplitudes and between estimator mu values. Its insertion and rotation measurement draws are independent except that rotation frame 1 reuses the maximum-insertion observation. Previous-observation packets refer to the actual preceding noisy packet, never a fresh independent redraw of the same past measurement. First-frame previous observations duplicate that frame, matching the original packet convention.

Stage 1 and Stage 2 receive the same noisy measurements and covariance sigma^2 I. The benchmark runner previously hardcoded 1e-8 I; it now honors cfg.sensorStd^2, retaining 1e-8 I as its default. Normalized FBG acceptance is 1.24. All rod/contact algorithms, numerical tolerances, objective multiplier, iteration limits, and mu values remain unchanged from the linearized benchmark. The full circular friction law and previous-mode-first search are preserved. Diagnostics remain reporting-only, and infeasible/nonconverged results are retained and propagated under the existing policy.

Rotation restores its own corresponding noisy insertion estimate at frame 21, separately for mu=0.3 and mu=0. This is a continuous noisy experiment, rather than an artificially pristine mechanical reset. Rotation frame 1 is saved but excluded from solve timing and convergence aggregates. The true trajectory remains the original maximum-insertion fixture.

The truth coefficient is 0.3 throughout; mu=0 is an estimator ablation on the same physical observations. No new baseline nonlinear run is needed for this request. Comparison to earlier noise-free timings is contextual: both the acceptance gate and the rotation checkpoint quality differ.

## Reproduction and outputs

Run from the package with a fresh stamp to preserve existing runs:

```text
python experiments/inclined_wall/calibrate_noisy_modes.py NEW_STAMP
```

```matlab
setup_tsfs;
addpath('experiments/inclined_wall');
run_noisy_linearized('NEW_STAMP');
```

```text
python experiments/inclined_wall/analyze_noisy_linearized.py NEW_STAMP
```

Raw calibration and noisy input MAT files: results/runs/noisy_linearized_20261011/.
Individual estimator MAT files: results/runs/inclined_{insertion,rotation}_noisy{2,5}mm_20261011/.
Individual per-frame and branch CSV files: results/published/ with the same run names.
Combined results: results/published/noisy_linearized_20261011/.

The reported step times include both sensing stages, diagnostics, and all failed/rejected mode attempts. They exclude noise calibration, physical truth simulation, input preparation, and disk serialization. They are single-run measurements, not statistical timing benchmarks. A converged feasible state, a passed FBG gate, and a correct contact mode are separate outcomes.
