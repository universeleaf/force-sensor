# Residual and convergence audit: 22.5 and 41.25 degrees

Date: 2026-10-07. Branch codex/two-stage-force-sensing. Analysis only: production solver and earlier benchmarks were not changed. No commit or push.

## Conclusion

These results do not establish two converged MAP minima. At 41.25 degrees, the linear-start output is an unfinished solve; restarting from its returned state reaches essentially the same low-cost solution as the previous-material-state start. At 22.5 degrees, the bad output selects sliding instead of sticking and passes an overly loose scaled stationarity check. A stricter restart stagnates, and feasible descent exists from both the original and the stalled states. Neither bad point is supported as a local optimum. Global uniqueness of this nonconvex MAP problem is not proved.

## Controlled comparison

For each angle, both starts use the same current Bishop observations, actuation, Stage 1 geometry, force priors and previous mechanical shape. The previous state is taken from the saved recurrent benchmark. Geometry and prior identity are asserted. These are same-history comparisons, not comparisons of different accumulated trajectories.

The implementation minimizes F=0.5||r_w||^2+0.5||r_tip-prior||^2+0.5||r_contact-prior||^2. Residuals are whitened; the declared Bishop standard deviation is 1e-6/mm. Cosserat differential equations are enforced by the IVP, not appended as a data residual. Terminal moment, contact projections and sampled nonpenetration are hard constraints. Their violations enter the SQP merit function but are not additional MAP objective blocks.

## Stage 1

Stage 1 is identical under both mechanics initializations. Its objective here is expressed as half the squared whitened residual norm, for consistency with the Stage 2 cost convention. Geometry covariance can couple gap and tangency, so individual whitened components should not be interpreted as independent physical errors.

| Angle | Shape RMS (mm) | Arc error (mm) | Normal error (deg) | Stage 1 half-squared cost |
|---|---|---|---|---|
| 22.5 | 0.000538294 | 0.001498433 | 0.00219607 | 0.001088393 |
| 41.25 | 0.0006492033 | -0.0007478848 | 0.0005785525 | 7.45841e-05 |

## MAP objective blocks at the returned original outputs

| Angle | Initialization | Total | Bishop sensor | Tip prior | Contact prior | w1 contribution | w2 contribution |
|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 404.0561 | 403.9911 | 0.06218465 | 0.002859516 | 392.1749 | 11.8162 |
| 22.5 | linear | 16.81385 | 16.81278 | 1.107651e-06 | 0.001071809 | 16.27016 | 0.542613 |
| 41.25 | previous-material | 0.1335074 | 0.1329624 | 3.932946e-07 | 0.0005446089 | 0.1030231 | 0.02993932 |
| 41.25 | linear | 424135.6 | 424135.4 | 0.1962787 | 0.005644177 | 229993.2 | 194142.3 |

The sensor block dominates the discrepancy at both angles. Force tracking priors do not explain the preference for the bad result. At 22.5 degrees most of the sensor excess is in w1; w1 is a Bishop-frame curvature component, not a world-force component.

| Angle | Initialization | Sensor RMS / sigma | Tip error (N) | Contact error (N) | Shape RMS (mm) | Contact branch | Slip (mm) | Cone margin (N) |
|---|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 4.102799 | 0.7104845 | 0.7942905 | 0.006490523 | slide | 0.009561787 | -9.000676e-07 |
| 22.5 | linear | 0.8369781 | 0.002390851 | 0.02011354 | 0.002558277 | stick | 1.258511e-09 | 0.5112898 |
| 41.25 | previous-material | 0.07443184 | 0.000366729 | 0.001040359 | 0.0009070884 | slide | 2.803058 | -4.442645e-07 |
| 41.25 | linear | 132.9372 | 1.255141 | 0.9023262 | 1.498096 | slide | 0.5307302 | -2.235763e-06 |

## Force solutions

| Angle | Initialization | Tip X (N) | Tip Y (N) | Tip Z (N) | Contact X (N) | Contact Y (N) | Contact Z (N) |
|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 1.460516 | 0.212978 | -1.497346 | -0.672916 | -1.865394 | -6.609871 |
| 22.5 | linear | 1.000961 | 0.001130023 | -1.001875 | -0.2432369 | -1.646281 | -7.25133 |
| 41.25 | previous-material | 0.9998027 | -0.0003085858 | -1.000019 | 0.1952439 | -2.322604 | -7.769334 |
| 41.25 | linear | -0.1606544 | -0.4676807 | -0.9023535 | 0.9772018 | -2.262768 | -8.215778 |

At 22.5 degrees the poor result trades tip and contact forces against one another and sits on the friction-cone boundary with nonzero inferred sliding. The better result sticks, with about 0.511 N of cone margin. Both are approximately contact-feasible at the production tolerances, but their data fits are not comparable. Physical feasibility alone is not MAP optimality.

At 41.25 degrees both returned outputs are on the sliding branch. The good solution predicts about 2.803 mm slip; the unfinished one predicts 0.531 mm and a substantially different shape and force decomposition.

## Physical constraint residuals

| Angle | Initialization | Tip moment infinity norm (N mm) | Normal projection (N) | Tangential projection infinity norm (N) | Cone excess (N) | Gap (mm) | Normal product (N mm) |
|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 6.465304e-07 | 6.345816e-09 | 9.42133e-07 | 9.000676e-07 | 6.345816e-09 | 4.194519e-08 |
| 22.5 | linear | 1.041962e-07 | 6.010481e-10 | 1.155401e-09 | 0 | 6.010484e-10 | 4.358406e-09 |
| 41.25 | previous-material | 2.551786e-06 | 5.734981e-09 | 8.176475e-07 | 4.442645e-07 | -5.734981e-09 | 4.455698e-08 |
| 41.25 | linear | 2.955782e-05 | 6.112596e-07 | 1.140362e-05 | 2.235763e-06 | 6.112596e-07 | 5.021978e-06 |

Production thresholds are tip moment 1e-5 N mm, contact projection 1e-6 N, cone excess 1e-6 N, gap/nonpenetration 1e-5 mm, and normal-force times gap 1e-5 N mm. The poor 22.5-degree output just satisfies the force projection/cone limits. The failed 41.25-degree output violates moment, tangential projection and cone thresholds. The sampled optimizer constraints and finer reporting-only collision checks are distinct; the latter were retained in the original benchmark.

## Why the convergence flag is misleading at 22.5 degrees

The SQP sets objectiveScale=1/max(1,max(diag(J0'*J0))) once from its initial residual Jacobian, with optimization variables already scaled by their physical scales. Here objectiveScale is approximately 2.1e-9. It then compares a stationarity residual of this scaled objective with the fixed threshold 1e-5. A uniformly scaled multiplier residual threshold would correspond to about 4.7e3 in the unscaled MAP-gradient units, in the same optimization coordinates. This is not a force error bound and does not imply scaling the objective changes its mathematical minimizer.

| Angle | Initialization | Scale | Reported scaled stationarity | Threshold | Equivalent unscaled threshold | Returned status | Iterations |
|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 2.147381e-09 | 1.185417e-06 | 1e-05 | 4656.835 | stationary-and-feasible | 43 |
| 22.5 | linear | 2.137286e-09 | 9.156062e-09 | 1e-05 | 4678.832 | stationary-and-feasible | 5 |
| 41.25 | previous-material | 2.114485e-09 | 4.959271e-09 | 1e-05 | 4729.284 | stationary-and-feasible | 5 |
| 41.25 | linear | 2.11306e-09 | 0.0006345894 | 1e-05 | 4732.472 | iteration-limit | 60 |

At the bad 22.5-degree stop, the next computed step has infinity norm about 0.00309; the stopping condition does not require a small step or small remaining data-fit improvement. The previous accepted iterations still lowered the objective from 413.07 to 409.37 to 406.42 to 404.06. The solver stops as soon as its loose stationarity and physical checks both pass. Independent nonnegative-multiplier fitting also leaves a substantial unscaled residual at this point. Multiplier-fit results depend on active-set tolerances and are supporting evidence, not a general MPCC optimality proof; one fit at the failed 41.25-degree point exhausted its fit iteration budget.

## Stricter restart experiments

An instrumented copy of the existing SQP is generated only inside the ignored audit output folder. It accepts the returned state as the initial point and preserves the original objectiveScale. The objective, priors, geometry, contact equations, ODE tolerances and physical tolerances remain fixed. The stationarity threshold is reduced to 1e-9 and budgets increased to 300 iterations/3000 trials. Restarting resets trust radius, damping and merit penalty to their normal initialization values; this is not a continuation of all internal optimizer state and cannot attribute recovery to additional iterations alone.

| Angle | Initialization | Objective | Tip error (N) | Contact error (N) | Status | Additional iterations | Branch |
|---|---|---|---|---|---|---|---|
| 22.5 | previous-material | 394.7523 | 0.7150886 | 0.7840713 | stagnation | 37 | slide |
| 22.5 | linear | 16.81385 | 0.002390851 | 0.02011354 | stationary-and-feasible | 3 | stick |
| 41.25 | previous-material | 0.1334811 | 0.0003650425 | 0.001036527 | stationary-and-feasible | 2 | slide |
| 41.25 | linear | 0.1334811 | 0.0003650544 | 0.001036543 | stationary-and-feasible | 13 | slide |

At 41.25 degrees, restarting the failed linear result reaches the low-cost solution in 13 iterations. Its cost agrees with the refined previous-state result to numerical precision. This supports a budget/globalization problem, not a second solution at the failed output.

At 22.5 degrees, the bad sliding point improves only to about 394.752 before stagnating. Damping grows to 1e4, trust radius falls to approximately 1.9e-6, the proposed step shrinks to 1.29e-10, and line searches reject the step. Tightening the stopping threshold alone therefore does not cure the step-selection/acceptance problem.

## Feasible descent check

To avoid interpreting a solver flag as evidence of a local minimum, the audit projects the negative objective gradient into the equality-constraint tangent space. Small moves are retracted onto the nonlinear equalities using a root-style least-squares solve with three tangent-coordinate anchors. All original sampled inequalities are checked afterward. This is an offline diagnostic, not a new estimator or replacement result. It uses the same objective, history and ODE settings.

| Starting state | Step in scaled coordinates | Objective | Cost change | Equality infinity norm | Inequality violation | Branch |
|---|---|---|---|---|---|---|
| original | 0 | 404.0559 | -0.0002868019 | 3.907985e-14 | 1.798561e-14 | slide |
| original | 1e-05 | 404.0508 | -0.005311227 | 4.085621e-14 | 4.050894e-14 | slide |
| original | 0.0001 | 404.0057 | -0.05046997 | 2.176037e-14 | 1.731948e-14 | slide |
| original | 0.001 | 403.5601 | -0.4960118 | 6.927792e-14 | 1.887379e-14 | slide |
| original | 0.01 | 399.7084 | -4.347775 | 2.04281e-14 | 0 | slide |
| tight-resume | 0 | 394.7523 | -9.970148e-08 | 1.456613e-13 | 0 | slide |
| tight-resume | 1e-05 | 394.7517 | -0.0006580405 | 4.973799e-14 | 0 | slide |
| tight-resume | 0.0001 | 394.7458 | -0.006519611 | 4.085621e-14 | 0 | slide |
| tight-resume | 0.001 | 394.6931 | -0.05920635 | 4.440892e-15 | 4.004501e-15 | slide |
| tight-resume | 0.01 | 394.7588 | 0.006467718 | 8.171241e-14 | 2.220446e-16 | slide |

From the original bad point, a 0.001 step reduces the objective by about 0.496 while equality and inequality residuals are below 1e-12. From the stricter stalled point, the same step size reduces cost by about 0.0592. Smaller steps also reduce cost. Both remain on the sliding branch. This is direct numerical evidence of feasible local descent in the implemented problem, so there is no need to claim a second local MAP minimum to explain either output.

## Recommended next implementation work (not performed)

Keep the observation model and two-stage formulation. Review objective/constraint scaling and stationarity normalization, prevent small heavily damped steps from being mistaken for optimality, and improve trust-region/damping recovery and merit acceptance near feasible contact states. Contact-mode-aware initialization may help but should not substitute for valid convergence checks. These are proposals for a subsequent authorized solver change; this audit changes no production tolerances or algorithm.

## Reproduce

From two_stage_force_sensing, with the saved Bishop benchmark present:

```matlab
setenv('TSFS_RUN_ID','base_rotation_bishop_audit_20261007'); setup_tsfs;
audit_bishop_solutions;
audit_bishop_descent;
audit_bishop_geometry;
```

Detailed residual blocks, spatial sensor residuals, iteration traces, and feasible descent points are in results/published/base_rotation_bishop_audit_20261007. Full states and generated restart instrumentation remain in results/runs/base_rotation_bishop_audit_20261007. Original rotation results are not overwritten.
