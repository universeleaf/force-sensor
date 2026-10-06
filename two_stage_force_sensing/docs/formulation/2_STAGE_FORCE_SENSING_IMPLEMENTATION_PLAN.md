# 2 stage force sensing implementation plan

Historical proposal, preserved from 2026-10-05. The estimator was subsequently implemented; see [current results](../reports/IMPLEMENTATION_AND_RESULTS.md). The original text below records the pre-implementation plan.

Design proposed on 2026-10-05, following the measured [current solver audit](../reports/CURRENT_SOLVER_BOTTLENECK_AUDIT_2026-10-05.md). This is an implementation and validation plan; the new estimator has not been implemented or benchmarked. The governing equations remain in [2 stage force sensing](2_STAGE_FORCE_SENSING.md).

## Framework and scope

One physical time step calls geometric estimation once, mechanical estimation once, and a reporting-only diagnostic function once. Numerical iterations inside either solver are allowed. There is no diagnostic-triggered retry, geometry correction loop, mode substitution, frame rejection, or skipped output.

Use three-dimensional position, rotation, curvature and force throughout. Support any admitted number of candidate contacts, including zero, and multiple environmental planes. Contact count, ordering, forces, and contact locations must not come from demo truth. Multiple contacts can share a surface. Use stable contact identities and transform force priors consistently when tangent bases change.

Use a causal step API with timestamps and a predecessor reference, not an assumed fixed offline window. The saved sliding demo's supplied predecessor samples are not generally its preceding displayed frame. Process the supplied history in time order or explicitly identify reconstructed history; never silently substitute the previous output frame. Missing predecessor information permits force-cone feasibility, but not a claimed sliding-direction observation.

The initial geometry representation is a collection of planes or identified smooth local surface patches, with a separate nonpenetration query. This is not a claim to handle arbitrary corners, distributed contact, or surface topology without additional modeling. General 3-D geometry also requires adequate twist/base-pose information; two bending channels alone do not provide arbitrary twist observability.

## Stage 1 solver

Choose MATLAB `lsqnonlin`, `Algorithm='trust-region-reflective'`, for the profiled geometric MAP residual. The unknown vector has one arc length per candidate and two normal-chart coordinates plus one offset per environmental plane: C+3P variables before removing known parameters. Bounds keep arc lengths inside their candidate neighborhoods and normal charts inside their valid regions. Distinct contact branches get ordered neighborhoods; genuinely ambiguous candidates are reported rather than silently declared resolved.

The residual is the concatenation of whitened closure/tangency residuals, environmental deviations, and an optional tracking prior. Reconstruct shape and its kinematic sensitivities once for the current sensor packet. Evaluate position, tangent and propagated joint covariance at trial arc lengths without Cosserat mechanics. Preserve correlations across candidate contacts and between closure and tangency. Recompute the small residual covariance as geometry changes and account for this dependence in its Jacobian; do not change weights invisibly between objective evaluations.

Compute the profiled quadratic cost using Cholesky/QR solves, not explicit matrix inverses. If its covariance is rank deficient, preserve deterministic consistency conditions in the unsupported subspace; a pseudoinverse alone must not silently discard a nonzero deterministic residual. Use a justified model-error covariance only when one has been declared. Rank failures are diagnostic events, not grounds for inventing sensor noise.

Initial settings, on dimensionless variables and whitened residuals:

| Setting | Initial value |
|---|---:|
| MaxIterations | 30 |
| MaxFunctionEvaluations | 300 |
| FunctionTolerance | 1e-8 |
| OptimalityTolerance | 1e-6 |
| StepTolerance | 1e-8 |
| SpecifyObjectiveGradient | true |

These are proposed starting settings, not measured optimal values. MAP residuals are allowed to be nonzero because the measurements are uncertain. Solver convergence is distinct from whether the geometric hypothesis is statistically consistent.

Output estimated normals, arc lengths and plane offsets, plus geometric points, uncertainty diagnostics, candidate identities and solver status. Freeze normals, arc lengths and offsets during Stage 2; do not fix the mechanical world contact points.

## Stage 2 variables and mechanics

Use x=(m0,f_tip,{fn_i,tau_i}), with tau_i in R^2 and 3C+6 variables under the current zero-tip-moment point-force model. The contact point is p_c,i=p(s_hat_i;x), obtained from the solved rod. Eliminating this equality from the numerical variable vector does not make the point known or fixed.

Reuse the lifted 3-D IVP architecture in `integrate_cosserat_load_state.m`. Optimize the base moment and enforce the terminal moment outside the IVP. Never call the old inner `fsolve` from the new estimator. Initialize forces by an observation-only moment fit and transport the previous estimate where available; this fit is an initializer, not a second likelihood.

Integrate variational sensitivities alongside the state, including force jumps at fixed material arc lengths. Since Stage 2 fixes arc lengths, their moving-boundary derivatives are absent. Differentiate FBG interpolation and the projection laws consistently. A state-plus-sensitivity integration counts as one augmented IVP, but its state dimension and RHS effort must also be logged; an IVP count alone is not a fair speed comparison.

Start with ODE relative tolerance 2e-8 and scaled/componentwise absolute tolerance corresponding to 2e-10 in dimensionless states. Validate these against a tighter reference before changing them for speed. Use a 0.5 mm collision grid for the requested benchmarks, plus contact points and model discontinuities. A denser 0.1 mm grid belongs to offline diagnostics and does not feed back into the solve.

## Semismooth SQP

Use a custom outer method rather than `fmincon` around nonsmooth expressions. Retain the exact normal half-line and tangential disk projection residuals:

\[
a_i=\max(0,f_{n,i}-\rho_n g_i),\quad
r_{N,i}=f_{n,i}-a_i,\quad
r_{T,i}=\tau_i-\Pi_{D(\mu_i a_i)}(\tau_i-\rho_t v_i).
\]

Start with rho_n=rho_t=1 N/mm for these benchmarks. Express them as F_scale/L_scale for other calibrated unit scales. They are numerical projection parameters, not friction coefficients, penalties on penetration, or measurement uncertainties. The zero sets are unchanged by positive choices, although conditioning changes.

At each SQP iteration:

1. Evaluate the MAP residual, terminal equilibrium, projection residuals and whole-rod nonpenetration, with a shared cached IVP and its sensitivities.
2. Assemble an ordinary residual Jacobian for smooth terms and a consistent generalized Jacobian for the projections, including derivatives through the radius mu*a. Implement the zero-radius branch explicitly. At switching boundaries use deterministic selections and record branch changes; do not normalize a zero vector.
3. Build a positive-definite regularized Gauss-Newton model in scaled coordinates. Start relative Hessian damping at 1e-6 and adjust it according to model agreement.
4. Solve the local convex QP with MATLAB `quadprog`, `Algorithm='active-set'`. Use the previous QP working point where applicable. Initial QP settings: ConstraintTolerance=1e-9, OptimalityTolerance=1e-8, MaxIterations=200.
5. Globalize with a trust region and an L1 constraint merit function evaluated using the original nonlinear projections. Start dimensionless trust radius at 1, shrink by 0.5, and expand by 2 when agreement supports it. Maximum SQP iterations: 60; maximum nonlinear trial evaluations: 300.

The local QP may use nonnegative elastic slacks to avoid failing when the linearized constraints are inconsistent. Penalize their L1 norm and drive them below feasibility tolerance. These are temporary numerical devices. Persistent slack means the original mechanical problem has not converged. It must be reported as such; it is not a new compliant contact law or successful relaxed MPCC solution.

Generalized-derivative linearization inside SQP does not replace the circular friction disk by a polygon. All acceptance and final residual checks evaluate the exact disk projection. There is no Scholtes tau schedule, friction-generator vector, or mode-polishing pass.

Avoid unnecessary duplicate equality rows at inactive contacts. Factorizations should expose rank deficiency; use regularization and elastic QP feasibility rather than claiming that projection removes all MPCC degeneracy. Small steps or small objective changes alone do not certify convergence.

## Initial physical tolerances

Success requires stationarity and original-model feasibility, not only the MATLAB QP exit flag. Report physical residuals separately from scaled solver norms.

| Quantity | Proposed initial criterion |
|---|---:|
| Scaled generalized stationarity residual | <= 1e-5 |
| Scaled relative step used to detect stagnation | <= 1e-7 |
| Terminal moment infinity norm | <= 1e-5 N mm |
| Normal projection residual | <= 1e-6 N |
| Tangential projection residual infinity norm | <= 1e-6 N |
| Negative gap on the solver collision grid | <= 1e-5 mm |
| Negative normal force | <= 1e-8 N |
| Circular-cone excess | <= 1e-6 N |
| Normal force times gap, independent diagnostic | <= 1e-5 N mm |
| Force threshold for active-contact diagnostic labels | 1e-3 N |
| Active-contact tangency warning | absolute n^T t > 1e-3 |
| Whitened FBG RMS warning | > 4 |

The active threshold only labels diagnostics; it does not delete a force or change the projection law. Tangency is diagnosed after Stage 2, as agreed; it is not an added correction equality. The normalized projection residuals and independent gap/product checks together avoid allowing a projection scale to hide a physical violation. Construction tolerance is not a solver tolerance, nor automatically the standard deviation of a Gaussian environment prior.

## Reporting without interrupting the sequence

The diagnostic function consumes an immutable estimate and returns residuals and flags. It cannot change state, forces, covariance, solver settings, or execution flow. Output flags include geometric MAP stationarity, geometric mismatch, candidate ambiguity/truncation, mechanical convergence, gap/tangency/equilibrium errors, cone and maximum-dissipation residuals, observation fit, and local force-separation warnings.

A capped or stalled solver returns its last accepted finite iterate with `converged=false`; this is not silently certified as feasible. If no finite iterate can be produced, return an explicitly invalid output record rather than fabricated forces, and continue the dataset loop. This numerical exception policy is distinct from diagnostic warnings and from the normal solver termination criteria. No diagnostic flag triggers another Stage 1 or Stage 2.

## Validation and benchmark sequence

First validate kinematic/IVP derivatives and projection derivatives away from switches, plus directional/generalized behavior at zero force, sticking/sliding boundaries, and mu=0. Verify maximum dissipation, contact creation/loss, multiple contacts, and zero-contact states. Verify that diagnostic-only warnings leave estimates and subsequent control flow unchanged.

Then run the exact regenerated 12-frame `sliding_clean` and 12-frame `s_channel_two_contact` inputs used in the audit. Adapt the latter to 3-D sensor/environment inputs and infer candidates from observations; do not pass its known truth contact count/order to the new core. Preserve actual measured curvature channels, covariance floors, base pose, timing and environment observations. Keep existing planar results as a specialized reference, not an algorithmically equivalent baseline.

Also replay the same full 3-D two-contact packet to provide a closer framework comparison. Compare causal and offline-window results with their different information sets explicitly identified; do not label a two-frame batch time as causal single-step latency.

Use the same new core on a genuinely spatial frictional multi-contact case and a three-contact case. A rigid rotation of a planar case checks coordinate consistency but does not establish general spatial loading. Repeat selected spatial tests under rigid coordinate transforms. Existing known model assumptions such as quasistatic point contact remain explicit.

For every reported frame retain Stage 1/Stage 2/diagnostic time, stage iterations, QP iterations, trial rejections, state and augmented IVP counts, ODE segments, RHS counts, solver exits, physical residuals, inferred active contacts, force and arc-length errors, and shape error. Separate first-frame and recurrent-step timings. Use repeated sequential runs after warm-up for timing statistics, retaining an explicit cold-start measurement; do not run competing timed solvers concurrently.

Compare the main path with the original baselines and a fixed-geometry ablation where necessary to distinguish geometric estimation error from mechanical solver error. No baseline-derived or true geometry is permitted in the main new estimator. Preserve failures and flagged frames in all tables. Initial runtime goals are substantially fewer mechanics evaluations and lower median/p95 time, not a promised real-time rate or a predetermined force-error threshold obtained by tuning against truth.

## Main concerns

- Frozen Stage 1 geometry can make the Stage 2 hard constraints incompatible with the sensed shape. Reporting-only diagnostics expose this but cannot repair it. Do not quietly soften the physical model to make a table look successful.
- Contact projection is nonsmooth and the full problem is nonconvex. Semismooth SQP can stall at a degenerate branch; regularization does not guarantee global convergence or uniqueness.
- Both stages reuse FBG information. Do not treat the Stage 1 shape or covariance as an independent additional observation in Stage 2. Conditional force covariance remains different from a calibrated joint posterior.
- Candidate detection and contact identity changes are separate from force optimization. Narrow candidate neighborhoods improve speed but can miss a moving contact. Corner and endpoint contact need appropriate geometry conditions rather than smooth-interior tangency by default.
- Missing twist or predecessor information limits observability. General 3-D equations cannot recover information absent from the sensors.
- The most precise clean-demo errors may increase while runtime improves. Report the resulting tradeoff, physical consistency, and uncertainty rather than targeting machine-precision force errors through relaxed acceptance thresholds.

## Solver references

The chosen MATLAB subsolvers and options are described in the official [lsqnonlin documentation](https://www.mathworks.com/help/optim/ug/lsqnonlin.html) and [quadprog documentation](https://www.mathworks.com/help/optim/ug/quadprog.html). The normal/ball projection contact law and semismooth treatment are described in the [GetFEM contact documentation](https://getfem.readthedocs.io/en/latest/userdoc/model_contact_friction.html). Numerical values in this plan are proposed project settings, not recommendations copied from these sources.
