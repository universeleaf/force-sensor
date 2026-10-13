# Successive rod linearization with full circular friction

This opt-in solver keeps Stage 1, the sensor likelihood, discrete contact modes, and circular Coulomb friction unchanged. It replaces repeated rod integrations inside fmincon with affine predictions, then checks and updates them with the original nonlinear variational IVP. The original full-IVP mode solver remains the default.

## Coordinates and local model

In scaled Cartesian coordinates z, the physical unknowns are x = diag(scale) z: initial world moment, world tip force, and each contact's normal plus two tangential force components. At a current point zbar, one augmented IVP supplies the state and analytic sensitivities. The local models are

- w_hat = w_bar + Jw (z-zbar), including the material-to-Bishop observation derivative;
- p_hat(s_i) = p_bar(s_i) + Jp_i (z-zbar);
- m_hat(L) = m_bar(L) + Jm (z-zbar);
- all sampled signed gaps and current contact displacement are affine in z.

Stage 1 arclengths and plane frames stay fixed during all mechanics iterations. The previous physical frame's mechanical shape also stays fixed. An inner iteration never advances friction history.

The FBG objective is the existing sensor-only 1e-4 * 0.5 * ||C^(-1/2)(w_hat-w_obs)||^2, with sigma = 1e-4 /mm and no force priors. This is quadratic in Cartesian z. In sliding coordinates (normal load, direction angle, displacement amplitude), the exact force mapping contains sine/cosine and products; therefore the inner problem is still a small nonlinear fmincon problem, not a linear-constraint QP or MIQP.

## Exact contact law

Separation: normal and tangential force zero, gap nonnegative. Sticking: gap zero, tangential displacement zero, normal force nonnegative, ||tau|| <= mu lambda. Sliding: gap zero, v = a [cos(theta),sin(theta)]', tau = -mu lambda [cos(theta),sin(theta)]', lambda >= 0, a >= 0. Mu=0 uses the existing exact frictionless specialization. No polygon, friction-direction freezing, complementarity relaxation, or squared-cone substitution is added.

The original mode scheduler is retained. Each frame shares one initial nonlinear expansion point among the attempted modes. Once their iterates differ, each mode requires its own relinearizations. Earlier physical-frame history is common and unchanged. The previous mode is tried first; only an actually nonlinear-validated solution meeting the existing FBG gate can stop mode search.

## Outer correction and acceptance

The affine subproblem includes an infinity-norm trust box in scaled Cartesian coordinates, |z-zbar| <= radius. Initial radius 4, maximum 64, minimum 1e-6. This is a numerical step restriction, not a physical force prior or new observation.

Each returned subproblem point receives a full nonlinear IVP evaluation with sensitivities. Actual terminal equilibrium, sampled nonpenetration, and original mode equations are evaluated by mode_residual. A scaled-objective plus L1 constraint-violation merit compares actual versus predicted reduction. The penalty is at least 1e-3 and at least 1.5 times the largest absolute inner constraint multiplier. A ratio at least 0.1 accepts a reducing step only when it stays within the trust box; a good boundary step (ratio > 0.75) can double the radius. Rejected steps halve it, except that an infeasible affine subproblem with a step at least 80% of the radius can expand it to seek feasibility. A feasible step below the stopping size can be accepted without a meaningful merit reduction.

Outer convergence requires a positive inner fmincon exit, nonlinear branch violation <= 1e-6, and scaled Cartesian step infinity norm <= 1e-4. At most 40 outer iterations are allowed per mode. Failure returns the best evaluated nonlinear candidate, with convergence false and diagnostics preserved. The outer step test is not a global optimality or exact nonlinear KKT certificate.

Existing inner settings remain: fmincon SQP, 300 iterations / 5000 evaluations, analytic gradients, optimality tolerance 1e-8, step tolerance 1e-12, constraint tolerance 1e-6. Sliding direction initialization is identical to the full solver: current geometric displacement relative to the preceding mechanical shape, with a force-direction fallback only when that displacement is nearly zero. Angle remains free during optimization.

The full-IVP and linearized trajectories keep the existing first-acceptable policy: raw FBG RMS <= 1e-6 /mm, equivalent to normalized RMS <= 0.01. Failed fit acceptance is distinct from convergence and branch feasibility. This experiment does not implement the separately discussed bound-based MIQP search.

## Modular interface and accounting

- linearized_model: original nonlinear evaluation and immutable previous-frame history.
- linearized_evaluate: affine rod outputs only; no IVP calls and no physical shape for publication.
- linearized_mode_branch: bounded successive-linearization loop around the existing exact-friction branch solver.
- mode_residual: nonlinear validation of original branch equations.
- mode_branch: optional mechanicalModel callback; default full-IVP path unchanged.
- mode_solve: optional mechanicsApproximation='successive-linearization'; common initial model and existing scheduling.

Every nonlinear candidate evaluation counts as an augmented IVP, including rejected candidates. The common initial IVP is attributed once to the first attempted mode. Affine callbacks are counted separately. Iterations sum all inner fmincon iterations across every outer correction and every attempted branch. No nonlinear truth is used for initialization, mode selection, or solver stopping.

Final states and diagnostics always use a nonlinear IVP result. Affine p/R/u fields are not exported as reconstructed shapes. The default solver and earlier result folders remain intact.

## Validation and experiment

`test_linearized_modes` verifies exact agreement at the expansion point, second-order Taylor error, analytic inner derivatives for separation/sticking/sliding, absence of inner IVP calls, and a nonlinear-validated rotation step. Full trajectories then compare both algorithms with identical noiseless observations, geometry, friction coefficients, and restored rotation checkpoint. Run `run_linearized_comparison` from experiments/inclined_wall. Use a new stamp to preserve earlier outputs.

For the linearized solver, stationarity and initialViolation inherited from the chosen inner solve describe that inner subproblem. Nonlinear convergence must be assessed using modeViolation, converged, and outerTrace; those inherited quantities are not a nonlinear optimality certificate.
