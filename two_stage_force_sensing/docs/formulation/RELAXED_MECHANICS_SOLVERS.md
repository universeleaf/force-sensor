# Direct relaxed mechanics solver contract

Status: experimental solver comparison, 2026-10-07. The default `tsfs.step` still calls the existing semismooth SQP. These solvers are called explicitly by `run_relaxed_rotation`.

## Common problem

Keep the approved geometry MAP, Bishop observations, full 3D Cosserat IVP, terminal moment condition, force priors, fixed Stage 1 normals/arclengths, and sampled nonpenetration. The mechanical centerline/contact position remains unknown. No truth is passed to the estimator. No noise is injected; declared sensor/environment covariance remains as before.

Let lambda >= 0 be the normal force, tau in R^2 the tangent force, g the gap, and v in R^2 the displacement of the same material contact arclength relative to the preceding estimated shape. With R=mu*lambda, introduce b>=0 and impose:

- g >= 0, lambda*g <= epsilon;
- tau'*tau <= R^2;
- v'*v <= b^2;
- R*b + tau'*v <= epsilon.

All quantities here are dimensionless, using force F0=1 N and gap/slip G0=V0=1 mm. The first relaxation is scalar Scholtes normal complementarity. The last is the conic duality-gap analogue for circular Coulomb friction, not a polygonal scalar-friction MPCC. At epsilon=0, the two cones imply R*b+tau'*v>=0. If R>0, equality gives the maximum-dissipation law; if R=0, tau=0 and any slip is allowed. No division by force or slip is used. The auxiliary b need not be uniquely determined at the zero-force apex. This preserves that physical case; it does not claim to remove all constraint degeneracy there.

At finite constraint tolerance the squared slip cone can permit a displacement error proportional to the square root of that tolerance. A 1e-6 duality gap is NOT a guaranteed 1e-6 N projection residual. Retain original projection, work, penetration, cone, tangency and terminal-moment diagnostics independently. They report only; no filtering, correction or step rejection is added.

## Numerical implementation

`tsfs.relaxed_constraints` builds analytic constraints/Jacobians. `tsfs.relaxed_solve` provides an identical function to both native solvers, caching by mechanical variables. Every uncached evaluation integrates the augmented variational IVP. Beta-only changes reuse the same IVP. Report all actual augmented integrations, including initialization and final evaluation. No finite-difference IVP gradient or Hessian is used in production.

Objective is the unchanged MAP cost multiplied by a fixed 1e-4 for BOTH solvers. This scalar does not change its minimizers; tolerances are on this scaled problem. Unlike the previous solver, scaling is not recomputed from the initial J'J. Moment variables use 200 N mm in this demo; force variables use 1 N. Terminal moment equality is divided by the moment scale. Gap/slip and force constraints have the normalization above. No additional automatic NLP scaling is requested.

- epsilon=1e-6 directly at every frame; no continuation or solver restart.
- Both: 300 iterations; optimality target 1e-8; constraint tolerance 1e-9.
- fmincon: SQP, supplied objective/constraint gradients, ScaleProblem=false, StepTolerance=1e-12, MaxFunctionEvaluations=5000, native Hessian updating/globalization. Positive exit 2 (small step) is distinguished from exit 1 (first-order conditions).
- IPOPT: limited-memory Hessian, nlp_scaling_method=none, acceptable_iter=0, bound_relax_factor=0, honor_original_bounds=yes. Other native defaults retained. Solver barrier progression is separate from the fixed physical relaxation epsilon.
- ODE: existing RelTol=2e-8, scaled AbsTol=2e-10. Stage 1 and diagnostic settings unchanged.
- Numerical initialization uses the prior mechanical state when the native solver reports success/positive exit and existing contact matching is eligible, otherwise its existing linear seed. For fmincon, this includes positive small-step exit 2; it is not an independent optimality certificate. b starts at norm(v)+1e-8. Neither solver reuses multipliers/Hessian across frames.
- As previously implemented, a finite unsuccessful solution still supplies the next history/prior, but is ineligible for the converged-state initialization branch. This can propagate errors. It is deliberately visible in the comparison.

## Measurement and timing interpretation

3.75 degrees uses 13 frames; 1 degree uses 46, including zero and 45 degrees. Both retain 0.02 s per frame and identical per-step prior settings. This is a quasistatic displacement-friction comparison, not equal rotation speed or equal number of sensor updates. The fine-step physical trajectory is regenerated from its actual preceding reference shape; it is not interpolated from the coarse trajectory.

Runs execute serially. Total frame time includes geometry, mechanics setup/solve and diagnostics; checkpoint/export I/O is excluded. First-frame loading/JIT costs are included. State-only IVP count is zero for the new adapters. Current custom-SQP timings are historical and mix state-only and augmented IVPs; no equivalent-accuracy speedup is claimed.

Native success, feasibility, original contact-law diagnostics, reported optimality and estimation error are separate measures. IPOPT and fmincon do not define all optimality metrics identically. Saved IPOPT iteration arrays may include additional restoration entries; CSV iteration is their zero-based trace index, while reported iteration totals come from iter_count.

## Dependencies and validation

MATLAB R2025b; CasADi 3.8.0 Windows64 MATLAB2018b archive; bundled IPOPT 3.14.19. Dependency lives only in ignored results/runs/_dependencies/casadi. Archive SHA256: E1B9AF1A0557E26FFFBFCE341C20E9D5482CE8351539F309559E32C2A7ACC20E.

Source: https://github.com/casadi/casadi/releases/download/3.8.0/casadi-3.8.0-windows64-matlab2018b.zip

`test_relaxed` checks the augmented-IVP constraint Jacobian against central differences at 0, 22.5 and 41.25 degrees and verifies the zero-force/nonzero-slip cone case. Relative Jacobian discrepancies measured 3.87e-7, 8.61e-8, 3.57e-8 with finite-difference step 1e-5. This validates the derivative implementation at that resolution, not a 1e-8 KKT certificate throughout the trajectory.

## Interpreting a fixed friction relaxation across increments

A fixed conic duality-gap tolerance is not a fixed friction-direction tolerance. On the sliding cone boundary, write theta for the angle between tau and -v. Then

`R*||v|| + tau'*v = R*||v||*(1-cos(theta))`,

and the squared force-direction mismatch is

`||tau + R*v/||v||||^2 = 2*R*(R*||v|| + tau'*v)/||v|| <= 2*R*epsilon/||v||`.

Thus epsilon=1e-6 N mm can allow roughly 1e-3 N force-direction errors when R and slip are order one. Reducing per-step displacement while keeping epsilon and its 1 mm slip scale fixed loosens this angular allowance. The exact Coulomb law is invariant to multiplying nonzero slip by a positive number, but a finite duality-gap relaxation is not. This is another reason the finer-step experiment is not guaranteed to improve convergence or physical residuals. This observation motivates future scaling/relaxation studies; no adaptive epsilon or motion normalization was added to these runs.

In sticking, the exact cone constraints imply `(R-||tau||)*||v|| <= epsilon`; the allowance grows near the stick/slip boundary. Squared-cone feasibility tolerances create an additional error source. Always inspect the original projection diagnostic, rather than equating epsilon with its force tolerance.
