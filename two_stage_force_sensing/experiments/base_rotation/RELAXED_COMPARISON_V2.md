# Revised relaxation experiment: no force priors

Requested by the mentor on 2026-10-07. The previous four benchmark runs remain preserved. The default `tsfs.step` implementation is not replaced.

## Exact changes

- Remove tip-force and contact-force prior residuals from Stage 2. Its objective has exactly 48 Bishop sensor residuals in this demo; no force penalties. The existing previous-state seed, contact matching, and preceding-shape friction history remain. The linear initializer's regularization is an initialization device, not a force objective penalty.
- Interpret the requested FBG covariance change using the preceding discussion's standard-deviation convention: sigma=1e-4 /mm, covariance=1e-8 I. Apply to current/previous FBG packets in BOTH stages. The sensor values remain identical and noiseless.
- Set native nonlinear constraint tolerance to 1e-6 for both fmincon and IPOPT. Keep optimality tolerance 1e-8, fmincon step tolerance 1e-12, 300 iterations, epsilon=1e-6 directly, and objective multiplier 1e-4. No continuation/restarts.
- Omit the explicit contact-gap inequality because the sampled gap constraints already include every contact arclength. Assert that this sampling condition holds. No other constraint is removed, replaced, or rescaled.
- Keep all variable/Jacobian-row scaling, IVP tolerances, cone expressions and native solver choices as before.

## Diagnostics

Diagnostics remain reporting-only. `flags` now checks gaps at 1e-6 mm, terminal moment components at 200*1e-6=2e-4 N mm, squared force/slip cone residuals at 1e-6 in their respective normalized units, and normal-product/friction-duality-gap upper bounds at epsilon+tol=2e-6 N mm. Nonnegativity and overall relaxed feasibility are also reported. Tangency and standardized FBG mismatch thresholds remain unchanged.

Original contact-law residual values are still written in every frame row. Original stricter physical warnings remain separately in `exactLawFlags`; they are not relabeled as successes or deleted. `relaxedFeasible` refers to the sampled NLP and bounds; a finer-grid penetration diagnostic can still warn independently. A positive native exit, sampled NLP feasibility, fine-grid physics checks and force-estimation accuracy are distinct results.

## Interpretation

With only FBG residuals and uniform sigma, changing sigma scales the whole Stage 2 objective at FIXED geometry/history. It does not change that exact problem's minimizer, though stopping/globalization can change. Stage 1 still balances the FBG and environment uncertainties, so its inferred geometry can change. Four settings changed together; this is a combined experiment, not a causal ablation of each one.

At a fixed state, FBG covariance does not enter the mechanical constraint Jacobian. The validation explicitly confirms unchanged constraint values/Jacobians after the covariance change; the whitened observation Jacobian divides by 100 and the sensor objective/gradient by 10,000. Constraints can have different Jacobian-row magnitudes at the NEW solved states, so the post-run audit still evaluates every seed and terminal state.

## Reproduction

From the package directory, with the previously downloaded package-local CasADi dependency and the existing two reference trajectories:

```matlab
setenv('TSFS_RUN_ID','base_rotation_relaxed_v2_validation_20261007');
setup_tsfs;
test_relaxed_v2;
run_relaxed_rotation_v2('all');
export_relaxed_rotation_v2;
```

The runner uses separate `base_rotation_relaxed_v2_375deg_20261007` and `base_rotation_relaxed_v2_1deg_20261007` run folders. Repeating this command overwrites those v2 run files; it never overwrites v1. The original source versions are backed up in ignored `results/runs/base_rotation_relaxed_v2_source_before_20261007`. For another formulation change, select new run identifiers.

Generic implementation: optional configuration in `tsfs.relaxed_solve`, duplicate-gap option in `tsfs.relaxed_constraints`, reporting adapter `tsfs.diagnose_relaxed`. Experiment configuration/measurement override is in the rotation runner. The original runner call without configuration keeps its previous settings and equations for reproducibility.
