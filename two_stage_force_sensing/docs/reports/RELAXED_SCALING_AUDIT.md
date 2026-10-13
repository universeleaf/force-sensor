# Objective and constraint scale audit

Date: 2026-10-07. Scope: inspect the direct-1e-6 IPOPT/fmincon rotating-rod implementation and reevaluate saved states. No optimizer, uncertainty, constraint, initialization, or stopping setting was changed. All 118 frames were audited at their initial and final states (236 evaluations). Initial evaluations use the saved geometry/prior and exactly the recorded predecessor history.

## What the objective numbers mean

There are 24 sensor stations and two Bishop channels: 48 scalar measurements. The covariance is `(1e-6 /mm)^2 I`, even though the simulated injected noise is zero. The Stage 2 objective is:

`Phi = 0.5*sum(((w_pred-w_meas)/1e-6).^2) + 0.5*||Ftip-Ftip_prior||^2/sigma_tip^2 + 0.5*sum_i ||Fc_i-Fc_i_prior||^2/sigma_contact^2`.

Subsequent frames use sigma_tip=2 N and sigma_contact=10 N. The first frame uses 4 N and 35 N. This is a SUM, not a mean or RMS. With a scalar curvature RMS error e, `Phi_FBG = 24*(e/1e-6)^2`. For example Phi_FBG=1e6 means e=2.04124e-4 /mm. Large initial likelihood cost is unsurprising with this very tight declared uncertainty; it is not an error measured in millimetres or newtons.

Both new solvers receive `f = 1e-4 * Phi`. Published objective/FBG plots undo this multiplier and show Phi. The multiplier is a fixed numerical heuristic; it is not a statistically calibrated covariance or a full equilibration of the NLP. It multiplies sensor and force-prior terms together and preserves their relative weighting. Changing sensor sigma alone would change the estimation problem, not just its numerical scaling.

Measured examples:

| State | Curvature RMS error /mm | FBG cost | Solver total objective |
|---|---:|---:|---:|
| IPOPT 3.75 degrees, initial | 2.38949e-4 | 1.37032e6 | 137.032 |
| IPOPT 3.75 degrees, final | 1.39439e-7 | 0.466639 | 4.67020e-5 |
| IPOPT 22.5 degrees, final | 8.43405e-7 | 17.0720 | 0.00170731 |
| fmincon 33.75 degrees, failed final | 1.52816e-3 | 5.60465e7 | 5604.65 |

At the last example, the tip-prior contribution is 1.93044 and contact-prior contribution 0.353374, versus sensor cost 56,046,450.2. The failed terminal cost is a real measurement mismatch, not merely a misleading plotting convention. By contrast, a large seed cost can fall to a very good fit.

An isolated 1 N tip-prior deviation costs 0.125 at subsequent frames; 1 N contact-prior deviation costs 0.005. A 1e-6 /mm error in one sensor channel costs 0.5. These express the chosen statistical weights, not a requirement to force all residual blocks to equal cost. A global objective multiplier cannot change the relative weights.

## Current variables and constraints

Mechanical variables are base moment divided by 200 N mm, all force components divided by 1 N, and auxiliary b divided implicitly by 1 mm. This is physical nondimensionalization, not sensitivity-based preconditioning. The nine physical variables span moments and forces; b has no objective residual and is constrained through the cones.

Let lambda be normal force, tau the two-component tangent force, v the two-component displacement slip, g the gap, R=mu*lambda, and b>=0 the auxiliary slip bound. The constraint quantities supplied to the solvers are:

| Block | Solver expression | Implicit scale | Meaning of 1e-9 constraint tolerance |
|---|---|---|---|
| Terminal moment | m_tip/200 = 0 | 200 N mm | 2e-7 N mm componentwise |
| Sampled/contact nonpenetration | -g <= 0 | 1 mm | 1e-9 mm sampled gap violation |
| Normal complementarity | lambda*g - 1e-6 <= 0 | 1 N mm | 1e-9 N mm excess above relaxation |
| Circular force cone | tau'*tau - R^2 <= 0 | 1 N^2 | 1e-9 N^2, not 1e-9 N cone excess |
| Slip cone | v'*v - b^2 <= 0 | 1 mm^2 | 1e-9 mm^2, not 1e-9 mm slip |
| Friction duality gap | R*b + tau'*v - 1e-6 <= 0 | 1 N mm | 1e-9 N mm excess above relaxation |

Normal-force and b nonnegativity are variable bounds. No extra constraint row normalization is applied. fmincon `ScaleProblem=false`; IPOPT `nlp_scaling_method=none`. IPOPT still has its own KKT error/stopping conventions; disabling NLP scaling does not make the two solvers' reported optimality metrics identical.

This gives 410 constraint rows for the single-contact demonstration. Most sampled gap inequalities are inactive. Their large negative values (up to about -142 mm) mean safely separated points, not poor feasibility or an objective penalty. Constraint magnitude alone must not be used to rank numerical importance.

One explicit contact gap inequality is duplicated: the IVP collision grid already contains each contact arclength, and relaxed_constraints appends the same -g inequality again. This gives identical constraint gradients and nonunique multiplier splitting when active. It is a redundant-constraint issue, not a change to physics; it has not been removed in this audit.

## Measured derivative scales

At the IPOPT 3.75-degree final state:

| Constraint | Jacobian row norm in current optimization coordinates | Absolute multiplier |
|---|---:|---:|
| Terminal moment | 1.07 to 1.19 | max 4.51e-4 |
| Contact gap | 5.24 | 6.70e-5 |
| Normal product | 31.95 | 0.0360 |
| Force cone squared | 1.226 | 1.83e-5 |
| Slip cone squared | 1.14083e-5 | 8.16e4 |
| Friction duality gap | 3.736 | 0.0499 |

Here b=5.57563e-7 mm and ||v||=5.57494e-7 mm. Derivatives of the squared slip cone shrink with slip and b. The normal-product/slip-cone row-norm ratio is approximately 2.80e6. Across all final states, slip-cone multiplier magnitudes reach 1.08e9. Multiplier magnitudes depend on scaling; these figures are evidence of difficult numerical balance in the CURRENT coordinates, not invariant physical quantities.

The force-cone squared gradient likewise degenerates near lambda=tau=0. At the failed coarse fmincon 33.75-degree state its row norm is 0.001177, with multiplier 1.98e7. The formulation admits the correct zero-force physical case, but this does not remove numerical degeneracy near the cone apex.

For the 3.75-degree initial state, the objective multiplier has reduced Phi to 137, but the solver-objective gradient infinity norm is still 3401. The scaled Gauss-Newton diagonal spans 31.75 to 46,092, and the whitened residual Jacobian condition number is about 1003 (corresponding normal-matrix condition about 1.01e6). At the final state those diagonal scales remain essentially the same. A scalar objective multiplier does not change this condition number. These are objective-only, unconstrained diagnostic quantities; they are not the reduced constrained Hessian or an independent KKT test. A nonzero objective gradient at a constrained optimum can be balanced by constraint multipliers.

## Tolerance consistency

Both NLPs target constraint violation 1e-9 and optimality 1e-8; fmincon's step threshold is 1e-12 in its scaled-coordinate stopping rule. The IVP nominal absolute position tolerance is `200*2e-10=4e-8 mm`, with relative tolerance 2e-8. Thus a 1e-9 mm gap tolerance is tighter than the nominal absolute position integration tolerance. This does not prevent the numerical IVP map from returning an algebraically feasible point, but it is not a guarantee of that physical gap accuracy or equally accurate derivatives.

Near b=0, a squared slip-cone violation of 1e-9 mm^2 can allow about sqrt(1e-9)=3.16e-5 mm of slip. A squared force-cone tolerance has an analogous square-root effect near zero normal force. This is why a single scalar ConstraintTolerance does not impose uniform physical accuracy across blocks.

Likewise, epsilon=1e-6 in the friction work-like duality gap is not a 1e-6 N projection guarantee. On the sliding cone boundary, force-direction error squared is bounded by `2*R*epsilon/||v||`. Constraint row rescaling must carry the relaxation right-hand side and physical tolerance mapping along with it; scaling only the residual's variable terms changes the problem.

## Stage 1 distinction

Stage 1 already whitens its geometric gap/tangency residual by propagated FBG covariance plus model floors: 0.001 mm position and 1e-5 dimensionless tangency. Environment priors in this demo have position-component std 0.03 mm and normal-component std 0.001. Arc variables use one tenth of the candidate interval (with a 1 mm minimum), and environment variables use their projected prior standard deviations. Its saved residual norm ranges from 0.00103 to 0.0608; it is not the source of the plotted million-scale Stage 2 costs. Stage 1 and Stage 2 are separate estimations and their cost magnitudes should not be directly compared.

## Assessment and proposed next work

The whitening explains the large initial FBG objective. It does not by itself establish a bug. However, the fixed 1e-4 multiplier plus basic physical variable scales is NOT a well-equilibrated NLP. Small squared-cone gradients, large multiplier cancellation, objective anisotropy, a duplicated gap row, and mismatched physical meanings of the common feasibility tolerance are concrete issues in the current adapter. The earlier comparison is a test of these solver/formulation/scaling settings, not proof that either native solver is intrinsically inadequate.

Proposed sequence, not implemented: remove duplicate gap rows; select variable/constraint scaling using physical tolerances and representative active Jacobian magnitudes; reconcile NLP tolerances with IVP accuracy; reassess the squared-cone treatment near sticking/apex; then compare short continuation with direct 1e-6 using identical state/history. Keep calibrated statistical weights separate from numerical scaling. If experimental uncertainty or model error supports a different covariance, treat that as an explicit estimator-model decision, not a cosmetic fix to the plotted objective. These observations do not quantitatively attribute every failure to scaling without controlled reruns.

Evidence: `results/published/base_rotation_relaxed_scaling_20261007/{objective_scales,constraint_scales,variable_scales}.csv`. Reproduction: set TSFS_RUN_ID to base_rotation_relaxed_scaling_20261007, call setup_tsfs, then audit_relaxed_scaling. No commit or push.
