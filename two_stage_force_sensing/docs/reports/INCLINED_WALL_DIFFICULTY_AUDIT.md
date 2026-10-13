# Why the inclined-wall optimization is still difficult

Date: 2026-10-08. Offline diagnostic audit of the unchanged inclined-wall run. Branch codex/two-stage-force-sensing, base b376013; no commit or push. Original trajectories, solver code, video, scores, geometry and sensor formulation are unchanged.

## Finding

The dominant observed problem is the numerical path from the current initialization through nearly degenerate contact constraints. The new contact geometry improves physical force identifiability. Accurate feasible solutions exist for the failed frames with exactly the same frozen geometry, objective, constraints and friction history. Small geometry errors and weak sensor directions still matter, but they do not force the large reported errors.

The evidence does not establish that every failure has one cause, nor prove global optimality or the existence of multiple local minima. The controlled initializations below are diagnostics using simulation truth, not a deployable estimator or replacement performance result.

[Summary figure](../../results/published/inclined_wall_difficulty_20261008/difficulty_audit.png). [All audit data](../../results/published/inclined_wall_difficulty_20261008).

## Controlled test: same problem, different initialization

At frames 6, 15 and 34, use known reference forces only to construct an initial state. With the original Stage 1 frozen arc/orientation/offset, solve the three base-moment components to enforce terminal equilibrium, then run the same fmincon SQP optimization. Keep the actual previous estimated shape as friction history. Keep all original settings, objective weights and constraints. No reference force term or additional constraint is added.

| Frame / distance | Original tip error N | Diagnostic tip error N | Original sensor cost | Diagnostic sensor cost | Diagnostic SQP iterations / IVPs |
|---|---:|---:|---:|---:|---:|
| 6 / 70 mm insertion | 1.46159 | 0.00004173 | 1.64550 | 5.746e-8 | 4 / 8 |
| 15 / 52 mm insertion | 23.54672 | 0.00135911 | 7005.67717 | 8.921e-5 | 7 / 11 |
| 34 / 66 mm withdrawal | 3.12039 | 0.00003155 | 2.88611 | 4.727e-8 | 3 / 6 |

All three diagnostic results have relaxed violations below 7e-14 and no reporting flags. They exit on small steps, with first-order optimality 3.85e-8 to 6.51e-8, slightly above the requested 1e-8; they are demonstrably better feasible points, not certified global optima. Their mechanics times were 0.830, 0.197 and 0.100 s, including solver overhead; the first has cold-start overhead. These timings are diagnostic and must not replace original benchmark timing.

The original worst frame is not constrained to an inaccurate result by frozen geometry. Its sensor objective falls by almost eight orders of magnitude with a different initial state. Likewise, the near-zero-contact solution at frame 6 is not an equally good sensor fit: its objective is about 2.9e7 times the diagnostic value.

## Physical force sensitivity is better than before

Let a be the initial moment, F=[F_tip; F_contact] the six physical world-frame forces, h(a,F)=0 terminal equilibrium, and w(a,F) the 48 Bishop observations. Eliminating the initial moment gives

G = dw/dF = w_F - w_a (h_a)^(-1) h_F.

This is a raw measurement sensitivity in (/mm)/N, using equal physical N units for tip and contact forces, not a mixed-unit full optimization Jacobian. On the 35 true-contact frames, cond(G)=64.6 to 111.3. The earlier planar sliding case started at 273.2 under the same calculation. At closest approach it is 79.9. Along the local exact sliding-contact tangent space N, cond(GN) falls from 50.1 at first contact to 23.1 at closest approach; the full insertion/withdrawal range is about 21.5 to 80.9.

The distal free length increases from 65.1 to 99.8 mm, with 8 to 12 sensors distal to contact. The largest tail distance from the contact-to-tip chord grows from 6.85 to 15.68 mm. This helps distinguish the forces. The weakest unconstrained force-exchange direction remains aligned approximately with that chord and lies in the XZ bending plane. Its Y fraction is below 5e-15. Thus the worst frame's large Y error is not evidence that the correct planar reference has a new Y singularity.

Condition number alone does not describe signal size. The minimum singular value of G is 3.71e-5 to 6.04e-5 (/mm)/N. A unit-N combined tip/contact perturbation in that local weak direction changes the 48-channel RMS by only about 5.4e-6 to 8.7e-6 /mm. With the chosen sigma=1e-4 /mm, that is a weak sensor signal. Finite covariance still sets numerical cost scale even though no noise was injected. Uniformly scaling a sensor-only objective does not change its exact minimizer, but affects stopping/merit numerics.

Sensitivity derivatives were checked against central finite differences at four frames; maximum relative discrepancy 1.88e-6. The independent reference and mechanics IVP agree to at most 1.28e-5 mm in shape and 5.06e-10 /mm in sensor RMS during this audit. These discrepancies are numerical, not injected noise.

## The initial state is often far from contact feasibility

The existing Bishop warm start copies the previous converged initial moment and forces when contact matching succeeds. It does not first correct the state for the new base translation and contact arclength. For a 2 mm decrease in perpendicular distance, many initial states penetrate by approximately 2 mm. On withdrawal, the corresponding positive gap can produce very large lambda*g: initial maximum violation is 34.18 at frame 23 and 22.28 at frame 27. The product target is only 1e-6 N mm.

A prior state can be accurate at its own actuation and still be a poor feasible seed for the next one. At frame 6 the solve collapses onto almost zero contact force: lambda=1.01e-6 N, compared with reference 2.163 N, while the slip epigraph beta grows to 6.419 mm versus actual slip 3.131 mm. It exits on a small step with a 1.462 N tip error. At frame 34 the estimated normal force is 0.01291 N versus reference 3.01596 N. These states fit the FBG at RMS 2.62e-5 and 3.47e-5 /mm, below sigma, yet have substantially worse likelihood than the accurate diagnostic states.

The initial repair is only part of the cost. At frame 11 feasibility is first reached at iteration 7 / function count 16, but the solve reaches function count 5000 without finishing. Frame 23 first reaches feasibility at iteration 8 / count 17 and also exhausts 5000 evaluations. Repeated small or rejected steps after reaching feasibility dominate these examples. The original mean is 853 augmented IVPs for 85 SQP iterations; it is not one IVP per iteration. Derivative and line-search evaluations each integrate an augmented IVP, subject to the existing cache.

## Friction constraints become nearly linearly dependent

Set R=mu*lambda and v=the two-dimensional tangential displacement. The solver uses

h_F = tau' tau - R^2 <= 0,
h_V = v' v - beta^2 <= 0,
h_D = R*beta + tau' v - epsilon <= 0.

At exact nonzero sliding, beta=||v|| and tau=-R*v/beta. Their gradients satisfy

grad(h_D) = -beta/(2R) grad(h_F) - R/(2beta) grad(h_V).

For epsilon=0 all three constraints are active and this is a constraint-qualification degeneracy. With positive epsilon, the duality constraint need not be active at the exact law; however, current iterates often lie on its relaxed boundary near the other two boundaries, retaining severe near-dependence. After normalizing the three row lengths, the smallest singular value is 2.06e-7 at frame 5 and about 6e-4 at closest approach. Row scaling alone cannot remove this angular dependence.

At small normal force the force-cone gradient also collapses. At frame 6 its row norm is 6.35e-7, while the slip-cone row norm is 27.19. The largest nonlinear inequality multiplier is 908 at that frame and about 1.27e7 at the infeasible frame 15. These numbers depend on the current scaling but reveal difficult QP/KKT systems. This is structural numerical evidence; the seed replays do not isolate the causal contribution of each individual constraint.

## Small geometric errors meet extremely narrow admissible gaps

For positive lambda, the nominal normal-product constraint permits

0 <= g <= epsilon/lambda.

At lambda=23.145 N and epsilon=1e-6 N mm, the nominal positive-gap interval is at most 4.32e-8 mm. Allowing the 1e-6 feasibility tolerance on the product row doubles that upper bound; it remains tiny. Negative-gap tolerance and the product tolerance act asymmetrically. The gap and product gradients are nearly parallel near contact: grad(lambda*g)=lambda*grad(g)+g*grad(lambda). The previous duplicate gap inequality is removed; this is near-dependence of distinct physical constraints, not an accidentally repeated row.

Stage 1's errors are small in physical units but much larger than this interval. Evaluating the true forces with frozen estimated geometry and re-solving base moment gives frame 15 a +0.00752 mm contact gap and normal-product violation about 0.0963, despite only 0.00924 mm shape RMS error. At closest approach the analogous gap is -0.03422 mm. The optimizer must deform the shape or redistribute forces to repair these mismatches.

Nevertheless, the controlled replays show that the same frozen geometry admits accurate feasible force estimates at all three tested failures. Geometry mismatch makes the numerical path harder; it is not a demonstrated reason that a 23 N error is unavoidable. The history-point discrepancy preceding frame 15 is only 0.000181 mm; preceding frame 34 it is 0.000022 mm. Large prior-history error does not explain those two failures. At other frames previous errors can contribute; frame 37's reference-load friction residual is 1.04e-7 with true history versus 3.76e-6 with estimated history.

## The mu=0 crash has a directly observed numerical mechanism

An isolated instrumented copy of our unchanged adapter reproduced tsfs:RHSBudget. It logs only; the production adapter is untouched. After accepted iteration 77, with objective 0.05530 and violation 2.14e-6, an optimizer trial proposed:

- Tip force [1.557, 10230.202, 3.125] N.
- Base moment [-1555426.106, 423.006, 731680.955] N mm.
- Local contact force [lambda,tau1,tau2]=[0,5.4374,1.5669] N, grossly outside the mu=0 cone.

That trial exceeded 100000 RHS calls in one augmented IVP. The adapter had completed 1292 augmented IVPs before the failed call. The callback exception propagates and aborts fmincon; it is not caught as a recoverable rejected trial. This is an optimizer step and callback-domain robustness failure, not a physically required load.

At mu=0 the inequality tau' tau<=0 has zero derivative at its only feasible tangential force tau=0. The unpenalized beta becomes physically irrelevant, while the remaining scalar duality inequality tau'v<=epsilon cannot provide both missing tangential directions. This representation is degenerate. The exact mu=0 reduction remains proposed, not approved or implemented.

## Recommended next priorities, not implemented

1. Improve initialization with a mechanics/contact-feasible predictor for the new actuation, preserving the current physical model and measurement contract. The oracle-seed tests establish the potential benefit, but do not design or validate a deployable predictor.
2. Address the contact representation and globalization together: exact mu=0 variable elimination; robust handling of invalid IVP trials; a friction formulation or continuation strategy that does not operate immediately on nearly dependent scalar boundaries. Compare on fixed problems before a trajectory rerun.
3. Choose product/geometry tolerances against a physical gap target and the Stage 1 model error, rather than assuming one numeric epsilon controls all physical errors. Any such change needs explicit approval and a documented physical interpretation.

Do not treat these results as evidence to add truth seeds, force priors, a planar restriction, or silently reject prior states in production. Do not change the rod physics merely to eliminate the numerical failures. Further implementation remains subject to the user's decision.

## Reproduction

From the package root in MATLAB: setup_tsfs; addpath('experiments/inclined_wall'); audit_inclined_difficulty; replay_mu0_failure; audit_inclined_seeds. The first reads saved outputs and performs derivative/state evaluations; the second instruments a copy of our adapter in ignored results/runs/inclined_wall_difficulty_probe; the third overrides only initialization in another isolated copy. Neither copy modifies the production solver. The original mu=0 reduction has not been implemented. CSVs and source hashes are in results/published/inclined_wall_difficulty_20261008; raw replay MAT files are in the ignored probe directory.
