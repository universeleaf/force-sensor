# Relaxed solver scaling audit handoff

Date: 2026-10-07. Author: Codex assisting the mentor. Direction: mentor to collaborator, prepared locally and not sent. Branch codex/two-stage-force-sensing; base commit b376013. No commit or push.

The mentor requested examination of the million-scale FBG objective and current constraint scaling. Audited all 118 saved frames at initialization and termination (236 states), without rerunning optimization or changing solver/model settings. New read-only entry point: experiments/base_rotation/audit_relaxed_scaling.m. Full report: docs/reports/RELAXED_SCALING_AUDIT.md. Complete CSV evidence: results/published/base_rotation_relaxed_scaling_20261007.

The displayed MAP objective is unscaled. Its 48 Bishop residuals are whitened by sigma=1e-6 /mm. A raw FBG cost 1e6 means curvature RMS error 2.04e-4 /mm; both solvers receive 1e-4 times the total MAP cost. This heuristic scalar preserves sensor/prior weighting but does not balance derivative scales. Subsequent force-prior stds are 2 N tip and 10 N contact; first-frame values are 4 N and 35 N.

Current variables scale base moment by 200 N mm and forces by 1 N; b implicitly uses 1 mm. Constraint blocks effectively use 200 N mm for terminal moment, 1 mm for gaps, 1 N mm for complementarity/duality gaps, 1 N^2 for force cone and 1 mm^2 for slip cone. No additional solver NLP scaling is enabled.

Measured at coarse IPOPT 3.75-degree termination: normal-product Jacobian row norm 31.95 vs slip-cone 1.14e-5; slip multiplier 8.16e4. Initial solver cost 137 still has gradient norm 3401 and Gauss-Newton diagonal 31.75 to 46092. Residual Jacobian condition ~1003. Across terminal states slip multipliers reach 1.08e9. Near-zero force states also weaken the squared force-cone gradient. Contact gap inequalities are duplicated because collision sampling includes the contact arclength before a separate -gap row is added. Nominal ODE absolute position tolerance 4e-8 mm is above the NLP's 1e-9 mm gap tolerance; squared-cone tolerances have square-root physical implications near their apex.

Conclusions: large seed objective alone is expected from whitening, but the adapter has substantive numerical balance/redundancy/tolerance issues. A large TERMINAL cost, such as failed fmincon 33.75-degree sensor cost 5.60e7, remains a real mismatch. Current experiments do not isolate how much each scaling issue causes failure. Do not present the current comparison as a verdict on the best possible performance of native solvers.

Next owner: mentor chooses whether to authorize numerical scaling/constraint redundancy changes and a controlled rerun. Proposed priorities: remove duplicate gaps, equilibrate variables/active constraint rows with explicit physical tolerance mapping, align IVP and NLP tolerances, examine sticking/apex cone treatment, then continuation. Preserve sensor covariance unless a separate physical/statistical change is approved. This audit changes no estimation behavior.
