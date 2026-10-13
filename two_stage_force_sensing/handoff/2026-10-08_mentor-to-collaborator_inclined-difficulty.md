# Inclined-wall difficulty audit handoff

2026-10-08. Author: Codex assisting mentor; direction mentor to collaborator, local and unsent. Branch codex/two-stage-force-sensing, base b376013; uncommitted/unpushed. Scope: diagnose why the current inclined-wall solve remains hard. No production solver, geometry, observation, noise, or physics changes.

[Full audit](../docs/reports/INCLINED_WALL_DIFFICULTY_AUDIT.md). Evidence: results/published/inclined_wall_difficulty_20261008. Source: experiments/inclined_wall/audit_inclined_difficulty.m, replay_mu0_failure.m, audit_inclined_seeds.m. Reproduction order is in the report; instrumented solver copies live only in ignored results/runs/inclined_wall_difficulty_probe.

Key evidence: physical six-force condition number 65-111 versus earlier planar initial 273, weakest direction in XZ rather than Y. Current friction scalar-cone/duality gradients nearly dependent; near-zero force collapses cone gradients. Direct epsilon=1e-6 at 23 N implies nominal positive gap <=4.3e-8 mm. Previous-state warm starts have roughly 2 mm gap violations on insertion and large product violations on withdrawal. Several solves reach feasibility in <10 iterations then spend nearly 5000 function evaluations making little progress.

Controlled tests preserve geometry, previous estimated shape/history, sensor cost, constraints, and solver settings. Only a diagnostic initial state is constructed from true loads. Frames 6/15/34 tip errors fall from 1.462/23.547/3.120 N to 4.17e-5/0.001359/3.16e-5 N in 4/7/3 iterations, with constraint violations <7e-14. These are accurate feasible diagnostic points, not production results or certified global optima. Original scores and video remain unchanged. The failures cannot be explained as unavoidable geometry inconsistency or equal-quality solutions.

The unchanged mu=0 replay logs a trial tip Y force 10230 N and base moment components up to 1.56e6 N mm after iteration 77. That trial triggers the IVP RHS cap; callback exception propagates. No exact mu=0 reduction or trial-rejection fix was implemented. Existing approval request for that formulation reduction remains unresolved; this audit request is not treated as approval.

Next decision belongs to mentor: initialization/contact-feasibility predictor; exact mu=0 reduction and IVP-domain handling; contact constraint representation/continuation and physical tolerances. Do not substitute diagnostic truth seeds in the estimator, add force priors, or force 2D as an unapproved fix. No commit/push or external message was sent.
