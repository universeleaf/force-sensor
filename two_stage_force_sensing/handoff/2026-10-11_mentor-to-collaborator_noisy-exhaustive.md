# 2026-10-11 mentor to collaborator: noisy exhaustive mode and iteration audit

Author: Codex for mentor. Local handoff, not sent. Branch codex/two-stage-force-sensing; base b376013. No commit or push.

The user requested a comparison of long versus converged attempts, a reasonable iteration limit, and explicit evidence whether exhaustive mode search reduces the large force RMS at 2 mm and 5 mm noise. Ran all four full recurrent trajectories with mu=0.3, identical saved noise packets, Stage 1, full cone, linearized rod/nonlinear correction, gate 1.24, and original 40 outer / 300 inner / 5000 evaluation limits. No default solver or physics change.

New run_noisy_exhaustive.m configures exhaustive selection and reuses noisy_linearized_20261011 inputs. Rotation restores its own exhaustive insertion frame 21. run_inclined_modes.m now optionally accepts estimatorMus (default remains [.3 0]) and reportFrameStart. analyze_noisy_mode_search.py validates full saved states, compares policies, audits mode candidates and iteration prefixes, and reproduces plots. All changes stay in our package. Original mu=0 runs are preserved; this experiment addresses mode selection with the correctly specified friction coefficient.

Tip/contact force RMSE in N, conditional -> exhaustive:
- 2 mm insertion: 4.495/5.642 -> 3.335/3.793; mean time 1.906 -> 28.934 s; convergence 39/41 -> 36/41.
- 2 mm rotation: 6.162/9.495 -> 4.985/8.800; mean time 26.857 -> 27.632 s; convergence 24/24 -> 24/24.
- 5 mm insertion: 8.068/9.762 -> 6.743/7.735; mean time 2.188 -> 36.454 s; convergence 41/41 -> 36/41.
- 5 mm rotation: 16.598/20.531 -> 13.283/17.312; mean time 15.366 -> 30.414 s; convergence 23/24 -> 22/24.

All selected states are feasible at the implemented 1e-6 tolerance. Exhaustive fallback ranking intentionally retains existing behavior: a feasible nonconverged candidate can beat a converged candidate on objective. Twelve selected exhaustive states hit the outer limit. Five-millimetre rotation shape RMS worsens despite lower force RMS. A full report lists all flagged frames, fit acceptance and mode identification.

The audit covers 264 saved states, 546 attempts and 9259 outer iterations. MATLAB and analysis exit successfully; core source hashes match the previous experiment. Eighty-nine exhaustive failed branches cycle through exactly repeated model/radius subproblems; none later converge or are selected. Every eventual returned candidate was already present before the first repeat. Post-repeat work totals 3123.19 s of 4074.03 s measured exhaustive solve time. This is retrospective potential saving, not a measured guarded run.

Recommendation: preserve 40 outer / 300 inner ceilings and detect repeated model/history/mode/initialization plus radius before repeating the same deterministic solve. Report numerical cycling, retain the best evaluated candidate, and do not call the physical mode globally infeasible. A 35-outer experimental cap preserves all selected converged traces, but loses one correct unselected sliding alternative that converges at 40. A 20 cap truncates 22/118 selected exhaustive successes. Five-rejection termination incorrectly interrupts 36 eventually converged exhaustive branches. No new guard or budget policy has been implemented. The 300/5000 options are hardcoded in mode_branch.m; changing similarly named config fields alone does not change them.

Remaining force errors have mixed causes. Wrong modes can fit noisy FBG better than correct modes; correct converged modes can still transfer load between tip and contact with tiny shape changes; missing Stage 1 candidates cannot be recovered by enumeration; many rotation steps return no feasible correct-mode candidate. At 5 mm insertion, six candidate misses account for 38.1% of tip-force squared error. At 5 mm rotation, 12 wrong-mode frames without a returned feasible correct-mode candidate account for 71.4%. These are descriptive categories, not a causal decomposition or global infeasibility certificate. Truth-oracle candidate scores are offline diagnostics only, never estimator inputs.

[Full report](../docs/reports/NOISY_EXHAUSTIVE_MODE_AUDIT.md). Outputs: results/published/noisy_mode_search_20261011/. Full new MAT checkpoints: results/runs/inclined_{insertion,rotation}_noisy{2,5}mm_exhaustive_20261011/. Log: results/runs/noisy_exhaustive_20261011/benchmark.log. Reproduce from the package with run_noisy_exhaustive('NEW_STAMP'), then python experiments/inclined_wall/analyze_noisy_mode_search.py NEW_STAMP. The source noise packets from noisy_linearized_20261011 are required; raw run artifacts are locally preserved and may need separate transfer because of repository ignore rules.

Next owner: mentor/collaborator to review cycle-guard approval and the distinction between mode search, missing geometry and force identifiability. Additional solver or physical assumptions remain subject to approval. No message was sent to the collaborator.
