# Base-rotation comparison handoff

Date: 2026-10-06. Author: Codex assisting the mentor. Direction: mentor to collaborator; prepared locally and not sent externally.

Branch: codex/two-stage-force-sensing. Base commit: b376013. This work is intentionally uncommitted and unpushed, following the user's updated AGENTS.md instruction.

## Approved scope and implementation

The mentor approved the inspected setup and requested implementation: rotate the precurved rod's base axially from 0 to 45 degrees at fixed 18 mm insertion, keep the plane and [1,0,-1] N tip force fixed in world coordinates, retain two noiseless bending FBG channels, and test centerline point-force friction without surface friction torque. Compare the current method, the same estimator with assumed mu=0, and Tongyu's unchanged Aloi implementation. Keep this analysis separate from the earlier sliding benchmark.

New code is confined to experiments/base_rotation, plus path registration and documentation. Our 15 estimator core files and collaborator source are unchanged. The independent forward wrapper calls the existing upstream 3D Cosserat map directly. It starts at rest with zero tangential preload, then uses true preceding material-point positions. Sticking and sliding reference roots are checked with the full disk projection, nonpenetration, gap, tangency, terminal moment and friction work. Nonphysical roots encountered during numerical development were rejected; their local diagnostic logs are retained.

## Evidence

See [full results and reproduction commands](../docs/reports/BASE_ROTATION_RESULTS.md) and results/published/base_rotation_20261006 for compact evidence. Large reference/estimator MAT files and logs stay under ignored results/runs/base_rotation_20261006. The previous sliding simulation is preserved.

With true mu=0.3, the contact sticks through 33.75 degrees and is sliding at 37.5 degrees. At 45 degrees, true friction is about 2.33 N and accumulated material twist is -6.95 degrees. Physical mu=0 still has about -2.61 degrees twist due to the world-fixed tip load; distinguish total twist from friction's additional effect.

Across all 13 frames including failures, the current estimator's contact/tip RMSE is about 0.825/0.610 N with mu=0.3, versus 3.819/2.748 N with assumed mu=0. Shape RMS is about 0.408/0.999 mm; Tongyu Aloi's is 1.666 mm. Aloi does not separately estimate contact and tip force. Correct-mu mechanics fails at 30 and 33.75 degrees and reports positive friction work among other warnings. Do not discard these points or treat other converged points as automatically passing fine-grid diagnostics.

Stage 1 omits unmeasured elastic twist and has 2.59 mm shape RMS error at 45 degrees; inferred arc shifts by 0.314 mm and normal by 0.336 degrees. An offline diagnostic restores reference twist solely to attribute reconstruction error; it is not an estimator result or an extra observation used in the comparisons.

The report separates forward physical sensitivity (regenerated true mu=0, 0.29, 0.3, 0.31) from inverse sensitivity (assumed mu=0.29/0.31 on fixed mu=0.3 data). Sticking loads can be insensitive to small coefficient changes until the bound activates; central differences near a stick/slip change are finite perturbations across modes. The current inverse also suffers from geometry bias and solver stopping effects.

## Validation and interpretation limits

Reference solutions are physically audited and the final reference map is rerun with tighter ODE tolerance. Deterministic estimator repetitions, source hashes and prior-result preservation are checked separately. Full per-frame outputs, solver iteration traces, physics checks and source provenance are retained. Reference equilibrium is not proof of global uniqueness, stability or real-world dynamic performance; actuation is quasistatic and no finite-radius friction torque is represented.

## Next owners and decisions

Mentor: review whether to improve treatment of unmeasured twist in geometric reconstruction before further friction-model tuning. Collaborator: reproduce this separate experiment and inspect the failed inverse steps and reference root audits. Codex: do not modify the observation model, add true twist to estimator input, add a within-step correction loop or commit/push without explicit authorization. No such change is implemented here.
