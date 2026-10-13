# Media and branch publication

2026-10-13. Author/direction: Codex, mentor-side collaborator to collaborator. Branch: codex/two-stage-force-sensing; parent commit b376013. The user explicitly authorized committing and pushing this branch in this turn. The resulting commit is identifiable in branch history as Publish Bishop sensing experiments and fine-step BVP media.

Generated eight videos covering insertion/rotation and noise targets 0/0.5/1/2 mm, plus 24 static plot sheets (force components; errors/timing/physical consistency; Stage 1 geometry), representative frames, a local HTML gallery, and compact rendering sources. All frames are included, including restored rotation seeds and failed corrections. Insertion videos have402 encoded frames /40.2 s; rotation videos242 /24.2 s; 10 fps with each saved state shown twice. Playback is illustrative, not physical time.

Validation: reference/estimated shape and XYZ force errors match saved numerical results; all eight MP4s fully decoded with exact frame counts and durations. Checksums are saved in media/validation.json. Representative normal and failure snapshots and diagnostic plots were visually inspected. All experiment Python sources parse. No mechanics rerun or solver change was needed for this request. Existing eight-trajectory numerical audits remain the mechanics validation evidence.

Added publish_fine_bvp_media.py and validate_fine_bvp_media.py under experiments/inclined_wall. Reproduce by running both in that order (default stamp20261012_step5). Published shapes.npz inputs allow rendering without large local MAT checkpoints. The exact noise payload/protocol and reference audits are packaged under results/published/fine_bvp_comparison_20261012_step5/reproduction. Raw MATLAB checkpoints and local logs remain excluded by existing ignore rules.

Publication includes the accumulated mentor-side Bishop observation, solver comparison, explicit-mode, linearized rod, fixed-load BVP, noise/sensitivity implementation and reports since b376013, together with the root AGENTS.md explicit commit/push rule. All staged changes are confined to two_stage_force_sensing/ and root AGENTS.md. The collaborator checkout and original algorithm folders remain untouched. Historical reports stating no commit/push describe their original milestones; this entry records the new authorization.

Known scientific limitations remain unchanged: BVP convergence does not enforce contact geometry/friction; all three2 mm BVP failures remain visible; local capped mode solves are not a global certificate. No noise filtering, extra outer loop, fallback change or hidden frame removal.

Output index: results/published/fine_bvp_comparison_20261012_step5/media/README.md. Local interactive gallery: media/index.html. Main numerical report: docs/reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md. No merge to main or pull request was requested.
