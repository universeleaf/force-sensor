# 2026-10-10 mentor to collaborator: loaded-state rotation and force videos

Author: Codex for mentor. Intended recipient: collaborator; local handoff only, not sent. Branch codex/two-stage-force-sensing, base b376013. Uncommitted; no push.

User requested XYZ tip/contact reference-versus-estimate plots on the insertion video, then bidirectional 45-degree rotation from the maximum-insertion frame. The new insertion video is separate from the old one. Rotation uses the saved frame 21 at 40 mm perpendicular wall distance, keeps each estimator's own previous solution, rotates local base Z 0 -> 45 -> 0 in 3.75-degree increments, and keeps the world tip load [1,0,0] N. Both mu=0.3 and mu=0 receive the same noiseless Bishop observations.

New experiment wrappers save/validate the checkpoint, generate independent 3D forward truth using the collaborator map read-only, replay the unchanged conditional solver, audit all costs/results, and render videos. The common run_inclined_modes runner accepts an optional replay source and initial states; its existing insertion defaults remain. No collaborator algorithms were copied or modified. mode_solve.m and mode_branch.m hashes match the previous insertion results.

24 new solves per method: mu=0.3 tip/contact RMSE 0.09755/0.34507 N, mean time 4.797 s, 57.67 iterations and 121.04 IVPs per step. Mu=0: 1.55852/5.50778 N, 2.044 s, 38.33 iterations and 46.17 IVPs. All selected solves converge and satisfy branch feasibility, but only 10/24 and 0/24 respectively pass the approved raw FBG RMS <= 1e-6 /mm gate. Every return step is flagged for mu=0.3. All failed fits and physical warnings are retained.

Independent truth slides on all 12 forward increments and sticks on all 12 reverse increments. Estimator mu=0.3 chooses these same modes. All 14 attempted sticking solves converge and are feasible, but fail the fit gate. Worst total time 36.811 s at return 7.5 degrees is dominated by a rejected alternative sliding solve (34.659 s); selected sticking takes 0.880 s. No new gate, history rejection, geometry correction or solver tuning was introduced.

Stage 1 max arc error is 0.04726 mm and max normal error 0.001340 degrees. Small nonzero geometry/history errors remain possible contributors to strict-gate failures. This run does not establish their causal contributions. All 25 physical reference states pass audits; final zero-angle contact force differs from the initial force because the return sticks.

[Full report, configuration, output paths and reproduction](../docs/reports/INCLINED_WALL_ROTATION_RESULTS.md). Published outputs: results/published/inclined_wall_rotation_20261010/. Raw loaded checkpoint and trajectories: results/runs/inclined_wall_rotation_20261010/. Updated insertion video: results/published/inclined_wall_modes_conditional_1e6_20261010/video_forces/.

Next actions for mentor/collaborator review: inspect flagged return fits and distinguish geometry/history mismatch from local minima with controlled diagnostic reruns before changing acceptance or physics. Those additional experiments have not been performed here.
