# Bishop rotation update: mentor-to-collaborator handoff

Date: 2026-10-07. Author: Codex assisting the mentor. Prepared locally; not sent externally. Branch: codex/two-stage-force-sensing; base commit b376013. No commit or push was requested or performed.

The mentor approved simulated geometric FBG measurements in Bishop notation: t'=w1*b1+w2*b2, with B(base)=R(base). This update implements that observation contract, its analytic mechanical-IVP Jacobian, and a separate rerun of the existing single-contact base rotation. Experimental IllumiSense/ShapeSensing angle interpretation remains unverified; these are ideal geometric observations.

Full material mechanics remains unchanged. A scalar psi state integrates u3, and predicted w=R2(psi)[u2,-u1] is compared to observations. Stage 1 integrates [-w2,w1,0] in a Bishop frame. Piecewise interpolation respects calibrated material transitions without using contact truth. Each calibrated segment requires two or more sensor stations. Legacy material observations still follow the original path. The native packet contains two Bishop channels, not a fictitious zero material-twist channel.

See ../docs/reports/BASE_ROTATION_BISHOP_RESULTS.md and ../results/published/base_rotation_bishop_20261007/. Reproduction entry points are prepare_bishop_rotation, run_bishop_rotation, analyze_bishop_rotation and diagnose_bishop_initialization. The existing physical reference is reused, with independently refined sensor conversion. Aloi is unmodified and receives positions reconstructed from the same sensor data. Previous rotation and sliding evidence is preserved.

Stage 1 is corrected: shape RMS across frames is 0.000550 mm; at 45 degrees it is 0.000642 mm instead of 2.588 mm. Maximum contact-arc error is 0.00150 mm; maximum normal error is 0.00233 degrees. Integration refinement leaves most of the small remaining error, implicating sparse interpolation.

Correct-mu final replay: tip/contact RMSE 0.1971/0.2205 N, shape RMS 0.00354 mm, mean 0.2715 seconds, 7.62 SQP iterations and 16.62 full-rod IVPs per frame. All 13 optimizer steps converge, but the 22.5-degree step is inaccurate (tip error 0.7105 N and FBG-mismatch warning). At 45 degrees tip/contact errors are 0.000305/0.000373 N. The assumed-zero-friction model converges but fits poorly: tip/contact RMSE 5.129/9.908 N. Aloi total-force RMSE is 1.485 N, shape RMS 0.1990 mm, mean 8.225 seconds, 10/13 positive optimizer exits and 13/13 final equilibrium checks.

Numerical initialization is still a bottleneck. The observation-only correction with the old linear seed fails at 41.25 degrees; that full replay is retained as cold.mat/cold_frames.csv. The final replay optionally starts from the preceding converged material solution (bishopWarmStart), without altering the objective, solver or tolerances. A same-history diagnostic shows the opposite seed is preferable at 22.5 degrees: objective 16.814 versus 404.06, tip error 0.00239 versus 0.71048 N. No automatic restart, best-result substitution or within-step geometry correction was added. Do not describe this as uniformly solved or as an irreducible sensor error.

Two small positive-friction-work warnings remain near sticking tolerance; values and projection/slip residuals are retained. Diagnostics remain reporting-only. True-plane and inferred-plane penetration are distinguished. See physics.csv and ours_frames.csv.

Validation: analytic helix, Bishop observation Jacobian finite differences, independent mechanical truth, world-rotation invariance, legacy core and spatial derivatives, identical baseline observations, deterministic repeats, and preserved-file hashes. Measured values and source hashes are published. Large MAT files/logs remain under results/runs/base_rotation_bishop_20261007.

Next owner: mentor decides whether to improve numerical initialization/globalization before adding noise. Collaborator can reproduce and review the explicit observation interface, statistical weighting and local force identifiability. Codex should retain the outlier and failed replay, avoid changing major assumptions without approval, and not commit/push without an explicit request.
