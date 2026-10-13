# Decisions and current status

Updated 2026-10-06. This is an index, not permission to implement every discussed idea.

| Item | Status |
|---|---|
| Geometry MAP, then mechanics MAP with exact projection contact laws, then diagnosis | Implemented; known multi-contact failures retained |
| One geometry and one mechanics stage per physical step | Agreed; no within-step correction loop |
| Diagnosis reports warnings without changing or rejecting a step | Implemented |
| General 3D and multiple contacts | Core scope; fixture-specific information limitations remain |
| Contact friction torque, tip contact, edge/corner geometry | Deferred by user; see DEFERRED_WORK.md |
| Sensor-centered constant-curvature integration | Reconstruction-only experiment; not adopted |
| FBG curvature angle/frame mapping | Derived convention; real-device calibration not yet confirmed |
| Shape-matching measurement objective | Discussed alternative; no objective change approved |
| Profile objective/constraints against fixed tip Fy | Deferred by user; not run |
| Baseline robustness to realistic sensor/environment noise | Follow-up investigation; clean results are not robustness evidence |
| Noiseless sliding: current method, mu=0 ablation, Tongyu Aloi | Completed; correct friction separates loads accurately; zero friction converges with biased forces and FBG warnings. See SINGLE_CONTACT_MU_COMPARISON.md |
| Noiseless 0-45 degree axial base rotation | Approved and implemented as separate experiment; reveals unobserved-twist geometry bias and retained solver failures. No core change or surface friction torque. See BASE_ROTATION_RESULTS.md |
| Folder migration and no copied baseline algorithms | Approved and implemented in this contribution |

The filter currently uses recurrent constrained MAP with stated priors, not a calibrated full joint Kalman covariance. Keep proposals, implemented behavior, and measured validation distinct.

2026-10-07: Approved Bishop simulation measurements implemented, with matching full-material-IVP observation map. Stage 1 rotation geometry is accurate; one force outlier remains dependent on initialization. See [report](../reports/BASE_ROTATION_BISHOP_RESULTS.md). Experimental sensor representation remains deferred. No commit/push.

2026-10-07 residual audit: [22.5/41.25 comparison](../reports/BISHOP_INITIALIZATION_RESIDUAL_AUDIT.md) identifies loose scaled stationarity, damping stagnation and an unfinished solve; no evidence that the bad outputs are distinct MAP minima. Production code unchanged; solver fixes remain proposed.

2026-10-07: Completed the authorized direct-1e-6 IPOPT/fmincon rotation comparison at 3.75 and 1 degrees. Both adapters are experimental; default semismooth SQP is unchanged. [Full evidence](../reports/RELAXED_ROTATION_COMPARISON.md) includes all failures and small-step exits. A shorter increment is not uniformly more robust. No continuation schedule or history rejection policy has been added.

2026-10-07: Completed authorized revised relaxation runs: no force priors, FBG sigma=1e-4 /mm in both stages, nonlinear constraint tolerance=1e-6, duplicate gap removed, row scaling unchanged. [Results](../reports/RELAXED_ROTATION_V2_RESULTS.md) retain all 118 frames. Coarse fmincon improves; both IPOPT and fine fmincon have larger aggregate errors. Exact-law warnings remain separately available. No further numerical changes or history rejection implemented.

2026-10-07: [Directional sensitivity audit](../reports/ROTATION_DIRECTION_SENSITIVITY.md) completed without estimator changes. Rotation carries an existing weak mixed force direction into world Y; no new exact force singularity. Correct sliding-branch conditioning improves at failure onset, supporting further numerical branch-following investigation.

2026-10-10: Authorized explicit integer-mode experiment completed with full nonlinear mechanics and previous-mode-first initialization. [Results](../reports/INCLINED_WALL_INTEGER_MODES_RESULTS.md): 41/41 success for both mu=0.3 and mu=0; correctly specified friction removes prior outliers, while exhaustive sticking attempts dominate runtime. Local continuous SQP is not globally certified. Conditional mode search, broader noise/3D/multiple-contact validation, and warning repair remain proposals. Default solver unchanged; no commit/push.

2026-10-10 follow-up: User approved conditional previous-mode-first search, explicitly choosing raw FBG RMS <= 1e-6 /mm. [Results](../reports/INCLINED_WALL_CONDITIONAL_MODES_RESULTS.md): 13.94x faster for correct-friction inclined-wall trajectory with identical estimates; mu=0 has 33 explicitly flagged fit failures. Exhaustive search remains the default; the new runner selects conditional mode. No commit/push.

2026-10-10 rotation follow-up: [Loaded-state bidirectional rotation](../reports/INCLINED_WALL_ROTATION_RESULTS.md) completed from saved insertion frame 21, with reference/estimated XYZ force videos. Correct-friction tip/contact RMSE 0.09755/0.34507 N; all 24 selected solves converge, but only 10 pass the strict FBG fit gate. Return genuinely sticks; all return fits are flagged. Conditional solver and branch implementation unchanged; no commit/push.

2026-10-11: [Full-cone rod-linearization comparison](../reports/LINEARIZED_ROD_COMPARISON.md) completed. Opt-in affine mechanics with nonlinear correction reduces IVPs but is not universally faster: mu=0.3 insertion 0.390 -> 0.284 s; rotation 1.536 -> 4.916 s due to rejected sliding branches. Accuracy is close; one mu=0 rotation return fails outer convergence. Full nonlinear mechanics remains default. No commit/push.

2026-10-11 noisy benchmark: calibrated 2 mm/5 mm geometric tip noise and normalized FBG gate 1.24 tested with unchanged linearized mechanics and full friction. Feasibility and residual acceptance do not ensure mode/force accuracy; no new default solver or noise policy adopted. See [report](../reports/NOISY_LINEARIZED_ROD_RESULTS.md).

2026-10-11 exhaustive noisy audit: all four recurrent tests completed with unchanged core, noise and limits. Force RMS improves in all cases but remains large; twelve exhaustive selected states hit the outer limit. Repeated-subproblem detection is proposed, not implemented. A hard 20-outer cap loses useful solutions; retain 40/300 ceilings pending an approved guard trial. [Full evidence](../reports/NOISY_EXHAUSTIVE_MODE_AUDIT.md). No commit/push.

2026-10-13: [Fine-step BVP results](../reports/FINE_STEP_FIXED_LOAD_BVP_RESULTS.md) and [all eight videos/plots](../../results/published/fine_bvp_comparison_20261012_step5/media/README.md) published under explicit user authorization to commit/push the collaboration branch. No solver changes in the media-export task. See [publication handoff](../../handoff/2026-10-13_mentor-to-collaborator_media-publication.md).
