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
| Folder migration and no copied baseline algorithms | Approved and implemented in this contribution |

The filter currently uses recurrent constrained MAP with stated priors, not a calibrated full joint Kalman covariance. Keep proposals, implemented behavior, and measured validation distinct.
