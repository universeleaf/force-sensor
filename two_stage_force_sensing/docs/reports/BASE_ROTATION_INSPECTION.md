# Base-rotation experiment: implementation inspection

Date: 2026-10-06. Status: the mentor approved implementation after this inspection. See [measured results](BASE_ROTATION_RESULTS.md). The inspection below motivated the separate experiment; estimator source and earlier sliding results remain unchanged.

## Current actuation

The original sliding_clean case in rod/contact_demo_scenes.m and rod/build_contact_demo_truth.m has 12 frames, fixed base orientation, insertion increasing from 18 to 22 mm, and world-X translation -0.5*(frame-1) mm. Each current/history pair is independently constructed by translating an equilibrium parallel to the fixed plane by 0.02 mm. Its friction coefficient is 0.3. These are prescribed sliding pairs, not a continuous simulated stick-to-slip trajectory. The independent truth solver for this case is planar.

## Rotation and friction already supported

Our mechanics stage accepts a general base rotation, three-dimensional contact/tip forces, and elastic material twist. A contact force is applied at the rod centerline. Its two tangential components lie in the environment contact plane; they are not constrained to the rod tangent. The exact disk projection opposes tangential displacement of the material centerline point between states. If the preceding mechanical posterior has the matching timestamp, the recurrent solve uses it as contact history; otherwise it reconstructs history from FBG.

There is no contact moment jump or finite-radius surface-friction torque. At zero radius, spinning a straight rod about its unchanged centerline produces no translational contact slip in this model. A precurved rod undergoing base rotation can move its centerline laterally, develop friction forces, and develop elastic torsion through spatial force loading. This tests that mechanism, not finite-radius circumferential friction torque.

The observation objective uses only the two measured bending components. Stage 1 reconstructs with intrinsic twist in the unobserved third component. The same limitation affects reconstructed-position input to Tongyu's Aloi implementation. Real elastic twist may therefore bias geometry even with noiseless FBG data. Keep this limitation visible in the requested current-method comparison; do not provide true twist as an observation or silently modify reconstruction.

## Approved setup

Rotate the base about its longitudinal axis from 0 to 45 degrees; hold its position at 18 mm insertion; hold the plane and tip force [1,0,-1] N fixed in world coordinates. Use centerline point-force Coulomb contact, mu=0.3, no independent contact torque, and the existing two noiseless bending channels and covariance settings. Generate a quasistatic sequence with consistent previous material-point positions, rather than rigidly rotating existing truth and the environment together.

This requires a separate three-dimensional forward contact wrapper. The existing, independent rod/solve_cosserat_multi_contact_map.m can be called directly for equilibrium, without copying or modifying the collaborator's mechanics. The wrapper must solve contact location, reaction and friction consistently, and audit terminal equilibrium, unilateral nonpenetration, tangency, cone and friction work before treating a forward solution as reference truth. Contact loss or an unsupported branch must be reported, not forced into single-contact validity.

Compare ours at assumed mu=0.3, ours at assumed mu=0, and the unchanged Tongyu Aloi estimator on the identical frictional packet. Preserve all failures and diagnostics. Track three-dimensional force errors, XYZ deflection, material twist u3 and accumulated twist, shape error, geometry mismatch, timing, optimizer steps, full-rod IVP passes and actual segmented ODE calls. Aloi supplies a distributed resultant rather than contact/tip separation.

Separate inverse sensitivity (changing assumed mu on fixed observations) from forward sensitivity (changing true friction under fixed actuation and regenerating observations). The former tests inference ambiguity; the latter tests physical force and deflection changes. Report material twist separately from geometric centerline torsion.

## Isolation and authorization

Use a new rotation experiment directory and a new run ID; retain previous sliding inputs and published results. No commit or push is authorized. Root AGENTS.md requires approval before important unplanned assumptions; the mentor approved this setup with 'implement this'. This inspection is retained separately from the measured rotation report.
