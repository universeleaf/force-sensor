# Nonlinear load-transfer and close-shape audit

Date: 2026-10-07. Author: Codex assisting mentor. Direction: mentor to collaborator, local only. Branch codex/two-stage-force-sensing; base commit b376013. No commit or push.

## Scope

User asked why tip/contact loads couple despite distinct spatial effects, and how close the resulting shapes can be. Added a separate offline mechanics audit and report; no estimator settings, previous runs or collaborator code changed. [Full report](../docs/reports/LOAD_TRANSFER_SHAPE_COUPLING.md). Evidence: results/published/base_rotation_load_transfer_20261007.

## Mechanism and settings

Upstream moment is (pL-p) cross Ftip + (pc-p) cross Fc. Equal-and-opposite load transfer d changes it by (pL-pc) cross d, which vanishes at frozen shape when d follows the contact-tip chord. The distal curved segment retains a nonzero signal. Contact loading also moves/rotates the distal rod via upstream deformation; its effect on global shape is not spatially confined to the upstream region.

Rod length 200 mm, contact near 160-161 mm. There are 24 two-channel FBG stations, of which only five are distal (165,174,183,191,200 mm). At 33.75 degrees chord length is 36.309 mm and tail maximum distance from chord 6.113 mm. The weakest tip-force direction/chord cosine is 0.99477. Model is inextensible/unshearable with no axial-strain likelihood.

## Nonlinear evidence

36 candidate equilibria: planar first frame and rotation 33.75/45 degrees; prescribed 0.5 N tip/contact/exchange loads along Y or chord, plus +/-0.01, +/-0.1, +/-0.5 N Y-profile perturbations with contact reactions solved on the original contact branch. Nonlinear mechanics, not merely linearized predictions. All terminal moments pass 1e-5 N mm. Twelve of eighteen branch candidates pass the stated contact checks; failures retained.

At 33.75 degrees, baseline tip [1,0,-1] N and contact [-0.506894,-2.470703,-8.631073] N compare with alternate tip [0.318591,-0.5,-0.261320] N and contact [0.146563,-1.957406,-9.512360] N. Force differences are 1.12248/1.21126 N, but shape RMS/max difference is only 0.00241083/0.00369716 mm. FBG RMS difference is 6.12571e-6 /mm. It passes current tolerances: terminal moment 2.05e-13 N mm, cone excess zero, contact point/gap below 1.5e-13 mm, refined near-contact penetration 4.40e-7 mm, tangency 1.69e-4. This is tolerance-based feasibility, not proof of exact smooth-surface contact equivalence.

The opposite +0.5 N Y-profile has a similarly close shape but violates the cone by 0.877 N. Large sliding-branch perturbations at 45 degrees fail penetration/tangency. The report distinguishes close shapes from valid contact candidates.

At sigma=1e-4 /mm, the successful example has full 48-channel whitened distance 0.4244 and sensor cost difference 0.09006. At sigma=1e-6 these become 42.44 and 900.58. Covariance changes actual information weighting; a scalar numerical objective multiplier does not. No noise was added and sensor differences are nonzero; exact nonidentifiability is not claimed.

Adding 0.5 N at tip versus contact along the chord at 33.75 degrees produces two shapes differing by 0.04592 mm RMS and FBG 5.3865e-6 /mm RMS. Either alone changes baseline shape by about 2.8 mm. Equal/opposite chord transfer changes baseline by only 0.001588 mm RMS. These prescribed-load comparisons do not impose contact validity and are explicitly labeled as mechanics-only.

## Checks and reproduction

Run setup_tsfs then audit_load_transfer with TSFS_RUN_ID=base_rotation_load_transfer_20261007; then experiments/base_rotation/summarize_load_transfer.py. Offline lsqnonlin uses analytic IVP derivatives to equilibrate base moment and (for branch-preserving trials) contact force; tip vector is prescribed. No sensor objective is optimized and no claim of a global best profile is made.

Gap checks use a 0.1 mm mesh plus continuous dense-IVP minimization within +/-0.5 mm of contact. Refinement caught sub-mesh penetration, including failures previously missed by the mesh. Thresholds: gap/penetration 1e-6 mm, terminal moment 1e-5 N mm, cone 1e-6 N, branch residual 1e-6 in corresponding units, and existing tangency warning 1e-3. All residuals, physical flags and solver exits are saved. Figures visually inspected.

No geometry/contact location uncertainty or history uncertainty included. Contact arclength remains fixed; strict smooth-surface tangency is not imposed as a new equality. Future model/sensor/solver changes require mentor direction. No external report sent, commit or push.
