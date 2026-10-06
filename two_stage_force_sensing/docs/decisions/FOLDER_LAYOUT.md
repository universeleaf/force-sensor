# Folder ownership and migration map

The branch adds only root AGENTS.md and two_stage_force_sensing/. Existing repository algorithms and dependency checkers are unchanged. This checkout is separate from the collaborator's checkout. The original standalone workspace is retained as a backup; this branch is now the canonical implementation.

- matlab/+tsfs: the unchanged estimator core.
- baseline: direct callers, recorded/regenerated input preparation, and source provenance.
- experiments/benchmarks: run_smoke, run_benchmarks, run_ablations, make_validation_fixtures.
- experiments/s_channel: geometry/force diagnosis and the additional-channel control.
- experiments/plane_sensitivity: equilibrium sensitivity and condition analysis.
- experiments/fbg_reconstruction: the interpolation/PCC comparison.
- experiments/bottleneck_audit: our baseline harness and independent analysis, without copied/instrumented solvers.
- scripts: our Python comparison report generator.
- docs/formulation: governing model, historical plan, FBG frames, and objective alternatives.
- docs/reports: dated scientific evidence; docs/decisions: current status and deferred scope.
- datasets: recorded input and truth fixtures, not baseline solver outputs.
- results/published: selected historical evidence and migration verification.
- results/runs: ignored generated files and retained local archives.
- handoff: reports between mentor and collaborator.

Runtime dependencies use the configured existing baseline root. Configuration is local or environment-based; no copied algorithms, junctions, or absolute machine paths are needed in tracked setup code. Root AGENTS.md applies to the entire checkout. Important unplanned scientific decisions still require the user's approval.
