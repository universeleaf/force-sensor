# Baseline audit

`run_bottleneck_audit` calls unchanged upstream estimators through `baseline/run_baseline.m`. `prepare_inputs` uses recorded fixtures by default; pass `regenerate` to call the original scene builders.

Use separate ordinary and `struct('profile',true)` runs. MATLAB profiler function-call counts are not automatically whole-rod IVP counts. No new exact internal counter is claimed unless the original solver exposes it.

`analyze_bottleneck_audit(folder)` recomputes independent physics/sensitivity analysis from complete sliding and planar results. It requires at least the original twelve planar frames (uses frame 6). It is additional analysis, not part of the timed solve.

`summarize_bottleneck_audit.py` is only for historical instrumented reports; it requires their counter fields. Its default reads the local legacy archive and writes a separate summary. The compact historical evidence is under `results/published/baseline_20261005/`.

The old `prepare_bottleneck_audit.py`, shadow solvers, and frozen-geometry solver copies are intentionally not migrated. Historical frozen-geometry results remain evidence, but replaying those altered solvers is not supported by this no-copy layout. No algorithm is silently substituted.
