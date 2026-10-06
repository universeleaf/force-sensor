# Results and provenance

`published/baseline_20261005/` contains compact historical baseline evidence, including instrumented runs and frozen-geometry ablations. These are not new unchanged-source runs. `published/two_stage_20261005/` contains the original two-stage benchmark and diagnosis evidence. Original timestamps, absolute paths, hashes, failures, and source-revision records are retained as historical metadata.

`runs/<TSFS_RUN_ID>/` contains generated output and is ignored by Git. Migration retained original full MATLAB outputs locally under `runs/legacy_20261005/` and baseline artifacts under `runs/baseline_legacy_20261005/`, without copying their MATLAB/Python solver bodies. Those large local archives do not ship with the branch. Recorded input fixtures do ship; rerun the entry points to generate new traces.

Generated comparison reports are written to runs/comparison_report by default, not over the curated historical report. Old report paths describe the old workspace; use this index and README for current paths.
