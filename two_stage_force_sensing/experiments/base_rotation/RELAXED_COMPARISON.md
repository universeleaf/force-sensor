# Reproducing the direct-relaxation rotation comparison

Run from the `two_stage_force_sensing` package in MATLAB R2025b with Optimization Toolbox and the existing `.local.json` baseline mapping. Do not run inside the collaborator's checkout. Old reports and the default estimator remain unchanged.

1. Extract the official CasADi 3.8.0 Windows64 MATLAB2018b archive into `results/runs/_dependencies/casadi` (so that `+casadi` is directly beneath that directory). See `docs/formulation/RELAXED_MECHANICS_SOLVERS.md` for the archive URL, checksum, bundled IPOPT version and exact mathematical/numerical contract. The archive's `include/licenses` directory is preserved. No global MATLAB path change is needed.
2. Prepare the coarse Bishop data, if not already present, using the existing `build_rotation_truth` and `prepare_bishop_rotation` experiment. The published run reused `results/runs/base_rotation_bishop_20261007/rotation/truth_mu_300.mat` without changing it.
3. Generate the independent fine-step reference and convert to native Bishop observations:

```matlab
setenv('TSFS_RUN_ID','base_rotation_relaxed_1deg_truth_20261007');
setup_tsfs;
build_rotation_truth(0.3,0:1:45);
setenv('TSFS_RUN_ID','base_rotation_relaxed_1deg_20261007');
[root,paths]=setup_tsfs;
prepare_relaxed_rotation(fullfile(root,'results/runs/base_rotation_relaxed_1deg_truth_20261007/rotation/truth_mu_300.mat'));
```

4. Validate the constraint derivatives and apex behavior with `test_relaxed`. The test uses the unchanged existing coarse Bishop reference.
5. Run both solvers serially on each increment. Use a fresh run ID for further experiments; the runner writes a separate file for each solver but rerunning the same solver/run ID overwrites that experimental result.

```matlab
setenv('TSFS_RUN_ID','base_rotation_relaxed_375deg_20261007');
[root,paths]=setup_tsfs;
mkdir(fullfile(paths.results,'rotation'));
copyfile(fullfile(root,'results/runs/base_rotation_bishop_20261007/rotation/truth_mu_300.mat'),fullfile(paths.results,'rotation'));
run_relaxed_rotation('ipopt');
run_relaxed_rotation('fmincon');
export_relaxed_rotation;
setenv('TSFS_RUN_ID','base_rotation_relaxed_1deg_20261007');
run_relaxed_rotation('ipopt');
run_relaxed_rotation('fmincon');
export_relaxed_rotation;
```

6. From the package directory, run `python experiments/base_rotation/summarize_relaxed_rotation.py` with NumPy, Matplotlib and h5py available. It publishes full-sequence summaries and figures. All original frame rows, including unsuccessful native exits and physical diagnostic flags, remain in the individual published run directories. Large MAT snapshots remain in ignored `results/runs`.

The generic mechanics code is `matlab/+tsfs/relaxed_solve.m`, `relaxed_constraints.m`, and `RelaxedCallback.m`. The experiment is a single-contact demo; the generic constraint assembler loops over arbitrary contacts and uses 3D IVP/contact tangent bases. It does not impose a plane-of-bending assumption. No finite-radius friction torque, tip contact, or edge/corner geometry is added.
