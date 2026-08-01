function analysis = rerun_aloi_saved_result(resultFile, frameSelection)
%RERUN_ALOI_SAVED_RESULT Recompute only the Aloi baseline from saved data.
%
% analysis = rerun_aloi_saved_result(resultFile)
% analysis = rerun_aloi_saved_result(resultFile, 'final')
% analysis = rerun_aloi_saved_result(resultFile, frameIndices)

packageDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(packageDir);
if nargin < 1 || isempty(resultFile)
    resultFile = fullfile(rootDir, 'force_outputs', ...
        'rod_plane_senior_geometry_force_sensing', ...
        'rod_plane_force_sensing_results.mat');
end

saved = load(resultFile, 'results');
results = saved.results;
nt = numel(results.measurements.betaMm);
isTextSelection = nargin >= 2 && ...
    (ischar(frameSelection) || (isstring(frameSelection) && isscalar(frameSelection)));
if nargin < 2 || isempty(frameSelection) || ...
        (isTextSelection && strcmpi(string(frameSelection), 'all'))
    frameIds = 1:nt;
elseif isTextSelection && strcmpi(string(frameSelection), 'final')
    frameIds = nt;
else
    frameIds = unique(frameSelection(:)');
    assert(all(frameIds >= 1 & frameIds <= nt & frameIds == round(frameIds)), ...
        'frameSelection must contain valid integer frame indices.');
end

addpath(genpath(fullfile(rootDir, 'LCP-Continuum')));
cfg = addCurrentAloiDefaults(results.config);
tube = CreatTube(cfg.exposedLengthMm);
if cfg.scalePrecurvature
    baseBendRad = trapz(tube.s, sqrt(sum(tube.uhat(1:2, :) .^ 2, 1)));
    tube.uhat = tube.uhat * deg2rad(cfg.targetBendDeg) / max(baseBendRad, eps);
end

measurements = subsetMeasurements(results.measurements, frameIds);
forward = subsetForward(results.forward, frameIds);
aloi = estimate_aloi_gaussian_baseline(tube, measurements, cfg);
metrics = compute_aloi_comparison_metrics(forward, aloi, cfg);

analysis = struct;
analysis.sourceResultFile = resultFile;
analysis.frameIds = frameIds;
analysis.config = cfg.aloi;
analysis.forward = forward;
analysis.aloi = aloi;
analysis.metrics = metrics;

outputDir = fileparts(resultFile);
save(fullfile(outputDir, 'rod_plane_aloi_reanalysis.mat'), ...
    'analysis', '-v7.3');
writeReanalysisSummary(analysis, fullfile(outputDir, ...
    'rod_plane_aloi_reanalysis_summary.txt'));
writeReanalysisCsv(analysis, fullfile(outputDir, ...
    'rod_plane_aloi_reanalysis_trajectory.csv'));
plotReanalysis(analysis, fullfile(outputDir, ...
    'rod_plane_aloi_reanalysis.png'));

fprintf('\nAloi reanalysis complete.\n');
fprintf('  Active-frame force RMSE: %.4f N\n', metrics.resultantRmse);
fprintf('  Raw final full-vector mismatch: %.4f %%\n', ...
    metrics.finalRelativeErrorPct);
fprintf('  True local axial fraction: %.4f %%\n', ...
    metrics.finalTrueAxialFractionPct);
fprintf('  Within paper transverse-load assumption: %d\n', ...
    metrics.finalWithinPaperTransverseLoadAssumption);
fprintf('  Outputs: %s\n', outputDir);
end


function cfg = addCurrentAloiDefaults(cfg)
defaults = struct;
defaults.numOptimizationStarts = 3;
defaults.maxEquilibriumIterations = 20;
defaults.equilibriumToleranceMm = 1e-4;
defaults.equilibriumRelaxation = 0.5;
defaults.maxAxialFractionForComparison = 0.05;
names = fieldnames(defaults);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(cfg.aloi, name)
        cfg.aloi.(name) = defaults.(name);
    end
end
end


function measurements = subsetMeasurements(source, frameIds)
measurements = source;
measurements.betaMm = source.betaMm(frameIds);
measurements.baseTraj = source.baseTraj(:, :, frameIds);
measurements.pSparse = source.pSparse(:, :, frameIds);
end


function forward = subsetForward(source, frameIds)
forward = struct;
forward.s = source.s;
forward.totalForceResultant = source.totalForceResultant(:, frameIds);
forward.contactForceResultant = source.contactForceResultant(:, frameIds);
forward.contactArcLength = source.contactArcLength(frameIds);
forward.R = source.R(:, :, :, frameIds);
forward.actuationMm = source.actuationMm(frameIds);
end


function writeReanalysisSummary(analysis, path)
fid = fopen(path, 'w');
cleanup = onCleanup(@() fclose(fid));
m = analysis.metrics;
a = analysis.aloi;
fprintf(fid, 'Aloi Gaussian baseline reanalysis\n');
fprintf(fid, '=================================\n\n');
fprintf(fid, 'Source: %s\n', analysis.sourceResultFile);
fprintf(fid, 'Source frame IDs: %s\n\n', mat2str(analysis.frameIds));
fprintf(fid, 'Final true force [Fx Fy Fz] N: [%.9g %.9g %.9g]\n', ...
    m.finalTrueForce);
fprintf(fid, 'Final estimated force [Fx Fy Fz] N: [%.9g %.9g %.9g]\n', ...
    m.finalEstimatedForce);
fprintf(fid, 'Final true material-frame force [Fx Fy Fz] N: [%.9g %.9g %.9g]\n', ...
    m.finalTrueLocalContactForce);
fprintf(fid, 'Active-frame force RMSE: %.9g N\n', m.resultantRmse);
fprintf(fid, 'Raw final full-vector mismatch: %.9g %%\n', m.finalRelativeErrorPct);
fprintf(fid, 'Final magnitude mismatch: %.9g %%\n', m.finalMagnitudeErrorPct);
fprintf(fid, 'Final direction mismatch: %.9g deg\n', m.finalDirectionErrorDeg);
fprintf(fid, 'Final true local axial fraction: %.9g %%\n', ...
    m.finalTrueAxialFractionPct);
fprintf(fid, 'Within Aloi local-transverse assumption: %d\n', ...
    m.finalWithinPaperTransverseLoadAssumption);
fprintf(fid, 'Final center / sigma: %.9g / %.9g mm\n', ...
    a.centerMm(end), a.sigmaMm(end));
fprintf(fid, 'Final sparse-position RMSE: %.9g mm\n', a.shapeRmseMm(end));
fprintf(fid, 'Final nonlinear-equilibrium residual: %.9g mm\n', ...
    a.equilibriumResidualMm(end));
fprintf(fid, 'Optimizer exit flag / iterations / starts: %d / %d / %d\n\n', ...
    a.solverExitFlag(end), a.solverIterations(end), a.optimizationStarts(end));
fprintf(fid, ['Interpretation: Aloi et al. Eq. (9) sets local axial load to zero. ', ...
    'When the assumption flag is false, the raw vector mismatch is an ', ...
    'out-of-domain scenario diagnostic, not an in-domain paper accuracy result.\n']);
end


function writeReanalysisCsv(analysis, path)
a = analysis.aloi;
m = analysis.metrics;
f = analysis.forward;
T = table;
T.source_frame = analysis.frameIds(:);
T.motion_mm = f.actuationMm(:);
T.true_Fx_N = f.totalForceResultant(1, :)';
T.true_Fy_N = f.totalForceResultant(2, :)';
T.true_Fz_N = f.totalForceResultant(3, :)';
T.aloi_Fx_N = a.totalForceResultant(1, :)';
T.aloi_Fy_N = a.totalForceResultant(2, :)';
T.aloi_Fz_N = a.totalForceResultant(3, :)';
T.force_error_N = vecnorm(a.totalForceResultant - f.totalForceResultant, 2, 1)';
T.true_local_axial_fraction_pct = m.trueAxialFractionPct(:);
T.within_transverse_load_assumption = m.withinPaperTransverseLoadAssumption(:);
T.center_mm = a.centerMm(:);
T.sigma_mm = a.sigmaMm(:);
T.shape_rmse_mm = a.shapeRmseMm(:);
T.equilibrium_residual_mm = a.equilibriumResidualMm(:);
writetable(T, path);
end


function plotReanalysis(analysis, path)
f = analysis.forward;
a = analysis.aloi;
m = analysis.metrics;
motion = f.actuationMm;
error = vecnorm(a.totalForceResultant - f.totalForceResultant, 2, 1);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 720]);
subplot(2, 2, 1);
plot(motion, f.totalForceResultant(1, :), 'k-o', 'LineWidth', 1.6, ...
    'MarkerSize', 4); hold on;
plot(motion, a.totalForceResultant(1, :), 'r--s', 'LineWidth', 1.5, ...
    'MarkerSize', 4);
plot(motion, f.totalForceResultant(3, :), 'b-o', 'LineWidth', 1.6, ...
    'MarkerSize', 4);
plot(motion, a.totalForceResultant(3, :), 'm--s', 'LineWidth', 1.5, ...
    'MarkerSize', 4);
grid on; box on; xlabel('Motion [mm]'); ylabel('Force [N]');
title('True and Aloi force components');
legend({'True F_x', 'Aloi F_x', 'True F_z', 'Aloi F_z'}, 'Location', 'best');

subplot(2, 2, 2);
plot(motion, error, 'r-o', 'LineWidth', 1.6, 'MarkerSize', 4); hold on;
yyaxis right;
plot(motion, m.trueAxialFractionPct, 'k:o', 'LineWidth', 1.6, ...
    'MarkerSize', 4);
grid on; box on; xlabel('Motion [mm]');
yyaxis left; ylabel('Force mismatch [N]');
yyaxis right; ylabel('True local axial fraction [%]');
title('Mismatch and paper-assumption check');

subplot(2, 2, 3);
plot(motion, a.shapeRmseMm, '-o', 'Color', [0.1 0.55 0.35], ...
    'LineWidth', 1.6, 'MarkerSize', 4);
grid on; box on; xlabel('Motion [mm]'); ylabel('RMSE [mm]');
title('Sparse-position fit');

subplot(2, 2, 4);
plot(motion, f.contactArcLength, 'k-o', 'LineWidth', 1.6, ...
    'MarkerSize', 4); hold on;
plot(motion, a.centerMm, 'r--s', 'LineWidth', 1.6, ...
    'MarkerSize', 4);
grid on; box on; xlabel('Motion [mm]'); ylabel('Arc length [mm]');
title('True contact and Gaussian center');
legend({'True contact', 'Aloi center'}, 'Location', 'best');

exportgraphics(fig, path, 'Resolution', 200);
close(fig);
end
