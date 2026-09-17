function force_rucker()
% Rucker 2011 EKF method for tip force estimation
clearvars;
clc;
close all;

cfg = defaultRuckerConfig();
results = runRuckerBenchmark(cfg);
printRuckerSummary(results, cfg);
plotRuckerReport(results, cfg);
end


function cfg = defaultRuckerConfig()
cfg.dim = 2;
cfg.seed = 7;
cfg.outputDir = fullfile(pwd, 'force_outputs');

cfg.L = 0.30;
cfg.EI = 0.03;
cfg.nGrid = 101;
cfg.baseCurvature = 3.2;

cfg.tipCase.forceTrue = 0.18;
cfg.tipCase.nSteps = 40;
cfg.tipCase.forceRampStart = 12;
cfg.tipCase.forceRampStop = 28;
cfg.tipCase.poseNoiseStd = [2.0e-4; 2.0e-4; 5.0e-3];
cfg.tipCase.actuationNoiseStd = 0.03;
cfg.tipCase.forceProcessStd = 0.006;
cfg.tipCase.forceInitStd = 0.10;
cfg.tipCase.actuationScaleTrue = 1.0;
end


function results = runRuckerBenchmark(cfg)
rng(cfg.seed, 'twister');
ensureOutputDir(cfg.outputDir);

model = beamInfluenceMatrices(cfg);
tipCase = simulateTipCase(cfg, model);

results.config = cfg;
results.model = model;
results.tipCase = tipCase;
results.rucker = estimateRuckerEKF(cfg, model, tipCase);
end


function model = beamInfluenceMatrices(cfg)
s = linspace(0, cfg.L, cfg.nGrid).';
ds = s(2) - s(1);

Phi = zeros(cfg.nGrid, cfg.nGrid);
for i = 1:cfg.nGrid
    si = s(i);
    for j = 1:cfg.nGrid
        xi = s(j);
        if si <= xi
            kernel = si^2 * (3 * xi - si) / (6 * cfg.EI);
        else
            kernel = xi^2 * (3 * si - xi) / (6 * cfg.EI);
        end
        Phi(i, j) = kernel * ds;
    end
end

actShape = 0.5 * cfg.baseCurvature * s.^2;
actTheta = cfg.baseCurvature * s;

model.s = s;
model.ds = ds;
model.Phi = Phi;
model.actShape = actShape;
model.actTheta = actTheta;
end


function tipCase = simulateTipCase(cfg, model)
n = cfg.tipCase.nSteps;
forceTrue = zeros(n, 1);
rampVals = linspace(0, cfg.tipCase.forceTrue, cfg.tipCase.forceRampStop - cfg.tipCase.forceRampStart + 1);
forceTrue(cfg.tipCase.forceRampStart:cfg.tipCase.forceRampStop) = rampVals;
forceTrue((cfg.tipCase.forceRampStop + 1):end) = cfg.tipCase.forceTrue;

actScaleTrue = cfg.tipCase.actuationScaleTrue * ones(n, 1);
poseTrue = zeros(3, n);
poseMeas = zeros(3, n);
actMeas = zeros(n, 1);
shapeTrue = zeros(cfg.nGrid, n);
thetaTrue = zeros(cfg.nGrid, n);

for k = 1:n
    [yTipLoad, thetaTipLoad] = tipForceInfluence(model, cfg, forceTrue(k));
    y = actScaleTrue(k) * model.actShape + yTipLoad;
    theta = actScaleTrue(k) * model.actTheta + thetaTipLoad;
    shapeTrue(:, k) = y;
    thetaTrue(:, k) = theta;
    poseTrue(:, k) = [model.s(end); y(end); theta(end)];
    poseMeas(:, k) = poseTrue(:, k) + [0; cfg.tipCase.poseNoiseStd(2) * randn; cfg.tipCase.poseNoiseStd(3) * randn];
    actMeas(k) = actScaleTrue(k) + cfg.tipCase.actuationNoiseStd * randn;
end

tipCase.forceTrue = forceTrue;
tipCase.actuationScaleTrue = actScaleTrue;
tipCase.poseTrue = poseTrue;
tipCase.poseMeas = poseMeas;
tipCase.actuationMeas = actMeas;
tipCase.shapeTrue = shapeTrue;
tipCase.thetaTrue = thetaTrue;
end


function rucker = estimateRuckerEKF(cfg, model, tipCase)
n = cfg.tipCase.nSteps;
x = [0; model.actShape(end); model.actTheta(end); 1.0; 0.0];
P = diag([1e-8, 1e-5, 1e-4, 0.04, cfg.tipCase.forceInitStd^2]);
Q = diag([1e-10, 1e-8, 1e-6, 2e-4, cfg.tipCase.forceProcessStd^2]);
R = diag([1e-10, cfg.tipCase.poseNoiseStd(2)^2, cfg.tipCase.poseNoiseStd(3)^2, cfg.tipCase.actuationNoiseStd^2]);

xHist = zeros(numel(x), n);
PHist = zeros(numel(x), numel(x), n);

for k = 1:n
    xPred = x;
    PPred = P + Q;

    zPred = measurementFromRuckerState(xPred, cfg, model);
    H = finiteDifferenceJacobian(@(xx) measurementFromRuckerState(xx, cfg, model), xPred, [1e-7; 1e-6; 1e-6; 1e-4; 1e-4]);
    z = [tipCase.poseMeas(:, k); tipCase.actuationMeas(k)];
    innov = z - zPred;
    S = H * PPred * H.' + R;
    K = (PPred * H.') / S;

    x = xPred + K * innov;
    P = (eye(size(P)) - K * H) * PPred;
    x(1) = model.s(end);
    x(4) = max(0.7, min(1.3, x(4)));

    xHist(:, k) = x;
    PHist(:, :, k) = P;
end

forceEst = xHist(5, :).';
sigmaForce = squeeze(sqrt(PHist(5, 5, :)));
relErr = abs(forceEst(end) - tipCase.forceTrue(end)) / max(abs(tipCase.forceTrue(end)), 1e-8);

rucker.xHist = xHist;
rucker.PHist = PHist;
rucker.forceEst = forceEst;
rucker.forceSigma = sigmaForce;
rucker.forceTrue = tipCase.forceTrue;
rucker.finalAbsErr = abs(forceEst(end) - tipCase.forceTrue(end));
rucker.finalRelErr = relErr;
rucker.shapeEst = reconstructRuckerShapes(xHist, cfg, model);
end


function z = measurementFromRuckerState(x, cfg, model)
scale = x(4);
force = x(5);
[yLoad, thetaLoad] = tipForceInfluence(model, cfg, force);
yTip = scale * model.actShape(end) + yLoad(end);
thetaTip = scale * model.actTheta(end) + thetaLoad(end);
z = [model.s(end); yTip; thetaTip; scale];
end


function [yLoad, thetaLoad] = tipForceInfluence(model, cfg, force)
s = model.s;
L = cfg.L;
yLoad = force .* s.^2 .* (3 * L - s) ./ (6 * cfg.EI);
thetaLoad = force .* s .* (2 * L - s) ./ (2 * cfg.EI);
end


function shapeEst = reconstructRuckerShapes(xHist, cfg, model)
nSteps = size(xHist, 2);
shapeEst = zeros(cfg.nGrid, nSteps);
for k = 1:nSteps
    scale = xHist(4, k);
    force = xHist(5, k);
    [yLoad, ~] = tipForceInfluence(model, cfg, force);
    shapeEst(:, k) = scale * model.actShape + yLoad;
end
end


function J = finiteDifferenceJacobian(fun, x, step)
f0 = fun(x);
m = numel(f0);
n = numel(x);
J = zeros(m, n);
for i = 1:n
    xp = x;
    xp(i) = xp(i) + step(i);
    fp = fun(xp);
    J(:, i) = (fp(:) - f0(:)) / step(i);
end
end


function printRuckerSummary(results, cfg)
rucker = results.rucker;
fprintf('\n=== Rucker 2011 EKF Method ===\n');
fprintf('Output directory: %s\n\n', cfg.outputDir);
fprintf('Final force true/est [N]: %.4f / %.4f\n', rucker.forceTrue(end), rucker.forceEst(end));
fprintf('Absolute error: %.4f N\n', rucker.finalAbsErr);
fprintf('Relative error: %.2f %%\n', 100 * rucker.finalRelErr);
end


function plotRuckerReport(results, cfg)
ensureOutputDir(cfg.outputDir);
model = results.model;
tipCase = results.tipCase;
rucker = results.rucker;

fig = figure('Name', 'Rucker EKF', 'Color', 'w', 'Position', [100 100 1200 800]);

subplot(2, 2, 1);
stairs(1:numel(rucker.forceTrue), rucker.forceTrue, 'k-', 'LineWidth', 2); hold on;
plot(1:numel(rucker.forceEst), rucker.forceEst, 'r-', 'LineWidth', 2);
plot(1:numel(rucker.forceEst), rucker.forceEst + 2 * rucker.forceSigma, 'r--', 'LineWidth', 1);
plot(1:numel(rucker.forceEst), rucker.forceEst - 2 * rucker.forceSigma, 'r--', 'LineWidth', 1);
grid on; box on;
xlabel('Time step'); ylabel('Tip force [N]');
title(sprintf('Force Estimation (Rel Err: %.1f%%)', 100 * rucker.finalRelErr));
legend({'True', 'Estimated', '+2\sigma', '-2\sigma'}, 'Location', 'best');

subplot(2, 2, 2);
plot(model.s, tipCase.shapeTrue(:, end) * 1e3, 'k-', 'LineWidth', 2); hold on;
plot(model.s, rucker.shapeEst(:, end) * 1e3, 'r-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Deflection y [mm]');
title('Shape Estimation');
legend({'True', 'Estimated'}, 'Location', 'best');

subplot(2, 2, 3);
plot(1:numel(rucker.forceEst), tipCase.poseMeas(2, :) * 1e3, 'b.-', 'LineWidth', 1.5); hold on;
plot(1:numel(rucker.forceEst), tipCase.poseTrue(2, :) * 1e3, 'k-', 'LineWidth', 1.5);
grid on; box on;
xlabel('Time step'); ylabel('Tip y [mm]');
title('Tip Position');
legend({'Measured', 'True'}, 'Location', 'best');

subplot(2, 2, 4);
errorVec = abs(rucker.forceEst - rucker.forceTrue);
plot(1:numel(errorVec), errorVec * 1e3, 'r-', 'LineWidth', 2);
grid on; box on;
xlabel('Time step'); ylabel('Absolute error [mN]');
title(sprintf('Force Error (Final: %.2f mN)', 1e3 * rucker.finalAbsErr));

sgtitle('Rucker 2011 Extended Kalman Filter Method');
saveFigure(fig, fullfile(cfg.outputDir, 'rucker_method.png'));
end


function ensureOutputDir(pathStr)
if ~exist(pathStr, 'dir')
    mkdir(pathStr);
end
end


function saveFigure(figHandle, filePath)
drawnow;
try
    print(figHandle, filePath, '-dpng', '-r200');
catch
    exportgraphics(figHandle, filePath, 'Resolution', 200);
end
end
