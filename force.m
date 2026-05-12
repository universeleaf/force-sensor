function force()
% Aloi 2022 Gaussian Load Estimation Method
% Scenario: 2D elastic rod with FBG sensors, two point loads (body + tip)
clearvars;
clc;
close all;

cfg = defaultAloiConfig();
results = runAloiSimulation(cfg);
printAloiSummary(results, cfg);
plotAloiResults(results, cfg);
plotAloiFig6Style(results, cfg);
end


function cfg = defaultAloiConfig()
cfg.dim = 2;
cfg.seed = 42;
cfg.outputDir = fullfile(pwd, 'force_outputs');

% Robot parameters
cfg.L = 0.30;                    % Length [m]
cfg.EI = 0.03;                   % Bending stiffness [N*m^2]
cfg.nGrid = 101;                 % Discretization points
cfg.nMeas = 21;                  % FBG sensor locations
cfg.baseCurvature = 0.0;         % No actuation curvature

% Two point loads (known magnitudes)
cfg.load1.position = 0.15;       % Body load at 15 cm
cfg.load1.magnitude = 0.08;      % 80 mN lateral force
cfg.load2.position = 0.30;       % Tip load at 30 cm
cfg.load2.magnitude = 0.12;      % 120 mN lateral force

% Measurement noise (FBG sensors)
cfg.shapeNoiseStd = 6.0e-4;      % Shape measurement noise [m]
cfg.curvatureNoiseStd = 0.05;    % Curvature noise [1/m]

% Aloi optimization parameters
cfg.aloi.nGauss = 2;             % Two Gaussian components for two loads
cfg.aloi.theta0 = [0.08; 0.15; 0.03; 0.12; 0.30; 0.03];  % [F1, mu1, sig1, F2, mu2, sig2]
cfg.aloi.lb = [0.02; 0.05; 0.01; 0.04; 0.20; 0.01];
cfg.aloi.ub = [0.15; 0.25; 0.08; 0.20; 0.30; 0.08];
cfg.aloi.maxIter = 100;
cfg.aloi.lambda0 = 1e-3;
cfg.aloi.fdStep = [1e-3; 2e-3; 1e-3; 1e-3; 2e-3; 1e-3];
cfg.aloi.priorWeight = diag([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);

% Multi-start seeds for robustness
cfg.aloi.forceSeeds1 = [0.06, 0.08, 0.10];
cfg.aloi.muSeeds1 = [0.12, 0.15, 0.18];
cfg.aloi.sigmaSeeds1 = [0.02, 0.03, 0.04];
cfg.aloi.forceSeeds2 = [0.10, 0.12, 0.14];
cfg.aloi.muSeeds2 = [0.28, 0.30];
cfg.aloi.sigmaSeeds2 = [0.02, 0.03];
end


function results = runAloiSimulation(cfg)
rng(cfg.seed, 'twister');
ensureOutputDir(cfg.outputDir);

model = buildBeamModel(cfg);
trueData = simulateTwoPointLoads(cfg, model);
estimation = estimateAloiLoad(cfg, model, trueData);

results.config = cfg;
results.model = model;
results.trueData = trueData;
results.estimation = estimation;
end


function model = buildBeamModel(cfg)
s = linspace(0, cfg.L, cfg.nGrid).';
ds = s(2) - s(1);

% FBG sensor locations (evenly distributed)
sMeas = linspace(0, cfg.L, cfg.nMeas).';
measIdx = round(linspace(1, cfg.nGrid, cfg.nMeas));
measIdx = unique(min(max(measIdx, 1), cfg.nGrid));
if numel(measIdx) ~= cfg.nMeas
    measIdx = round(interp1(1:numel(measIdx), measIdx, linspace(1, numel(measIdx), cfg.nMeas), 'nearest', 'extrap'));
end
sMeas = s(measIdx);

% Influence matrix (Green's function for cantilever beam)
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

% Derivative operators
D1 = zeros(cfg.nGrid, cfg.nGrid);
for i = 2:(cfg.nGrid - 1)
    D1(i, i-1) = -0.5 / ds;
    D1(i, i+1) = 0.5 / ds;
end
D1(1, 1:2) = [-1, 1] / ds;
D1(end, end-1:end) = [-1, 1] / ds;

D2 = zeros(cfg.nGrid, cfg.nGrid);
for i = 2:(cfg.nGrid - 1)
    D2(i, i-1:i+1) = [1, -2, 1] / ds^2;
end
D2(1, 1:3) = [1, -2, 1] / ds^2;
D2(end, end-2:end) = [1, -2, 1] / ds^2;

model.s = s;
model.ds = ds;
model.sMeas = sMeas;
model.measIdx = measIdx;
model.Phi = Phi;
model.D1 = D1;
model.D2 = D2;
model.actShape = zeros(cfg.nGrid, 1);  % No actuation
model.actKappa = zeros(cfg.nGrid, 1);
end


function trueData = simulateTwoPointLoads(cfg, model)
% Create two point loads as delta functions
qTrue = zeros(cfg.nGrid, 1);

% Load 1: body load
[~, idx1] = min(abs(model.s - cfg.load1.position));
qTrue(idx1) = cfg.load1.magnitude / model.ds;

% Load 2: tip load
[~, idx2] = min(abs(model.s - cfg.load2.position));
qTrue(idx2) = cfg.load2.magnitude / model.ds;

% Compute true deflection
yLoad = model.Phi * qTrue;
yTrue = model.actShape + yLoad;
thetaTrue = model.D1 * yLoad;
kappaTrue = model.actKappa + model.D2 * yLoad;

% Add FBG measurement noise
yMeas = yTrue(model.measIdx) + cfg.shapeNoiseStd * randn(cfg.nMeas, 1);
kappaMeas = kappaTrue(model.measIdx) + cfg.curvatureNoiseStd * randn(cfg.nMeas, 1);

trueData.qTrue = qTrue;
trueData.yTrue = yTrue;
trueData.thetaTrue = thetaTrue;
trueData.kappaTrue = kappaTrue;
trueData.yMeas = yMeas;
trueData.kappaMeas = kappaMeas;
trueData.load1Idx = idx1;
trueData.load2Idx = idx2;
trueData.rawMeasurementRmse = sqrt(mean((yMeas - yTrue(model.measIdx)).^2));
end


function estimation = estimateAloiLoad(cfg, model, trueData)
Wpos = eye(cfg.nMeas) / (cfg.shapeNoiseStd^2);
best.theta = cfg.aloi.theta0;
best.cost = inf;
best.iter = 0;

% Multi-start optimization
fprintf('Running Aloi optimization with multi-start...\n');
nTries = 0;
for f1 = cfg.aloi.forceSeeds1
    for mu1 = cfg.aloi.muSeeds1
        for sig1 = cfg.aloi.sigmaSeeds1
            for f2 = cfg.aloi.forceSeeds2
                for mu2 = cfg.aloi.muSeeds2
                    for sig2 = cfg.aloi.sigmaSeeds2
                        nTries = nTries + 1;
                        theta0 = [f1; mu1; sig1; f2; mu2; sig2];
                        [thetaTry, costTry, iterTry] = runAloiLocalFit(theta0, cfg, model, trueData, Wpos);
                        if costTry < best.cost
                            best.theta = thetaTry;
                            best.cost = costTry;
                            best.iter = iterTry;
                            fprintf('  Try %d: New best cost = %.6f\n', nTries, costTry);
                        end
                    end
                end
            end
        end
    end
end
fprintf('Optimization complete. Best cost: %.6f\n', best.cost);

theta = best.theta;
qEst = twoGaussianLoad(model.s, theta);
yEst = model.actShape + model.Phi * qEst;

shapeRmse = sqrt(mean((yEst(model.measIdx) - trueData.yTrue(model.measIdx)).^2));

% Extract estimated load locations and magnitudes
estimation.paramsEst = theta;
estimation.qEst = qEst;
estimation.yEst = yEst;
estimation.shapeRmse = shapeRmse;
estimation.cost = best.cost;
estimation.iterations = best.iter;
estimation.load1Est = [theta(2), theta(1)];  % [position, magnitude]
estimation.load2Est = [theta(5), theta(4)];
end


function [theta, bestCost, iter] = runAloiLocalFit(theta0, cfg, model, trueData, Wpos)
theta = projectBounds(theta0, cfg.aloi.lb, cfg.aloi.ub);
lambda = cfg.aloi.lambda0;
bestCost = inf;

for iter = 1:cfg.aloi.maxIter
    yPred = predictShapeFromTwoGaussian(theta, model);
    r = trueData.yMeas - yPred(model.measIdx);
    J = finiteDifferenceJacobian(@(tt) predictShapeFromTwoGaussian(tt, model, model.measIdx), theta, cfg.aloi.fdStep);
    prior = cfg.aloi.priorWeight * (theta - cfg.aloi.theta0);
    costNow = r.' * Wpos * r + prior.' * prior;
    if costNow < bestCost
        bestCost = costNow;
    end

    A = J.' * Wpos * J + lambda * eye(numel(theta)) + cfg.aloi.priorWeight;
    b = J.' * Wpos * r - cfg.aloi.priorWeight * (theta - cfg.aloi.theta0);
    step = A \ b;

    accepted = false;
    for alpha = [1.0, 0.5, 0.25, 0.1, 0.05]
        thetaCand = projectBounds(theta + alpha * step, cfg.aloi.lb, cfg.aloi.ub);
        yCand = predictShapeFromTwoGaussian(thetaCand, model);
        rCand = trueData.yMeas - yCand(model.measIdx);
        priorCand = cfg.aloi.priorWeight * (thetaCand - cfg.aloi.theta0);
        costCand = rCand.' * Wpos * rCand + priorCand.' * priorCand;
        if costCand < costNow
            theta = thetaCand;
            bestCost = costCand;
            lambda = max(lambda * 0.6, 1e-7);
            accepted = true;
            break;
        end
    end

    if ~accepted
        lambda = min(lambda * 8.0, 1e6);
    end

    if norm(step) < 1e-7
        break;
    end
end
end


function y = predictShapeFromTwoGaussian(theta, model, idx)
if nargin < 3
    idx = [];
end
q = twoGaussianLoad(model.s, theta);
y = model.actShape + model.Phi * q;
if ~isempty(idx)
    y = y(idx);
end
end


function q = twoGaussianLoad(s, theta)
% theta = [F1, mu1, sigma1, F2, mu2, sigma2]
F1 = theta(1); mu1 = theta(2); sig1 = max(theta(3), 1e-4);
F2 = theta(4); mu2 = theta(5); sig2 = max(theta(6), 1e-4);

q1 = F1 * exp(-0.5 * ((s - mu1) ./ sig1).^2) ./ (sqrt(2 * pi) * sig1);
q2 = F2 * exp(-0.5 * ((s - mu2) ./ sig2).^2) ./ (sqrt(2 * pi) * sig2);
q = q1 + q2;
end


function x = projectBounds(x, lb, ub)
x = min(max(x, lb), ub);
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


function printAloiSummary(results, cfg)
est = results.estimation;
fprintf('\n=== Aloi 2022 Gaussian Load Estimation ===\n');
fprintf('Output directory: %s\n\n', cfg.outputDir);
fprintf('True loads:\n');
fprintf('  Load 1: %.3f N at s = %.3f m\n', cfg.load1.magnitude, cfg.load1.position);
fprintf('  Load 2: %.3f N at s = %.3f m\n', cfg.load2.magnitude, cfg.load2.position);
fprintf('\nEstimated loads:\n');
fprintf('  Load 1: %.3f N at s = %.3f m\n', est.load1Est(2), est.load1Est(1));
fprintf('  Load 2: %.3f N at s = %.3f m\n', est.load2Est(2), est.load2Est(1));
fprintf('\nErrors:\n');
fprintf('  Load 1 position error: %.1f mm\n', 1e3 * abs(est.load1Est(1) - cfg.load1.position));
fprintf('  Load 1 magnitude error: %.1f mN\n', 1e3 * abs(est.load1Est(2) - cfg.load1.magnitude));
fprintf('  Load 2 position error: %.1f mm\n', 1e3 * abs(est.load2Est(1) - cfg.load2.position));
fprintf('  Load 2 magnitude error: %.1f mN\n', 1e3 * abs(est.load2Est(2) - cfg.load2.magnitude));
fprintf('\nShape RMSE: %.4f mm\n', 1e3 * est.shapeRmse);
fprintf('Optimization cost: %.6f\n', est.cost);
fprintf('Iterations: %d\n', est.iterations);
end


function plotAloiResults(results, cfg)
ensureOutputDir(cfg.outputDir);
model = results.model;
trueData = results.trueData;
est = results.estimation;

fig = figure('Name', 'Aloi Method Results', 'Color', 'w', 'Position', [100 100 1400 900]);

subplot(2, 3, 1);
plot(model.s, trueData.yTrue * 1e3, 'k-', 'LineWidth', 2.5); hold on;
plot(model.s, est.yEst * 1e3, 'r-', 'LineWidth', 2);
plot(model.sMeas, trueData.yMeas * 1e3, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', [0.3 0.6 1.0]);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Deflection y [mm]');
title(sprintf('Shape Estimation (RMSE: %.3f mm)', 1e3 * est.shapeRmse));
legend({'True shape', 'Estimated shape', 'FBG measurements'}, 'Location', 'best');

subplot(2, 3, 2);
stem(model.s, trueData.qTrue, 'k', 'LineWidth', 1.5, 'MarkerSize', 8); hold on;
plot(model.s, est.qEst, 'r-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Load q [N/m]');
title('Load Distribution: True vs Estimated');
legend({'True point loads', 'Estimated (Gaussian)'}, 'Location', 'best');

subplot(2, 3, 3);
plot(model.sMeas, trueData.kappaMeas, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', [0.3 0.6 1.0]); hold on;
plot(model.s, trueData.kappaTrue, 'k-', 'LineWidth', 2);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Curvature \kappa [1/m]');
title('FBG Curvature Measurements');
legend({'FBG measurements', 'True curvature'}, 'Location', 'best');

subplot(2, 3, 4);
plot(model.s, (trueData.yTrue - est.yEst) * 1e3, 'r-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
grid on; box on;
xlabel('Arc length s [m]'); ylabel('Shape error [mm]');
title('Shape Estimation Error');

subplot(2, 3, 5);
bar([cfg.load1.position, cfg.load2.position], ...
    [cfg.load1.magnitude, cfg.load2.magnitude], 0.02, 'FaceColor', 'k'); hold on;
bar([est.load1Est(1), est.load2Est(1)], ...
    [est.load1Est(2), est.load2Est(2)], 0.015, 'FaceColor', 'r');
grid on; box on;
xlabel('Position [m]'); ylabel('Force magnitude [N]');
title('Load Locations and Magnitudes');
legend({'True', 'Estimated'}, 'Location', 'best');

subplot(2, 3, 6);
errorPos = [abs(est.load1Est(1) - cfg.load1.position), abs(est.load2Est(1) - cfg.load2.position)] * 1e3;
errorMag = [abs(est.load1Est(2) - cfg.load1.magnitude), abs(est.load2Est(2) - cfg.load2.magnitude)] * 1e3;
bar([errorPos; errorMag].');
set(gca, 'XTickLabel', {'Load 1', 'Load 2'});
ylabel('Error');
title('Estimation Errors');
legend({'Position [mm]', 'Magnitude [mN]'}, 'Location', 'best');
grid on;

sgtitle('Aloi 2022 Method: Two Point Loads with FBG Sensing');
saveFigure(fig, fullfile(cfg.outputDir, 'aloi_method_results.png'));
end


function plotAloiFig6Style(results, cfg)
% Plot similar to Fig. 6(b) in Aloi paper
ensureOutputDir(cfg.outputDir);
model = results.model;
trueData = results.trueData;
est = results.estimation;

fig = figure('Name', 'Aloi Fig 6 Style', 'Color', 'w', 'Position', [150 150 1000 800]);

% Convert to x-y coordinates for plotting
x = model.s;
y = trueData.yTrue;

% Plot robot shape
plot(x * 1e3, y * 1e3, 'k-', 'LineWidth', 3); hold on;
axis equal;
grid on; box on;

% Plot FBG marker locations (green circles)
xMeas = model.sMeas;
yMeas = trueData.yTrue(model.measIdx);
plot(xMeas * 1e3, yMeas * 1e3, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);

% Plot true applied forces (yellow arrows)
forceScale = 150;  % Scale factor for visualization
xLoad1 = model.s(trueData.load1Idx);
yLoad1 = trueData.yTrue(trueData.load1Idx);
thetaLoad1 = trueData.thetaTrue(trueData.load1Idx);
forceDir1 = [sin(thetaLoad1), cos(thetaLoad1)];
quiver(xLoad1 * 1e3, yLoad1 * 1e3, ...
       -forceDir1(2) * cfg.load1.magnitude * forceScale, ...
       forceDir1(1) * cfg.load1.magnitude * forceScale, ...
       0, 'Color', [0.9 0.7 0.1], 'LineWidth', 4, 'MaxHeadSize', 1.5);

xLoad2 = model.s(trueData.load2Idx);
yLoad2 = trueData.yTrue(trueData.load2Idx);
thetaLoad2 = trueData.thetaTrue(trueData.load2Idx);
forceDir2 = [sin(thetaLoad2), cos(thetaLoad2)];
quiver(xLoad2 * 1e3, yLoad2 * 1e3, ...
       -forceDir2(2) * cfg.load2.magnitude * forceScale, ...
       forceDir2(1) * cfg.load2.magnitude * forceScale, ...
       0, 'Color', [0.9 0.7 0.1], 'LineWidth', 4, 'MaxHeadSize', 1.5);

% Plot estimated load distribution (red line with markers)
qNorm = est.qEst / max(abs(est.qEst));
loadScale = 30;  % Scale for load visualization
for i = 1:5:cfg.nGrid
    xi = model.s(i);
    yi = trueData.yTrue(i);
    thetai = trueData.thetaTrue(i);
    qi = qNorm(i);
    if abs(qi) > 0.1
        dirVec = [-sin(thetai), cos(thetai)];
        plot([xi * 1e3, (xi + dirVec(1) * qi * loadScale * 1e-3) * 1e3], ...
             [yi * 1e3, (yi + dirVec(2) * qi * loadScale * 1e-3) * 1e3], ...
             'r-', 'LineWidth', 2);
        plot(xi * 1e3, yi * 1e3, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    end
end

xlabel('x [mm]');
ylabel('y [mm]');
title('2D Continuum Robot with Two Point Loads (Aloi Method)');
legend({'Robot shape', 'FBG marker locations', 'Applied force (true)', '', 'Estimated load'}, ...
       'Location', 'best', 'FontSize', 11);

% Add text annotations
text(10, max(y)*1e3*0.9, sprintf('Load 1: %.0f mN @ %.0f mm', ...
     cfg.load1.magnitude*1e3, cfg.load1.position*1e3), 'FontSize', 10);
text(10, max(y)*1e3*0.8, sprintf('Load 2: %.0f mN @ %.0f mm', ...
     cfg.load2.magnitude*1e3, cfg.load2.position*1e3), 'FontSize', 10);
text(10, max(y)*1e3*0.7, sprintf('Shape RMSE: %.2f mm', est.shapeRmse*1e3), 'FontSize', 10);

saveFigure(fig, fullfile(cfg.outputDir, 'aloi_fig6_style.png'));
fprintf('\nFig 6 style plot saved to: %s\n', fullfile(cfg.outputDir, 'aloi_fig6_style.png'));
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
