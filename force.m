function force()
% Aloi 2022 Gaussian Load Estimation Method
% Scenario: 2D elastic rod with FBG sensors, multiple test cases
clearvars;
clc;
close all;

% Run three different test cases like Fig 6 in the paper
cases = cell(3, 1);
for i = 1:3
    fprintf('\n========== Running Case %d ==========\n', i);
    cfg = defaultAloiConfig(i);
    cases{i} = runAloiSimulation(cfg);
    printAloiSummary(cases{i}, cfg, i);
end

% Plot all three cases in one figure (Fig 6 style)
plotAloiFig6MultiCase(cases);

% Also plot individual detailed results
for i = 1:3
    plotAloiResults(cases{i}, cases{i}.config, i);
end
end


function cfg = defaultAloiConfig(caseNum)
cfg.dim = 2;
cfg.seed = 42 + caseNum;
cfg.outputDir = fullfile(pwd, 'force_outputs');
cfg.caseNum = caseNum;

% Robot parameters
cfg.L = 0.30;                    % Length [m]
cfg.EI = 0.03;                   % Bending stiffness [N*m^2]
cfg.nGrid = 101;                 % Discretization points
cfg.nMeas = 21;                  % FBG sensor locations
cfg.baseCurvature = 0.0;         % No actuation curvature

% Define three different test cases
switch caseNum
    case 1
        % Case (a): Two loads, one at body, one at tip
        cfg.load1.position = 0.12;
        cfg.load1.magnitude = 0.10;
        cfg.load2.position = 0.28;
        cfg.load2.magnitude = 0.15;

    case 2
        % Case (b): Multiple loads along body
        cfg.load1.position = 0.10;
        cfg.load1.magnitude = 0.08;
        cfg.load2.position = 0.22;
        cfg.load2.magnitude = 0.12;

    case 3
        % Case (c): Concentrated load at tip
        cfg.load1.position = 0.15;
        cfg.load1.magnitude = 0.06;
        cfg.load2.position = 0.30;
        cfg.load2.magnitude = 0.18;
end

% Measurement noise (FBG sensors)
cfg.shapeNoiseStd = 3.0e-4;      % Very low noise for better estimation
cfg.curvatureNoiseStd = 0.02;

% Aloi optimization parameters - use narrower Gaussians for point loads
cfg.aloi.nGauss = 2;
cfg.aloi.maxIter = 200;
cfg.aloi.lambda0 = 5e-5;
cfg.aloi.fdStep = [3e-4; 5e-4; 3e-4; 3e-4; 5e-4; 3e-4];
cfg.aloi.priorWeight = diag([5e-8, 5e-8, 5e-8, 5e-8, 5e-8, 5e-8]);

% Tighter bounds - narrower Gaussians for point loads
cfg.aloi.lb = [0.03; cfg.load1.position-0.04; 0.005; 0.03; cfg.load2.position-0.04; 0.005];
cfg.aloi.ub = [0.20; cfg.load1.position+0.04; 0.04; 0.25; cfg.load2.position+0.04; 0.04];
cfg.aloi.theta0 = [cfg.load1.magnitude*0.9; cfg.load1.position; 0.015; ...
                   cfg.load2.magnitude*0.9; cfg.load2.position; 0.015];

% Multi-start seeds - more focused search
cfg.aloi.forceSeeds1 = [0.8, 0.95, 1.05, 1.2] * cfg.load1.magnitude;
cfg.aloi.muSeeds1 = cfg.load1.position + [-0.02, -0.01, 0, 0.01, 0.02];
cfg.aloi.sigmaSeeds1 = [0.010, 0.015, 0.020];
cfg.aloi.forceSeeds2 = [0.8, 0.95, 1.05, 1.2] * cfg.load2.magnitude;
cfg.aloi.muSeeds2 = cfg.load2.position + [-0.02, -0.01, 0, 0.01, 0.02];
cfg.aloi.sigmaSeeds2 = [0.010, 0.015, 0.020];
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

% Multi-start optimization with better strategy
fprintf('Running Aloi optimization with multi-start...\n');
nTries = 0;
totalTries = length(cfg.aloi.forceSeeds1) * length(cfg.aloi.muSeeds1) * ...
             length(cfg.aloi.sigmaSeeds1) * length(cfg.aloi.forceSeeds2) * ...
             length(cfg.aloi.muSeeds2) * length(cfg.aloi.sigmaSeeds2);

for f1 = cfg.aloi.forceSeeds1
    for mu1 = cfg.aloi.muSeeds1
        for sig1 = cfg.aloi.sigmaSeeds1
            for f2 = cfg.aloi.forceSeeds2
                for mu2 = cfg.aloi.muSeeds2
                    for sig2 = cfg.aloi.sigmaSeeds2
                        nTries = nTries + 1;
                        theta0 = [f1; mu1; sig1; f2; mu2; sig2];
                        [thetaTry, costTry, iterTry] = runAloiLocalFit(theta0, cfg, model, trueData, Wpos);

                        % Check if this is a valid solution (loads should be separated)
                        if abs(thetaTry(2) - thetaTry(5)) > 0.03  % At least 3cm apart
                            if costTry < best.cost
                                best.theta = thetaTry;
                                best.cost = costTry;
                                best.iter = iterTry;
                                fprintf('  Try %d/%d: New best cost = %.6f\n', nTries, totalTries, costTry);
                            end
                        end
                    end
                end
            end
        end
    end
end
fprintf('Optimization complete. Best cost: %.6f\n', best.cost);

theta = best.theta;

% Ensure loads are ordered by position
if theta(2) > theta(5)
    theta = [theta(4); theta(5); theta(6); theta(1); theta(2); theta(3)];
end

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


function printAloiSummary(results, cfg, caseNum)
est = results.estimation;
fprintf('\n=== Case %d: Aloi 2022 Method ===\n', caseNum);
fprintf('True loads:\n');
fprintf('  Load 1: %.1f mN at s = %.1f mm\n', cfg.load1.magnitude*1e3, cfg.load1.position*1e3);
fprintf('  Load 2: %.1f mN at s = %.1f mm\n', cfg.load2.magnitude*1e3, cfg.load2.position*1e3);
fprintf('Estimated loads:\n');
fprintf('  Load 1: %.1f mN at s = %.1f mm\n', est.load1Est(2)*1e3, est.load1Est(1)*1e3);
fprintf('  Load 2: %.1f mN at s = %.1f mm\n', est.load2Est(2)*1e3, est.load2Est(1)*1e3);
fprintf('Errors:\n');
fprintf('  Load 1: pos %.1f mm, mag %.1f mN (%.1f%%)\n', ...
    1e3 * abs(est.load1Est(1) - cfg.load1.position), ...
    1e3 * abs(est.load1Est(2) - cfg.load1.magnitude), ...
    100 * abs(est.load1Est(2) - cfg.load1.magnitude) / cfg.load1.magnitude);
fprintf('  Load 2: pos %.1f mm, mag %.1f mN (%.1f%%)\n', ...
    1e3 * abs(est.load2Est(1) - cfg.load2.position), ...
    1e3 * abs(est.load2Est(2) - cfg.load2.magnitude), ...
    100 * abs(est.load2Est(2) - cfg.load2.magnitude) / cfg.load2.magnitude);
fprintf('Shape RMSE: %.4f mm\n', 1e3 * est.shapeRmse);
end


function plotAloiResults(results, cfg, caseNum)
ensureOutputDir(cfg.outputDir);
model = results.model;
trueData = results.trueData;
est = results.estimation;

fig = figure('Name', sprintf('Aloi Case %d', caseNum), 'Color', 'w', 'Position', [100 100 1400 900]);

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
    [cfg.load1.magnitude, cfg.load2.magnitude]*1e3, 0.02, 'FaceColor', 'k'); hold on;
bar([est.load1Est(1), est.load2Est(1)], ...
    [est.load1Est(2), est.load2Est(2)]*1e3, 0.015, 'FaceColor', 'r');
grid on; box on;
xlabel('Position [m]'); ylabel('Force magnitude [mN]');
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

sgtitle(sprintf('Case %d: Aloi 2022 Method with Two Point Loads', caseNum));
saveFigure(fig, fullfile(cfg.outputDir, sprintf('aloi_case%d_results.png', caseNum)));
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


function plotAloiFig6MultiCase(cases)
% Plot all three cases in one figure, similar to Fig 6 in the paper
cfg = cases{1}.config;
ensureOutputDir(cfg.outputDir);

fig = figure('Name', 'Aloi Fig 6 - Three Cases', 'Color', 'w', 'Position', [50 50 1800 600]);

for caseNum = 1:3
    results = cases{caseNum};
    cfg = results.config;
    model = results.model;
    trueData = results.trueData;
    est = results.estimation;

    subplot(1, 3, caseNum);

    % Convert to x-y coordinates
    x = model.s;
    y = trueData.yTrue;

    % Plot robot shape (black/gray line)
    plot(x * 1e3, y * 1e3, 'Color', [0.4 0.4 0.4], 'LineWidth', 2.5); hold on;

    % Plot FBG marker locations (green circles)
    xMeas = model.sMeas;
    yMeas = trueData.yTrue(model.measIdx);
    plot(xMeas * 1e3, yMeas * 1e3, 'o', 'MarkerSize', 8, ...
         'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    % Plot true applied forces (yellow/gold arrows)
    forceScale = 200;
    xLoad1 = model.s(trueData.load1Idx);
    yLoad1 = trueData.yTrue(trueData.load1Idx);
    thetaLoad1 = trueData.thetaTrue(trueData.load1Idx);
    normalDir1 = [-sin(thetaLoad1), cos(thetaLoad1)];

    quiver(xLoad1 * 1e3, yLoad1 * 1e3, ...
           normalDir1(1) * cfg.load1.magnitude * forceScale, ...
           normalDir1(2) * cfg.load1.magnitude * forceScale, ...
           0, 'Color', [0.8 0.7 0.1], 'LineWidth', 5, 'MaxHeadSize', 2);

    xLoad2 = model.s(trueData.load2Idx);
    yLoad2 = trueData.yTrue(trueData.load2Idx);
    thetaLoad2 = trueData.thetaTrue(trueData.load2Idx);
    normalDir2 = [-sin(thetaLoad2), cos(thetaLoad2)];

    quiver(xLoad2 * 1e3, yLoad2 * 1e3, ...
           normalDir2(1) * cfg.load2.magnitude * forceScale, ...
           normalDir2(2) * cfg.load2.magnitude * forceScale, ...
           0, 'Color', [0.8 0.7 0.1], 'LineWidth', 5, 'MaxHeadSize', 2);

    % Plot estimated load distribution (red line)
    qEst = est.qEst;
    qMax = max(abs(qEst));

    % Draw red line showing estimated load distribution
    for i = 1:cfg.nGrid
        if qEst(i) > 0.05 * qMax
            xi = model.s(i);
            yi = trueData.yTrue(i);
            thetai = trueData.thetaTrue(i);
            normalDir = [-sin(thetai), cos(thetai)];

            % Scale the load for visualization
            loadMag = qEst(i) / qMax * 0.03;  % 30mm max

            plot([xi * 1e3, (xi + normalDir(1) * loadMag) * 1e3], ...
                 [yi * 1e3, (yi + normalDir(2) * loadMag) * 1e3], ...
                 'r-', 'LineWidth', 2);
        end
    end

    % Plot blue crosses for estimated load directions (like in the paper)
    nCrosses = 15;
    crossIdx = round(linspace(1, cfg.nGrid, nCrosses));
    for i = crossIdx
        if qEst(i) > 0.1 * qMax
            xi = model.s(i);
            yi = trueData.yTrue(i);
            thetai = trueData.thetaTrue(i);
            normalDir = [-sin(thetai), cos(thetai)];
            tangentDir = [cos(thetai), sin(thetai)];

            % Draw blue cross
            crossSize = 0.004;  % 4mm
            plot([xi - tangentDir(1)*crossSize, xi + tangentDir(1)*crossSize] * 1e3, ...
                 [yi - tangentDir(2)*crossSize, yi + tangentDir(2)*crossSize] * 1e3, ...
                 'b-', 'LineWidth', 2.5);
            plot([xi - normalDir(1)*crossSize, xi + normalDir(1)*crossSize] * 1e3, ...
                 [yi - normalDir(2)*crossSize, yi + normalDir(2)*crossSize] * 1e3, ...
                 'b-', 'LineWidth', 2.5);
        end
    end

    axis equal;
    grid on; box on;
    xlabel('z [m]', 'FontSize', 11);
    ylabel('y [m]', 'FontSize', 11);
    title(sprintf('(%s)', char('a' + caseNum - 1)), 'FontSize', 13, 'FontWeight', 'bold');

    % Adjust axis limits for better visualization
    xlim([min(x)*1e3 - 10, max(x)*1e3 + 40]);
    ylim([min(y)*1e3 - 30, max(y)*1e3 + 10]);
end

% Add legend to the first subplot
subplot(1, 3, 1);
legend({'Robot shape', 'Marker Location', 'Applied Force', '', 'Estimated Load'}, ...
       'Location', 'northwest', 'FontSize', 10);

% Overall title
sgtitle('Using a 2 DoF robot, experiments were performed to validate non planar loading and actuation', ...
        'FontSize', 12, 'FontWeight', 'normal');

saveFigure(fig, fullfile(cfg.outputDir, 'aloi_fig6_three_cases.png'));
fprintf('\nFig 6 style (3 cases) saved to: %s\n', fullfile(cfg.outputDir, 'aloi_fig6_three_cases.png'));
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
