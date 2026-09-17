% Simple test script for Cosserat rod implementation
clearvars;
clc;

fprintf('Testing Cosserat rod helper functions...\n');

% Load helpers
helpers = cosseratHelpers();

% Test parameters
L = 0.3;  % 30 cm rod
nGrid = 51;
s = linspace(0, L, nGrid)';
ds = s(2) - s(1);

% Rod properties
EI = 0.03;  % Bending stiffness
K = [EI; EI; EI*0.5];  % [EI_x, EI_y, GJ]

% Base transformation
T_base = eye(4);

% No intrinsic curvature
u_hat = zeros(3, nGrid);

% Test 1: Unloaded rod (should remain straight)
fprintf('\nTest 1: Unloaded rod\n');
f_ext = zeros(3, nGrid);
[u, p, R, converged] = helpers.solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, 100, 1e-6);
fprintf('  Converged: %d\n', converged);
fprintf('  Max deflection: %.6f m\n', max(abs(p(2,:))));
fprintf('  Expected: ~0 (straight rod)\n');

% Test 2: Tip load
fprintf('\nTest 2: Tip load (0.1 N at tip)\n');
f_ext = zeros(3, nGrid);
f_ext(2, end) = -0.1 / ds;  % Point load at tip
[u, p, R, converged] = helpers.solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, 100, 1e-6);
fprintf('  Converged: %d\n', converged);
fprintf('  Tip deflection: %.6f m (%.2f mm)\n', p(2, end), p(2, end)*1e3);

% Analytical solution for tip load: delta = FL^3/(3EI)
F = 0.1;
delta_analytical = F * L^3 / (3 * EI);
fprintf('  Analytical (beam theory): %.6f m (%.2f mm)\n', delta_analytical, delta_analytical*1e3);
fprintf('  Relative error: %.2f%%\n', abs(p(2,end) - delta_analytical)/delta_analytical * 100);

% Test 3: Distributed load
fprintf('\nTest 3: Uniform distributed load\n');
q_uniform = 0.5;  % N/m
f_ext = zeros(3, nGrid);
f_ext(2, :) = -q_uniform;
[u, p, R, converged] = helpers.solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, 100, 1e-6);
fprintf('  Converged: %d\n', converged);
fprintf('  Tip deflection: %.6f m (%.2f mm)\n', p(2, end), p(2, end)*1e3);

% Analytical solution for uniform load: delta = qL^4/(8EI)
delta_analytical_uniform = q_uniform * L^4 / (8 * EI);
fprintf('  Analytical (beam theory): %.6f m (%.2f mm)\n', delta_analytical_uniform, delta_analytical_uniform*1e3);
fprintf('  Relative error: %.2f%%\n', abs(p(2,end) - delta_analytical_uniform)/delta_analytical_uniform * 100);

% Plot results
figure('Name', 'Cosserat Rod Test', 'Position', [100 100 1200 400]);

subplot(1, 3, 1);
plot(s*1e3, p(2,:)*1e3, 'b-', 'LineWidth', 2);
grid on; xlabel('Arc length [mm]'); ylabel('Deflection [mm]');
title('Test 1: Unloaded Rod');

subplot(1, 3, 2);
f_ext = zeros(3, nGrid);
f_ext(2, end) = -0.1 / ds;
[u, p, R, converged] = helpers.solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, 100, 1e-6);
plot(s*1e3, p(2,:)*1e3, 'r-', 'LineWidth', 2); hold on;
s_analytical = linspace(0, L, 100);
y_analytical = F * s_analytical.^2 .* (3*L - s_analytical) / (6*EI);
plot(s_analytical*1e3, y_analytical*1e3, 'k--', 'LineWidth', 1.5);
grid on; xlabel('Arc length [mm]'); ylabel('Deflection [mm]');
title('Test 2: Tip Load');
legend('Cosserat', 'Analytical', 'Location', 'best');

subplot(1, 3, 3);
f_ext = zeros(3, nGrid);
f_ext(2, :) = -q_uniform;
[u, p, R, converged] = helpers.solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, 100, 1e-6);
plot(s*1e3, p(2,:)*1e3, 'g-', 'LineWidth', 2); hold on;
y_analytical_uniform = q_uniform * s_analytical.^2 .* (L^2 - s_analytical.^2/2) / (24*EI);
plot(s_analytical*1e3, y_analytical_uniform*1e3, 'k--', 'LineWidth', 1.5);
grid on; xlabel('Arc length [mm]'); ylabel('Deflection [mm]');
title('Test 3: Uniform Load');
legend('Cosserat', 'Analytical', 'Location', 'best');

fprintf('\nAll tests completed!\n');
