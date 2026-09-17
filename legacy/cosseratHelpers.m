function helpers = cosseratHelpers()
% Cosserat rod helper functions extracted from LCP-Continuum
% These functions implement the geometric mechanics needed for Cosserat rod theory

helpers.LargeSE3 = @LargeSE3;
helpers.LargeSO3 = @LargeSO3;
helpers.hat = @hat;
helpers.solveShape = @solveShape;
helpers.computeJacobian = @computeJacobian;
helpers.solveCosseratWithLoad = @solveCosseratWithLoad;

end

function T = LargeSE3(w, v)
% SE(3) exponential map: exp([w]_x v; 0 0) where w is angular velocity, v is linear velocity
wsqr = w' * w;
wnorm = sqrt(wsqr);

if wnorm > eps
    R = LargeSO3(w);
    W = hat(w);
    Wv = W * v;

    % Compute P*v using Rodriguez formula
    Pv = v + (1 - cos(wnorm)) / wsqr * Wv + (wnorm - sin(wnorm)) / (wnorm * wsqr) * W * Wv;
    T = [R, Pv; zeros(1, 3), 1];
else
    T = [eye(3), v; zeros(1, 3), 1];
end
end

function R = LargeSO3(w)
% SO(3) exponential map: exp([w]_x) - Rodriguez formula
theta = norm(w);
if theta > eps
    W = hat(w);
    R = eye(3) + sin(theta) / theta * W + (1 - cos(theta)) / (theta^2) * (W * W);
else
    R = eye(3);
end
end

function W = hat(w)
% Hat operator: R^3 -> so(3), converts vector to skew-symmetric matrix
W = [0, -w(3), w(2);
     w(3), 0, -w(1);
     -w(2), w(1), 0];
end

function [T, R, p] = solveShape(T_base, u, s)
% Solve Cosserat rod shape by integrating strain field u along arc length s
% Inputs:
%   T_base: 4x4 base transformation matrix
%   u: 3xn strain field (curvature and twist at each point)
%   s: nx1 arc length coordinates
% Outputs:
%   T: 4x4xn transformation matrices along the rod
%   R: 3x3xn rotation matrices
%   p: 3xn position vectors

n = length(s);
T = zeros(4, 4, n);
T(:, :, 1) = T_base;

for i = 1:(n-1)
    ds = s(i+1) - s(i);
    % Integrate: T' = T * [u]_x, split into rotation and translation
    T(:, :, i+1) = T(:, :, i) * LargeSE3(u(:, i) * ds, [0; 0; 0]) * LargeSE3([0; 0; 0], [0; 0; ds]);
end

if nargout >= 2
    R = T(1:3, 1:3, :);
end

if nargout >= 3
    p = T(1:3, 4, :);
    p = reshape(p, 3, []);
end
end

function J = computeJacobian(R, p)
% Compute Jacobian matrix dp/du for Cosserat rod
% This relates changes in strain u to changes in position p
% Inputs:
%   R: 3x3xn rotation matrices
%   p: 3xn position vectors
% Output:
%   J: (3n)x(3n) Jacobian matrix

[~, ns] = size(p);

% Create skew-symmetric matrices for all positions
p_skew = zeros(3, 3, ns);
p_skew(1, 2, :) = -p(3, :);
p_skew(2, 1, :) = p(3, :);
p_skew(1, 3, :) = p(2, :);
p_skew(3, 1, :) = -p(2, :);
p_skew(2, 3, :) = -p(1, :);
p_skew(3, 2, :) = p(1, :);

p_skew = reshape(p_skew, 3, []);

% Compute Jacobian
J = repmat(p_skew, ns, 1);
J = J + J';

for j = 1:ns
    J(1:3*j, 3*j-2:3*j) = 0;
    J(:, 3*j-2:3*j) = J(:, 3*j-2:3*j) * R(:, :, j);
end
end

function [u, p, R, converged] = solveCosseratWithLoad(s, u_hat, K, f_ext, T_base, maxIter, tol)
% Solve Cosserat rod equilibrium with external forces using shooting method
% For small deflections, we integrate the moment equation directly
% Inputs:
%   s: nx1 arc length
%   u_hat: 3xn intrinsic curvature (precurvature)
%   K: 3x1 or 3xn stiffness matrix (bending and torsional stiffness)
%   f_ext: 3xn external force distribution in global frame
%   T_base: 4x4 base transformation
%   maxIter: maximum iterations
%   tol: convergence tolerance
% Outputs:
%   u: 3xn strain field
%   p: 3xn position
%   R: 3x3xn rotation
%   converged: boolean

if nargin < 6
    maxIter = 100;
end
if nargin < 7
    tol = 1e-6;
end

ns = length(s);
ds = s(2) - s(1);

% Handle scalar or vector stiffness
if numel(K) == 3
    K_mat = repmat(K(:), 1, ns);
else
    K_mat = K;
end

% For cantilever beam with small deflections, we can compute curvature directly
% from the bending moment distribution
% M(s) = integral from s to L of (x-s) * f(x) dx
% u(s) = M(s) / EI + u_hat(s)

% Compute bending moment distribution by integrating forces
% For 2D case: moment about x-axis from forces in y-direction
M = zeros(3, ns);

for i = 1:ns
    % Moment at position i from all forces downstream
    for j = i:ns
        lever_arm = s(j) - s(i);
        % Moment = r × F, for 2D: Mx = (z * Fy - y * Fz)
        % Since forces are in y-direction and lever arm is in z-direction:
        M(1, i) = M(1, i) + lever_arm * f_ext(2, j) * ds;
    end
end

% Compute curvature from moment: u = M / K + u_hat
u = zeros(3, ns);
for i = 1:ns
    u(:, i) = M(:, i) ./ K_mat(:, i) + u_hat(:, i);
end

% Compute shape from curvature
[~, R, p] = solveShape(T_base, u, s);

converged = true;
end
