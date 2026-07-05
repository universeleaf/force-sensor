function results = simu_rod_plane_force_sensing_copy(quickMode, overrides)
%SIMU_ROD_PLANE_FORCE_SENSING_COPY
% User-owned copied/modified entry point for the rod_plane experiment.
% The original LCP-Continuum/simulations/simu_rod_plane.m is not modified.
%
% Usage:
%   results = simu_rod_plane_force_sensing_copy();
%   results = simu_rod_plane_force_sensing_copy(true);  % faster smoke run
%   results = simu_rod_plane_force_sensing_copy(true, overrides);

if nargin < 1
    quickMode = false;
end
if nargin < 2
    overrides = struct;
end

results = run_rod_plane_force_sensing_experiment(quickMode, overrides);
end
