function report=run_accuracy_benchmark(sceneIds)
% Paired archived-input evaluation of rod geometry and FBG likelihood fixes.
if nargin<1
    sceneIds={'sliding_clean','sliding_noisy','ceiling_hook','side_wall','inclined_plane','long_soft_rod'};
end
report=run_plane_geometry_benchmark(sceneIds,'496dc953-eb47-499d-9e38-bd83a92dd98b',1e-6);
end
