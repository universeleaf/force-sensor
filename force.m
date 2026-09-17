function results = force(scenario, quickMode)
%FORCE Run the current rod-plane experiment from the workspace root.
% force() / force('wall')   : separated body contact, strict fmincon solver
% force('senior')           : horizontal plane, contact at the tip
% force('wall', true)       : approximate pipeline smoke test
% Historical standalone examples are preserved in legacy/.
if nargin < 1, scenario = 'wall'; end
if nargin < 2, quickMode = false; end
if islogical(scenario)
    quickMode = scenario; scenario = 'wall';
end
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'rod'));
switch lower(char(scenario))
    case 'wall'
        results = simu_rod_plane_displacement_force_sensing(quickMode);
    case 'senior'
        results = simu_rod_plane_senior_geometry_force_sensing(quickMode);
    otherwise
        error('force:UnknownScenario', 'Use force(''wall'') or force(''senior'').');
end
end
