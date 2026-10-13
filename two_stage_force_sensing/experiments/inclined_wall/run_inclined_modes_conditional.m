function run_inclined_modes_conditional(maxFrames,tag,fbgLimit)
% Same physics and local solves; accept the first mode with an adequate fit.
% fbgLimit is RMS in whitened sensor units, NOT physical curvature or cost.
if nargin<1,maxFrames=Inf;end
if nargin<2,tag='inclined_wall_modes_conditional_1e6_20261010';end
if nargin<3,fbgLimit=0.01;end % Approved raw RMS 1e-6 /mm, sensor sigma 1e-4 /mm.
search=struct('modeSearch','first-acceptable','allAssignments',false, ...
 'fbgRmsAcceptance',fbgLimit,'solver','conditional-modes/fmincon-SQP', ...
 'acceptanceRule','positive native exit AND violation <= 1e-6 AND whitened FBG RMS <= fbgRmsAcceptance', ...
 'noAcceptableMode','exhaust search, return best feasible objective or least violation; flag modeAccepted=false');
run_inclined_modes(maxFrames,tag,search);
end
