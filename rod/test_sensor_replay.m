function report = test_sensor_replay(folder)
%TEST_SENSOR_REPLAY Recompute estimates using the isolated sensor input only.
% Comparison labels are loaded only AFTER estimation has finished.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1, folder=fullfile(root,'out','stage1','video_zero_reduced'); end
d=load(fullfile(folder,'sensor_input.mat'),'sensorInput'); input=d.sensorInput;
assert(~any(isfield(input,{'forward','results','truth'})));
assert(~any(isfield(input.config,{'tipLoadN','planePointMm','forward'})));
estimate=run_rod_plane_force_sensing_experiment('replay-sensors',input);
reference=load(fullfile(folder,'inverse_results.mat'),'results');
report.maxStateDifference=max(abs(estimate.ours.state-reference.results.ours.state),[],'all');
report.maxForceDifferenceN=max(abs(estimate.ours.totalForceResultant-reference.results.ours.totalForceResultant),[],'all');
assert(report.maxStateDifference<1e-8 && report.maxForceDifferenceN<1e-8, ...
    'Isolated packet replay differs from the experiment estimator.');
report.passed=true;
fid=fopen(fullfile(folder,'replay_check.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true)); fclose(fid);
disp(report);
end
