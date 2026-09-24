function results = run_sensor_stage1(caseName, coordinates, useShapeOnlySeed)
%RUN_SENSOR_STAGE1 Reproducible sensor-only-history experiments.
% Saved legacy artifacts supply scenario configuration only, not estimator
% history or force labels. Every case generates fresh forward/measurement data.
if nargin<1, caseName='wall_tip'; end
if nargin<2, coordinates='mode-reduced'; end
if nargin<3, useShapeOnlySeed=true; end
root=fileparts(fileparts(mfilename('fullpath')));
source='wall';
if startsWith(caseName,'video'), source='video'; end
matName='results.mat'; if strcmp(source,'video'), matName='inverse_results.mat'; end
d=load(fullfile(root,'out',source,matName),'results'); o=d.results.config;
o.scenarioName=['sensor_stage1_' caseName];
o.outputDir=fullfile(root,'out','stage1',caseName);
if useShapeOnlySeed
    o.outputDir=[o.outputDir '_seeded'];
    if strcmp(caseName,'video_zero'), o.outputDir=fullfile(root,'out','stage1','video_seeded'); end
    o.scenarioName=[o.scenarioName '_seeded'];
elseif strcmp(coordinates,'mode-reduced')
    o.outputDir=[o.outputDir '_reduced'];
else
    o.outputDir=[o.outputDir '_full'];
end
o.forceSensor.subproblemCoordinates=coordinates;
o.forceSensor.useShapeOnlySeed=useShapeOnlySeed;
o.sensing.historySource='sparse-paired';
o.sensing.curvatureInterpolation='intrinsic-delta';
o.sensing.samplePeriodSeconds=0.02;
o.sensing.planeNormalNoiseStdDeg=0;
o.sensing.curvatureNoiseStd=0;
o.tipLoadN=zeros(3,1);
o.diagnostics.stopAfterInverse=true;
o.diagnostics.runMapCandidateCosts=false;
o.video.enabled=false;
switch caseName
    case {'wall_zero','video_zero'}
    case 'wall_tip'
        o.tipLoadN=[2;0;-1];
    case 'wall_tip_reverse'
        o.tipLoadN=[-2;0;1];
    case 'video_tip'
        o.tipLoadN=[1;0;-1];
    otherwise
        error('Unknown case: %s',caseName);
end
results=run_rod_plane_force_sensing_experiment(false,o);
results=save_result_checkpoint(results,o.outputDir,'inverse_results.mat');
export_inverse_audit(results);
plot_sensor_stage1(o.outputDir,true);
end
