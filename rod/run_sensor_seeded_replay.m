function results = run_sensor_seeded_replay(sourceFolder,outputFolder)
% Estimate from isolated packet first; load scoring truth only afterwards.
d=load(fullfile(sourceFolder,'sensor_input.mat'),'sensorInput'); sensorInput=d.sensorInput;
sensorInput.config.forceSensor.useShapeOnlySeed=true;
sensorInput.config.forceSensor.subproblemCoordinates='mode-reduced';
if ~isfolder(outputFolder), mkdir(outputFolder); end
runRecord=begin_result_run(outputFolder);
atomic_write_artifact(fullfile(outputFolder,'sensor_input.mat'),'mat',struct('sensorInput',sensorInput));
estimate=run_rod_plane_force_sensing_experiment('replay-sensors',sensorInput);
scoring=load(resolve_result_file(sourceFolder),'results'); results=scoring.results;
results.runRecord=runRecord;
results.ours=estimate.ours; results.measurements=estimate.measurements;
results.config.outputDir=outputFolder;
results.config.forceSensor=estimate.config.forceSensor;
results.config.scenarioName=[results.config.scenarioName '_seeded'];
results.metrics.ours.finalRelativeErrorPct=100*norm( ...
    results.ours.totalForceResultant(:,end)-results.forward.totalForceResultant(:,end))/ ...
    max(norm(results.forward.totalForceResultant(:,end)),eps);
try
    results.validation=validate_rod_plane_displacement_results(results);
catch errorInfo
    results.validation=struct('passed',false,'error',errorInfo.message);
    results=save_result_checkpoint(results,outputFolder,'failed_results.mat');
    rethrow(errorInfo);
end
results=save_result_checkpoint(results,outputFolder,'inverse_results.mat');
export_inverse_audit(results);
plot_sensor_stage1(outputFolder,true);
end
