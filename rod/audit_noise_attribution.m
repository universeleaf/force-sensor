function report=audit_noise_attribution(runId)
% Verify the control experiment changed exactly the claimed sensor field.
root=fileparts(fileparts(mfilename('fullpath')));parent=fullfile(root,'out','accuracy','noise');
if nargin<1
    source=jsondecode(fileread(fullfile(parent,'comparison.json')));runId=source.runRecord.runId;
end
folder=fullfile(parent,runId);source=jsondecode(fileread(fullfile(folder,'comparison.json')));
assert(strcmp(source.state,'complete'),'rod:IncompleteNoiseRun','Finish noise controls before auditing.');
baseline=load(fullfile(root,'out','accuracy',source.baselineRunId,'sliding_noisy','results.mat'),'sensorInput');
reference=baseline.sensorInput;reference.config.forceSensor.frameCheckpointDirectory='';
report=struct('runId',runId,'baselineRunId',source.baselineRunId, ...
    'auditorSha256',file_sha256([mfilename('fullpath') '.m']), ...
    'scope','Validate single-field isolation; not independent force-accuracy or continuous-geometry certification.', ...
    'allPassed',false,'cases',{{}});
for j=1:numel(source.cases)
    entry=source.cases(j);d=load(fullfile(folder,entry.id,'results.mat'),'sensorInput','output');
    control=d.sensorInput;control.config.forceSensor.frameCheckpointDirectory='';
    changed=entry.changedObservation;
    assert(~isequal(control.packet.(changed),reference.packet.(changed)), ...
        'rod:InvalidNoiseControl','The declared observation did not change.');
    control.packet.(changed)=reference.packet.(changed);
    item=struct('id',entry.id,'singleFieldIsolationPassed',isequaln(control,reference), ...
        'maxConstraintResidual',max(d.output.ours.activeConstraintResidual), ...
        'optimizerWarnings',d.output.quality.hasOptimizationWarning);
    consistent=d.output.quality.frictionKinematicsEnforced & d.output.quality.complementarityResidual<1e-3 & ...
        d.output.ours.frictionWMin>=-1e-5;
    item.frictionObservationQuality=friction_direction_quality(d.output.ours,consistent);
    report.cases{j}=item;
end
report.allPassed=all(cellfun(@(c)c.singleFieldIsolationPassed,report.cases));
atomic_write_artifact(fullfile(folder,'isolation_audit.json'),'json',report);
assert(report.allPassed,'rod:InvalidNoiseControl','More than the declared observation changed; inspect isolation_audit.json.');
fprintf('All noise controls change exactly one declared observation field.\n');
end
