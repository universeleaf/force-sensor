function report=audit_accuracy_run(runId)
% Post-hoc diagnostics. Truth is loaded ONLY here/benchmark scoring, never
% supplied to the estimator or used to select likelihood weights.
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'out','accuracy');
if nargin<1
    source=jsondecode(fileread(fullfile(folder,'comparison.json')));runId=source.runRecord.runId;
end
folder=fullfile(folder,runId);source=jsondecode(fileread(fullfile(folder,'comparison.json')));
try
    physics=validate_plane_geometry_run(runId,'accuracy');
catch err
    if ~strcmp(err.identifier,'rod:GeometryAuditFailure'),rethrow(err);end
    % Keep diagnostics for failed runs as well; retain the failing exit below.
    physics=jsondecode(fileread(fullfile(folder,'physics_audit.json')));
end
report=struct('runId',runId,'physicsPassed',physics.allPassed, ...
    'scope',['Force sensitivities condition on rod/base/contact arclength and omit environment/contact constraints. ', ...
    'They describe noise amplification, not calibrated force intervals or an impossibility bound.'], ...
    'auditorSha256',file_sha256([mfilename('fullpath') '.m']),'cases',{{}});
for j=1:numel(source.cases)
    id=source.cases(j).id;d=load(fullfile(folder,id,'results.mat'));
    o=d.output.ours;nt=size(o.state,2);idx=d.sensorInput.packet.fbgIdx;
    sigma=d.sensorInput.config.forceSensor.measurementStd.curvature;
    item=struct('id',id,'curvatureLikelihoodStdPerMm',sigma, ...
        'bendingPredictionTruthRmsePerMm',zeros(1,nt),'bendingObservationRmsePerMm',zeros(1,nt), ...
        'conditionalForceStdN',nan(6,nt),'sensitivityConditionNumber',zeros(1,nt), ...
        'intermediateNonpositiveExits',zeros(1,nt),'rejectedMechanicsTrials',zeros(1,nt));
    for k=1:nt
        prediction=o.u(1:2,idx,k);truth=d.truth.u(1:2,idx,k);
        measurement=d.sensorInput.packet.curvaturePerMm(1:2,:,k);
        item.bendingPredictionTruthRmsePerMm(k)=sqrt(mean((prediction-truth).^2,'all'));
        item.bendingObservationRmsePerMm(k)=sqrt(mean((prediction-measurement).^2,'all'));
        f=o.forceSensitivity{k};item.conditionalForceStdN(:,k)=f.conditionalNoiseStdN;
        item.sensitivityConditionNumber(k)=f.conditionNumber;
        trace=o.solverTrace{k}{1};
        item.intermediateNonpositiveExits(k)=sum(cellfun(@(s)s.exitflag<=0,trace.homotopyTrace));
        item.rejectedMechanicsTrials(k)=trace.rejectedMechanicsTrials+trace.polish.rejectedMechanicsTrials;
    end
    item.bendingPredictionTruthRmseInLikelihoodSigma=item.bendingPredictionTruthRmsePerMm/sigma;
    report.cases{j}=item;
end
atomic_write_artifact(fullfile(folder,'diagnostics.json'),'json',report);
fprintf('Accuracy diagnostics and independent physics audit saved to %s\n',folder);
assert(physics.allPassed,'rod:GeometryAuditFailure','Physics audit failed; diagnostics were retained.');
end
