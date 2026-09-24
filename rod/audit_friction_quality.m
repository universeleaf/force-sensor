function report=audit_friction_quality(runId)
% Re-evaluate observational flags without recomputing or overwriting forces.
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'out','accuracy');
if nargin<1
    source=jsondecode(fileread(fullfile(folder,'comparison.json')));runId=source.runRecord.runId;
end
folder=fullfile(folder,runId);source=jsondecode(fileread(fullfile(folder,'comparison.json')));
assert(strcmp(source.state,'complete'),'rod:IncompleteAccuracyRun','Finish the run before auditing quality.');
report=struct('runId',runId,'scope',['Post-hoc observation-quality labels only; stored forces, solver exits and original labels retained. ', ...
    'An optimized MPCC mode does not establish direction observability from noisy FBG.'], ...
    'auditorSha256',file_sha256([mfilename('fullpath') '.m']), ...
    'qualityFunctionSha256',file_sha256(fullfile(root,'rod','friction_direction_quality.m')),'cases',{{}});
for j=1:numel(source.cases)
    entry=source.cases(j);d=load(fullfile(folder,entry.id,'results.mat'),'output');o=d.output.ours;
    consistent=o.frictionKinematicsEnforced & max([o.normalComplementarity;o.frictionComplementarity; ...
        o.coneComplementarity],[],1)<1e-3 & o.frictionWMin>=-1e-5;
    item=friction_direction_quality(o,consistent);item.id=entry.id;
    item.originalRequiresReview=d.output.quality.requiresReview;
    item.requiresReview=item.originalRequiresReview | item.hasFrictionObservationWarning;
    report.cases{j}=item;
end
atomic_write_artifact(fullfile(folder,'quality_audit.json'),'json',report);
fprintf('Observation-resolution quality audit saved to %s\n',folder);
end
