function report=run_multi_contact_plane_uncertainty()
%RUN_MULTI_CONTACT_PLANE_UNCERTAINTY Replay fixed 1-mm plane error with MAP offsets.
% Prior standard deviations are specified before scoring; no truth is used by
% the inverse. The same nominal-noise packets from multi-benchmark are reused.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','benchmarks','multi_contact');
source=jsondecode(fileread(fullfile(folder,'comparison.json')));
assert(strcmp(source.state,'complete'),'Run force(''multi-benchmark'') first.');
report=struct('schemaVersion',1,'state','running','runRecord',new_run_record(), ...
    'scope',['Replay of saved 24-sensor, 2.5e-5/mm packets with one observed plane ', ...
    'displaced +1 mm along its normal. Inferred offsets have zero-mean Gaussian priors.'], ...
    'methods',{{'fixed_shifted_plane','latent_plane_std_0p1mm','latent_plane_std_1mm'}}, ...
    'cases',{{}},'sourceRunId',source.runRecord.runId);
path=fullfile(folder,'plane_uncertainty.json');
atomic_write_artifact(path,'json',report);
caseIndex=0;
for sceneIndex=1:numel(source.sceneIds)
    sceneId=source.sceneIds{sceneIndex};
    d=load(fullfile(folder,sceneId,'truth.mat'),'truth'); truth=d.truth;
    d=load(fullfile(folder,sceneId,'trials.mat'),'raw'); trials=d.raw;
    for trialIndex=1:numel(trials)
        trial=trials(trialIndex);
        if trial.sensorCount~=24 || trial.noiseStdPerMm~=2.5e-5, continue; end
        shifted=trial.input;
        plane=shifted.model.contactPlaneIndex(1);
        normal=shifted.model.planeNormalXZ(:,plane);
        shifted.model.planePointXZ(:,plane)=shifted.model.planePointXZ(:,plane)+normal/norm(normal);
        methodNames=report.methods;
        standardDeviations=[0 0.1 1];
        for methodIndex=1:numel(methodNames)
            caseIndex=caseIndex+1;
            entry=struct('sceneId',sceneId,'seed',trial.seed,'frameIndex',trial.frameIndex, ...
                'method',methodNames{methodIndex},'planePointStdMm',standardDeviations(methodIndex), ...
                'completed',false);
            try
                estimate=estimate_planar_multi_contact(shifted, ...
                    struct('planePointStdMm',standardDeviations(methodIndex)));
                trueContact=truth.contactForces([1 3],:,trial.frameIndex);
                trueTip=truth.tipForce([1 3],trial.frameIndex);
                delta=estimate.state.contactForceXZ-trueContact;
                entry.contactForceRmseN=sqrt(mean(sum(delta.^2,1)));
                entry.contactArcRmseMm=sqrt(mean((estimate.state.contactS(:)- ...
                    truth.contactS(:,trial.frameIndex)).^2));
                entry.totalForceErrorN=norm(sum(estimate.state.contactForceXZ,2)+ ...
                    estimate.state.tipForceXZ-sum(trueContact,2)-trueTip);
                entry.planePointOffsetMm=estimate.planePointOffsetMm;
                entry.requiresReview=estimate.requiresReview;
                entry.seconds=estimate.seconds;
                entry.completed=all(isfinite([entry.contactForceRmseN entry.totalForceErrorN]));
            catch errorInfo
                entry.errorIdentifier=errorInfo.identifier;
                entry.error=errorInfo.message;
                fprintf(2,'PLANE UNCERTAINTY FAILED %s %s: %s\n', ...
                    sceneId,methodNames{methodIndex},errorInfo.message);
            end
            report.cases{caseIndex}=entry;
        end
    end
    fprintf('PLANE UNCERTAINTY %s: %d entries\n',sceneId,caseIndex);
    atomic_write_artifact(path,'json',report);
end
report.caseCount=caseIndex;
report.failureCount=sum(cellfun(@(entry)~entry.completed,report.cases));
report.state='complete';
if report.failureCount>0, report.state='complete-with-failures'; end
report.runRecord.state=report.state;
atomic_write_artifact(path,'json',report);
end
