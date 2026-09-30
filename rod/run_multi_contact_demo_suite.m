function report = run_multi_contact_demo_suite()
%RUN_MULTI_CONTACT_DEMO_SUITE Full truth -> sparse observation -> inverse -> video.
% The prototype assumes planar frictionless contacts of known count/order.
% Stable paths: out/demos/multi_contact/<scene-id>/; prior runs in history/.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','demos','multi_contact');
if ~isfolder(folder), mkdir(folder); end
scenes=multi_contact_demo_scenes(); record=new_run_record();
archiveFolder=fullfile(folder,'history',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
report=struct('schemaVersion',2,'state','running','runRecord',record, ...
    'scope','Planar frictionless known-contact-order sparse-curvature inverse; model-consistent numerical demonstration.', ...
    'caseCount',numel(scenes),'successCount',0,'cases',{{}});
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
for j=1:numel(scenes)
    scene=scenes(j); caseFolder=fullfile(folder,scene.id);
    assert(~isempty(regexp(scene.id,'^[a-z0-9_]+$','once')),'Invalid scene ID.');
    if isfolder(caseFolder)
        if ~isfolder(archiveFolder), mkdir(archiveFolder); end
        movefile(caseFolder,fullfile(archiveFolder,scene.id));
    end
    mkdir(caseFolder);
    entry=struct('id',scene.id,'title',scene.title,'completed',false, ...
        'contactCount',numel(scene.contactPlaneIndex), ...
        'artifactFolder',['multi_contact/' scene.id], ...
        'scope',report.scope);
    stage='truth'; timer=tic;
    fprintf('\nMULTI-CONTACT DEMO %d/%d: %s\n',j,numel(scenes),scene.id);
    try
        [truth,scene,sensorInput]=build_multi_contact_demo_truth(scene,scene.seed);
        atomic_write_artifact(fullfile(caseFolder,'input.mat'),'mat',struct('sensorInput',sensorInput));
        stage='inverse';
        estimate=runInverse(sensorInput);
        entry.stateCount=numel(sensorInput.frames);
        entry.contactForceRmseN=rmse3(estimate.contactForces-truth.contactForces);
        entry.contactForceRmseByContactN=sqrt(mean(sum((estimate.contactForces-truth.contactForces).^2,1),3));
        entry.contactArcLengthRmseMm=sqrt(mean((estimate.contactS(:)-truth.contactS(:)).^2));
        entry.tipForceRmseN=rmse3(estimate.tipForce-truth.tipForce);
        entry.totalForceRmseN=rmse3(estimate.totalForce-truth.totalForce);
        entry.shapeRmseMm=rmse3(estimate.p-truth.p);
        entry.reviewCount=sum(estimate.requiresReview);
        entry.truthMaxAbsGapMm=max(abs(truth.contactGapMm(:)));
        entry.truthMaxPenetrationMm=max(truth.maxPenetrationMm);
        entry.truthMaxTipMomentResidualNmm=max(truth.tipMomentResidualNmm);
        entry.estimateMaxAbsGapMm=max(cellfun(@(a)a.maxAbsGapMm,estimate.audit));
        entry.estimateMaxPenetrationMm=max(cellfun(@(a)a.maxPenetrationMm,estimate.audit));
        entry.estimateMaxTipMomentResidualNmm=max(cellfun(@(a)a.tipMomentResidualNmm,estimate.audit));
        entry.forceMagnitudeTruthN=squeeze(vecnorm(truth.contactForces,2,1));
        entry.forceMagnitudeEstimateN=squeeze(vecnorm(estimate.contactForces,2,1));
        entry.frameSeconds=estimate.seconds;
        entry.wallClockSeconds=toc(timer);
        entry.inputSha256=file_sha256(fullfile(caseFolder,'input.mat'));
        payload=struct('truth',truth,'estimate',estimate,'sensorInput',sensorInput,'scene',scene,'entry',entry,'runRecord',record);
        atomic_write_artifact(fullfile(caseFolder,'results.mat'),'mat',payload);
        stage='video';
        entry.video=render_multi_contact_demo_video(payload,fullfile(caseFolder,'forces.mp4'));
        entry.video.videoPath='forces.mp4';
        entry.video.sha256=file_sha256(fullfile(caseFolder,'forces.mp4'));
        stage='export';
        writeCsv(fullfile(caseFolder,'forces.csv'),truth,estimate,scene);
        entry.completed=true;
        payload.entry=entry;
        atomic_write_artifact(fullfile(caseFolder,'results.mat'),'mat',payload);
        entry.resultsSha256=file_sha256(fullfile(caseFolder,'results.mat'));
        writeJson(fullfile(caseFolder,'data.json'),truth,estimate,scene,entry);
        report.successCount=report.successCount+1;
        fprintf('RESULT %s contact/tip/total RMSE %.4g / %.4g / %.4g N, review %d/%d\n', ...
            scene.id,entry.contactForceRmseN,entry.tipForceRmseN,entry.totalForceRmseN,entry.reviewCount,entry.stateCount);
    catch err
        entry.completed=false; entry.error=err.message; entry.failureIdentifier=err.identifier; entry.failureStage=stage;
        fprintf(2,'FAILED %s [%s]: %s\n',scene.id,stage,getReport(err,'extended','hyperlinks','off'));
    end
    report.cases{j}=entry;
    atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.state='complete';
if report.successCount<report.caseCount, report.state='complete-with-failures'; end
report.runRecord.state=report.state;
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
fprintf('Saved %d/%d cases to %s\n',report.successCount,report.caseCount,folder);
end

function estimate=runInverse(sensorInput)
nt=numel(sensorInput.frames); model=sensorInput.frames{1}.model;
s=model.sMm; ns=numel(s); K=numel(model.contactPlaneIndex);
estimate=struct('p',zeros(3,ns,nt),'contactPoints',zeros(3,K,nt), ...
    'contactForces',zeros(3,K,nt),'contactS',zeros(K,nt), ...
    'tipForce',zeros(3,nt),'totalForce',zeros(3,nt), ...
    'requiresReview',false(1,nt),'seconds',zeros(1,nt), ...
    'diagnostics',{{}},'audit',{{}});
for k=1:nt
    e=estimate_planar_multi_contact(sensorInput.frames{k}); q=e.state;
    estimate.p([1 3],:,k)=interp1(q.sMm,q.pXZ',s)';
    estimate.contactPoints([1 3],:,k)=q.contactPointsXZ;
    estimate.contactForces([1 3],:,k)=q.contactForceXZ;
    estimate.contactS(:,k)=q.contactS(:); estimate.tipForce([1 3],k)=q.tipForceXZ;
    estimate.totalForce(:,k)=sum(estimate.contactForces(:,:,k),2)+estimate.tipForce(:,k);
    estimate.requiresReview(k)=e.requiresReview; estimate.seconds(k)=e.seconds;
    estimate.audit{k}=e.audit; estimate.diagnostics{k}=rmfield(e,'state');
end
estimate.scope='Planar frictionless known-contact-order sparse-curvature inverse; not full 3-D frictional MAP.';
end

function value=rmse3(x), value=sqrt(mean(sum(x.^2,1),'all')); end

function writeCsv(path,truth,estimate,scene)
K=size(truth.contactForces,2); nt=size(truth.p,3);
values=[(1:nt)' scene.baseMotionMm(:)]; names={'state','base_x_mm'};
for j=1:K
    trueF=reshape(truth.contactForces(:,j,:),3,nt)';
    estF=reshape(estimate.contactForces(:,j,:),3,nt)';
    values=[values trueF estF vecnorm(trueF,2,2) vecnorm(estF,2,2) truth.contactS(j,:)' estimate.contactS(j,:)']; %#ok<AGROW>
    suffix={'true_fx_N','true_fy_N','true_fz_N','est_fx_N','est_fy_N','est_fz_N','true_magnitude_N','est_magnitude_N','true_s_mm','est_s_mm'};
    names=[names cellfun(@(s)sprintf('contact%d_%s',j,s),suffix,'UniformOutput',false)]; %#ok<AGROW>
end
values=[values truth.tipForce' estimate.tipForce' truth.totalForce' estimate.totalForce' estimate.requiresReview'];
names=[names {'tip_true_fx_N','tip_true_fy_N','tip_true_fz_N','tip_est_fx_N','tip_est_fy_N','tip_est_fz_N', ...
    'total_true_fx_N','total_true_fy_N','total_true_fz_N','total_est_fx_N','total_est_fy_N','total_est_fz_N','requires_review'}];
writetable(array2table(values,'VariableNames',names),path);
end

function writeJson(path,truth,estimate,scene,entry)
nt=size(truth.p,3); frames=cell(1,nt);
for k=1:nt
    frames{k}=struct('state',k,'baseMotionMm',scene.baseMotionMm(k), ...
        'contactArcLengthMm',truth.contactS(:,k)','estimatedContactArcLengthMm',estimate.contactS(:,k)', ...
        'contactPointsMm',truth.contactPoints(:,:,k),'estimatedContactPointsMm',estimate.contactPoints(:,:,k), ...
        'contactForceN',truth.contactForces(:,:,k),'estimatedContactForceN',estimate.contactForces(:,:,k), ...
        'tipForceN',truth.tipForce(:,k),'estimatedTipForceN',estimate.tipForce(:,k), ...
        'totalForceN',truth.totalForce(:,k),'estimatedTotalForceN',estimate.totalForce(:,k), ...
        'trueShapeMm',truth.p(:,:,k)','estimatedShapeMm',estimate.p(:,:,k)', ...
        'truthAudit',truth.audit{k},'estimateAudit',estimate.audit{k},'requiresReview',estimate.requiresReview(k));
end
data=struct('scene',scene,'entry',entry,'frames',{frames},'contactCount',scene.contactCount,'scope',estimate.scope);
atomic_write_artifact(path,'json',data);
end
