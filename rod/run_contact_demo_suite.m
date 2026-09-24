function report=run_contact_demo_suite(sceneIds)
%RUN_CONTACT_DEMO_SUITE Independent contact truth -> sensor-only estimates.
% force('demos') runs all six cases. Optional IDs select a subset. Artifacts
% are stored per invocation; a failed run cannot reuse an old success.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
scenes=contact_demo_scenes();
if nargin>0
    if ischar(sceneIds)||isstring(sceneIds),sceneIds=cellstr(sceneIds);end
    assert(all(ismember(sceneIds,{scenes.id})),'Unknown demo scene ID.');
    scenes=scenes(ismember({scenes.id},sceneIds));
end
record=new_run_record();
folder=fullfile(root,'out','demos'); if ~isfolder(folder),mkdir(folder);end
runFolder=fullfile(folder,record.runId); mkdir(runFolder);
report=struct('state','running','runRecord',record,'cases',{{}}, ...
    'scope','Independent planar continuous contact truth; nonlinear 3-D inverse; simulation only. All cases retained.', ...
    'supersedes','out/formulation/demo_suite is invalid as contact truth because rods penetrate its planes.');
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
for j=1:numel(scenes)
    scene=scenes(j); timer=tic; stage='truth';
    caseFolder=fullfile(runFolder,scene.id);mkdir(caseFolder);
    entry=struct('id',scene.id,'title',scene.title,'completed',false, ...
        'description',scene.description,'artifactFolder',strrep(fullfile(record.runId,scene.id),'\','/'));
    fprintf('\nDEMO %d/%d: %s\n',j,numel(scenes),scene.id);
    try
        [sensorInput,truth]=build_contact_demo_truth(scene);
        atomic_write_artifact(fullfile(caseFolder,'input_and_truth.mat'),'mat', ...
            struct('sensorInput',sensorInput,'truth',truth,'scene',scene));
        stage='inverse';
        output=estimate_sensor_forces(sensorInput);o=output.ours;
        stage='scoring';
        entry.completed=true;
        entry.contactRmseN=forceRmse(o.contactForceResultant-truth.contactForce);
        entry.tipRmseN=forceRmse(o.tipForce-truth.tipForce);
        entry.totalRmseN=forceRmse(o.totalForceResultant-truth.totalForce);
        entry.contactMagnitudeRmseN=sqrt(mean((vecnorm(o.contactForceResultant)-vecnorm(truth.contactForce)).^2));
        active=vecnorm(truth.contactForce)>0.05;
        entry.contactLocationRmseMm=sqrt(mean((o.contactArcLength(active)-truth.contactS(active)).^2));
        delta=o.p-truth.p;
        entry.trueShapeRmseMm=sqrt(mean(sum(delta.^2,1),'all'));
        entry.observationShapeRmseMm=o.shapeRmseMm;
        entry.quality=output.quality;
        entry.frameSeconds=o.frameSeconds;
        entry.truthMaxPenetrationMm=max(cellfun(@(e)e.maxPenetrationMm,truth.equilibria),[],'all');
        n=truth.planeNormal;
        entry.estimatedMaxPenetrationMm=max(0,-min(reshape(n'*reshape(o.p-truth.planePoint,3,[]),[],1)));
        entry.forceMagnitudeTruthN=vecnorm(truth.contactForce);
        entry.forceMagnitudeEstimateN=vecnorm(o.contactForceResultant);
        entry.frames=size(o.state,2);
        entry.seconds=toc(timer);
        data=portableData(scene,truth,output,entry);
        atomic_write_artifact(fullfile(caseFolder,'data.json'),'json',data);
        atomic_write_artifact(fullfile(caseFolder,'results.mat'),'mat', ...
            struct('sensorInput',sensorInput,'truth',truth,'output',output,'scene',scene,'report',entry));
        writeCsv(caseFolder,data);
        fprintf('DEMO_RESULT %s: contact/tip/total RMSE %.6f / %.6f / %.6f N; truth penetration %.3g mm\n', ...
            scene.id,entry.contactRmseN,entry.tipRmseN,entry.totalRmseN,entry.truthMaxPenetrationMm);
    catch err
        entry.completed=false;entry.failureStage=stage;
        entry.errorIdentifier=err.identifier;entry.error=err.message;entry.seconds=toc(timer);
        fprintf(2,'DEMO_FAILED %s (%s): %s\n',scene.id,stage,getReport(err,'extended','hyperlinks','off'));
    end
    report.cases{j}=entry;
    atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.successCount=sum(cellfun(@(x)x.completed,report.cases));
report.caseCount=numel(scenes);
report.state='complete';
if report.successCount<report.caseCount,report.state='complete-with-failures';end
report.runRecord.state=report.state;
atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
fprintf('\nDemo results saved to %s\n',folder);
fprintf('Build the offline viewer: python scripts/render_contact_demos.py\n');
end

function v=forceRmse(e),v=sqrt(mean(sum(e.^2,1)));end

function data=portableData(scene,truth,out,entry)
o=out.ours;nt=size(o.state,2);
data=struct('scene',scene,'summary',entry,'frames',{{}}, ...
    'planePoint',truth.planePoint,'planeNormal',truth.planeNormal, ...
    'lengthMm',truth.sMm(end),'forceUnits','N','positionUnits','mm');
for k=1:nt
    e=truth.equilibria{2,k};
    data.frames{k}=struct('index',k,'pushMm',scene.pushMm(k), ...
        'sensorTimeSeconds',out.measurements.timeSeconds(k), ...
        'pTrue',truth.p(:,:,k)','pEstimated',o.p(:,:,k)', ...
        'pObserved',out.measurements.p(:,:,k)', ...
        'basePosition',truth.basePose(1:3,4,k), ...
        'contactTrue',truth.contactPoint(:,k),'contactEstimated',o.contactPoint(:,k), ...
        'contactArcTrue',truth.contactS(k),'contactArcEstimated',o.contactArcLength(k), ...
        'fcTrue',truth.contactForce(:,k),'fcEstimated',o.contactForceResultant(:,k), ...
        'feTrue',truth.tipForce(:,k),'feEstimated',o.tipForce(:,k), ...
        'totalTrue',truth.totalForce(:,k),'totalEstimated',o.totalForceResultant(:,k), ...
        'contactErrorN',norm(o.contactForceResultant(:,k)-truth.contactForce(:,k)), ...
        'tipErrorN',norm(o.tipForce(:,k)-truth.tipForce(:,k)), ...
        'totalErrorN',norm(o.totalForceResultant(:,k)-truth.totalForce(:,k)), ...
        'mode',o.complementarityMode{k},'requiresReview',out.quality.requiresReview(k), ...
        'complementarityResidual',out.quality.complementarityResidual(k), ...
        'truthPenetrationMm',e.maxPenetrationMm, ...
        'tipMomentResidualNmm',e.stationarityInfNmm);
end
end

function writeCsv(folder,data)
f=data.frames; nt=numel(f);
values=zeros(nt,30);
for k=1:nt
    a=f{k};
    values(k,:)=[k,a.pushMm,a.fcTrue(:)',a.fcEstimated(:)',a.feTrue(:)', ...
        a.feEstimated(:)',a.totalTrue(:)',a.totalEstimated(:)', ...
        norm(a.fcTrue),norm(a.fcEstimated),a.contactErrorN,a.tipErrorN,a.totalErrorN, ...
        a.contactArcTrue,a.contactArcEstimated,a.complementarityResidual, ...
        double(a.requiresReview),a.truthPenetrationMm];
end
names={'frame','push_mm','true_contact_x_N','true_contact_y_N','true_contact_z_N', ...
    'estimated_contact_x_N','estimated_contact_y_N','estimated_contact_z_N', ...
    'true_tip_x_N','true_tip_y_N','true_tip_z_N','estimated_tip_x_N','estimated_tip_y_N','estimated_tip_z_N', ...
    'true_total_x_N','true_total_y_N','true_total_z_N','estimated_total_x_N','estimated_total_y_N','estimated_total_z_N', ...
    'true_contact_magnitude_N','estimated_contact_magnitude_N','contact_vector_error_N','tip_vector_error_N','total_vector_error_N', ...
    'true_contact_s_mm','estimated_contact_s_mm','complementarity_residual','requires_review','truth_penetration_mm'};
writetable(array2table(values,'VariableNames',names),fullfile(folder,'forces.csv'));
end
