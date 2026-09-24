function report=run_plane_geometry_benchmark(sceneIds,sourceRunId,curvatureModelStd)
%RUN_PLANE_GEOMETRY_BENCHMARK Reuse observations; add rod admissibility only.
% Archived estimates are the baseline, never a seed. Truth is for scoring.
% Every output has a run ID and the exact archived input/results hash.
if nargin<1,sceneIds={'sliding_noisy','sliding_clean','ceiling_hook','side_wall','inclined_plane','long_soft_rod'};end
if nargin<2,sourceRunId='496dc953-eb47-499d-9e38-bd83a92dd98b';end
if nargin<3,curvatureModelStd=[];end
if ischar(sceneIds)||isstring(sceneIds),sceneIds=cellstr(sceneIds);end
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
sourceFolder=fullfile(root,'out','demos');
% Pin the baseline so regenerating demos with the new defaults cannot change
% the meaning of this paired comparison or silently relabel corrected runs.
source=jsondecode(fileread(fullfile(sourceFolder,sourceRunId,'comparison.json')));
record=new_run_record();kind='geometry';
if ~isempty(curvatureModelStd),kind='accuracy';end
folder=fullfile(root,'out',kind);
runFolder=fullfile(folder,record.runId);mkdir(runFolder);
report=struct('state','running','runRecord',record,'sourceRunId',source.runRecord.runId, ...
    'scope','Same saved sensor observations and weights; add sampled centerline nonpenetration and smooth body-contact tangency. Simulation only.', ...
    'cases',{{}});
if ~isempty(curvatureModelStd)
    report.scope=['Same archived observations: rod admissibility plus declared FBG noise/model variance. ', ...
        'Nonlinear shape seed refinement and observation-informed numerical coordinates; ', ...
        'other weights, priors and fixed-history model unchanged. Truth only for scoring. Simulation only.'];
    report.curvatureModelStdPerMm=curvatureModelStd;
end
publish();
for j=1:numel(sceneIds)
    id=sceneIds{j}; found=find(strcmp({source.cases.id},id),1);
    assert(~isempty(found)&&source.cases(found).completed,'rod:MissingDemo','Missing completed demo: %s.',id);
    sourceFile=fullfile(sourceFolder,source.cases(found).artifactFolder,'results.mat');
    saved=load(sourceFile,'sensorInput','truth','output','scene');
    assert(~isfield(saved.sensorInput.config.forceSensor,'planeContactGeometry') || ...
        strcmp(saved.sensorInput.config.forceSensor.planeContactGeometry,'point-only'), ...
        'rod:InvalidBaseline','Choose a point-only archived demo run for this ablation.');
    entry=struct('id',id,'completed',false,'sourceSha256',file_sha256(sourceFile), ...
        'sourceFile',strrep(erase(sourceFile,[root filesep]),'\','/'));
    caseFolder=fullfile(runFolder,id);mkdir(caseFolder); timer=tic;
    fprintf('\nPLANE_GEOMETRY %d/%d %s\n',j,numel(sceneIds),id);
    try
        entry.baseline=score(saved.output,saved.truth,saved.sensorInput.tube);
        sensorInput=saved.sensorInput;
        sensorInput.config.forceSensor.planeContactGeometry='rod';
        sensorInput.config.forceSensor.planeCollisionStepMm=.5;
        sensorInput.config.forceSensor.showProgress=true;
        if ~isempty(curvatureModelStd)
            sensorInput.config.forceSensor.useNonlinearShapeSeed=true;
            sensorInput.config.forceSensor.useObservationScaling=true;
            sensorInput.config=calibrate_fbg_likelihood(sensorInput.config, ...
                saved.scene.curvatureNoiseStd,curvatureModelStd);
            entry.fbgLikelihoodCalibration=sensorInput.config.forceSensor.fbgLikelihoodCalibration;
        end
        sensorInput.config.forceSensor.frameCheckpointDirectory=caseFolder;
        atomic_write_artifact(fullfile(caseFolder,'input_and_truth.mat'),'mat', ...
            struct('sensorInput',sensorInput,'truth',saved.truth,'scene',saved.scene));
        % Do not call formulation_solver_config here: all other archived
        % settings, including likelihood and history interpolation, stay fixed.
        output=estimate_sensor_forces(sensorInput);
        entry.corrected=score(output,saved.truth,sensorInput.tube);
        entry.completed=true;entry.seconds=toc(timer);
        atomic_write_artifact(fullfile(caseFolder,'results.mat'),'mat', ...
            struct('sensorInput',sensorInput,'output',output,'truth',saved.truth,'scene',saved.scene,'report',entry));
        fprintf('GEOMETRY_RESULT %s: contact %.7f -> %.7f; tip %.7f -> %.7f; total %.7f -> %.7f N\n', ...
            id,entry.baseline.contactRmseN,entry.corrected.contactRmseN, ...
            entry.baseline.tipRmseN,entry.corrected.tipRmseN,entry.baseline.totalRmseN,entry.corrected.totalRmseN);
    catch err
        entry.errorIdentifier=err.identifier;entry.error=getReport(err,'extended','hyperlinks','off');
        entry.seconds=toc(timer);fprintf(2,'%s\n',entry.error);
    end
    report.cases{j}=entry;publish();
end
report.state='complete';
if ~all(cellfun(@(e)e.completed,report.cases)),report.state='complete-with-failures';end
report.runRecord.state=report.state;publish();
fprintf('Rod-plane geometry comparison: %s\n',fullfile(runFolder,'comparison.json'));
    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end

function item=score(out,truth,tube)
o=out.ours; nt=size(o.state,2);
item=struct('contactRmseN',rmse(o.contactForceResultant-truth.contactForce), ...
    'tipRmseN',rmse(o.tipForce-truth.tipForce),'totalRmseN',rmse(o.totalForceResultant-truth.totalForce), ...
    'contactForceN',o.contactForceResultant,'tipForceN',o.tipForce, ...
    'totalForceN',o.totalForceResultant,'trueContactForceN',truth.contactForce, ...
    'trueTipForceN',truth.tipForce,'trueTotalForceN',truth.totalForce, ...
    'contactArcLengthMm',o.contactArcLength,'trueContactArcLengthMm',truth.contactS, ...
    'quality',out.quality,'frameSeconds',o.frameSeconds,'cost',o.cost, ...
    'maxConstraintResidual',max(o.activeConstraintResidual), ...
    'maxConeViolationN',max(o.frictionConeViolation), ...
    'minFrictionW',min(o.frictionWMin), ...
    'mode',{o.complementarityMode},'solverTrace',{o.solverTrace}, ...
    'denseMinimumGapMm',zeros(1,nt),'normalTangent',zeros(1,nt), ...
    'denseVerificationStepMm',.1);
% A separate tighter mechanics solve and five-times-finer grid checks the
% estimated plane, not the inaccessible true plane. No effect on estimates.
for k=1:nt
    tube.T_base=out.measurements.baseTraj(:,:,k);
    shape=solve_cosserat_force_map(tube,o.contactArcLength(k),o.contactForceResultant(:,k),o.tipForce(:,k), ...
        struct('relativeTolerance',2e-8,'momentToleranceNmm',2e-5,'collisionStepMm',.1));
    item.denseMinimumGapMm(k)=min([o.planeNormal(:,k)'*(shape.collisionP-o.state(1:3,k)), ...
        o.planeNormal(:,k)'*(shape.pc-o.state(1:3,k))]);
    item.normalTangent(k)=o.planeNormal(:,k)'*shape.contactTangent;
end
end
function v=rmse(e),v=sqrt(mean(sum(e.^2,1)));end
