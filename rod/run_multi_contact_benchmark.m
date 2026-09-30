function report = run_multi_contact_benchmark(quickMode)
%RUN_MULTI_CONTACT_BENCHMARK Paired synthetic tests, ablations and timing.
% All methods receive the same curvature packet. The shape-only baseline
% additionally receives the contact count/order, but no plane geometry.
if nargin<1, quickMode=false; end
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','benchmarks','multi_contact');
if quickMode, folder=fullfile(folder,'smoke'); end
if ~isfolder(folder), mkdir(folder); end
scenes=multi_contact_demo_scenes(); scenes=scenes(1:3);
seeds=[11 22 33]; noiseLevels=[0 2.5e-5 5e-5]; sensorCounts=[8 16 24];
frameIndices=[3 10];
if quickMode
    scenes=scenes(1); seeds=seeds(1); noiseLevels=noiseLevels(2);
    sensorCounts=sensorCounts(end); frameIndices=frameIndices(1);
end
record=new_run_record();
report=struct('schemaVersion',1,'state','running','runRecord',record, ...
    'scope',['Independent planar frictionless forward equilibrium; known ordered contact count; ', ...
    'paired sparse-curvature observations. Literature-inspired geometry only; no original-paper method reproduction.'], ...
    'sceneIds',{ {scenes.id} },'seeds',seeds,'noiseStdPerMm',noiseLevels, ...
    'sensorCounts',sensorCounts,'frameIndices',frameIndices, ...
    'methods',{{'EnFiRCE_environment','shape_only_point_loads', ...
        'ablation_no_gap','ablation_no_tangency','ablation_no_geometry_penalties', ...
        'ablation_plane_offset_1mm'}},'cases',{{}});
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
caseIndex=0;
for sceneIndex=1:numel(scenes)
    scene=scenes(sceneIndex); scene.curvatureNoiseStd=0;
    fprintf('\nBENCHMARK TRUTH %s\n',scene.id);
    [truth,scene,sensorInput]=build_multi_contact_demo_truth(scene,scene.seed);
    sceneFolder=fullfile(folder,scene.id);
    if ~isfolder(sceneFolder), mkdir(sceneFolder); end
    atomic_write_artifact(fullfile(sceneFolder,'truth.mat'),'mat', ...
        struct('truth',truth,'scene',scene,'sensorInput',sensorInput));
    raw=struct('sceneId',{},'seed',{},'noiseStdPerMm',{},'sensorCount',{}, ...
        'frameIndex',{},'input',{},'results',{});
    rawIndex=0;
    for seed=seeds
        for frameIndex=frameIndices
            stream=RandStream('mt19937ar','Seed',10000*sceneIndex+100*seed+frameIndex);
            standardNoise=randn(stream,1,numel(truth.sMm));
            for noiseStd=noiseLevels
                for sensorCount=sensorCounts
                    sensorIndices=unique(round(linspace(2,numel(truth.sMm)-1,sensorCount)));
                    input=sensorInput.frames{frameIndex};
                    input.fbgArcLengthMm=truth.sMm(sensorIndices);
                    input.curvaturePerMm=truth.u(2,sensorIndices,frameIndex)+ ...
                        noiseStd*standardNoise(sensorIndices);
                    input.curvatureStdPerMm=max(noiseStd,1e-7);
                    rawIndex=rawIndex+1;
                    raw(rawIndex).sceneId=scene.id;
                    raw(rawIndex).seed=seed;
                    raw(rawIndex).noiseStdPerMm=noiseStd;
                    raw(rawIndex).sensorCount=sensorCount;
                    raw(rawIndex).frameIndex=frameIndex;
                    raw(rawIndex).input=input;
                    raw(rawIndex).results=struct();
                    runMethod('EnFiRCE_environment',@()estimate_planar_multi_contact(input));
                    runMethod('shape_only_point_loads',@()estimate_planar_shape_only_point_loads(input));
                    if sensorCount==24 && noiseStd==2.5e-5
                        runMethod('ablation_no_gap',@()estimate_planar_multi_contact(input, ...
                            struct('gapWeight',0)));
                        runMethod('ablation_no_tangency',@()estimate_planar_multi_contact(input, ...
                            struct('tangentWeight',0)));
                        runMethod('ablation_no_geometry_penalties',@()estimate_planar_multi_contact(input, ...
                            struct('gapWeight',0,'tangentWeight',0)));
                        shifted=input;
                        plane=shifted.model.contactPlaneIndex(1);
                        shifted.model.planePointXZ(:,plane)=shifted.model.planePointXZ(:,plane)+ ...
                            shifted.model.planeNormalXZ(:,plane)/norm(shifted.model.planeNormalXZ(:,plane));
                        runMethod('ablation_plane_offset_1mm',@()estimate_planar_multi_contact(shifted));
                    end
                end
            end
        end
        fprintf('BENCHMARK %s seed %d complete: %d entries\n',scene.id,seed,caseIndex);
    end
    atomic_write_artifact(fullfile(sceneFolder,'trials.mat'),'mat',struct('raw',raw));
    report.sceneArtifacts.(scene.id)=struct('truth',file_sha256(fullfile(sceneFolder,'truth.mat')), ...
        'trials',file_sha256(fullfile(sceneFolder,'trials.mat')));
    atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.caseCount=caseIndex;
report.failureCount=sum(cellfun(@(entry)~entry.completed,report.cases));
report.state='complete';
if report.failureCount>0, report.state='complete-with-failures'; end
report.runRecord.state=report.state;
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
fprintf('BENCHMARK %d estimates, %d failures\n',caseIndex,report.failureCount);

    function runMethod(method,solver)
        caseIndex=caseIndex+1;
        entry=struct('sceneId',scene.id,'seed',seed,'frameIndex',frameIndex, ...
            'baseMotionMm',scene.baseMotionMm(frameIndex), ...
            'noiseStdPerMm',noiseStd,'sensorCount',sensorCount, ...
            'contactCount',numel(scene.contactPlaneIndex),'method',method, ...
            'completed',false);
        try
            estimate=solver();
            if isfield(estimate,'state')
                force=estimate.state.contactForceXZ;
                arc=estimate.state.contactS(:);
                tip=estimate.state.tipForceXZ(:);
                entry.shapeRmseMm=sqrt(mean(sum((interp1(estimate.state.sMm, ...
                    estimate.state.pXZ',truth.sMm)'-truth.p([1 3],:,frameIndex)).^2,1)));
                compact=rmfield(estimate,'state');
            else
                force=estimate.contactForceXZ;
                arc=estimate.contactS(:);
                tip=estimate.tipForceXZ(:);
                entry.shapeRmseMm=NaN;
                compact=estimate;
            end
            trueForce=truth.contactForces([1 3],:,frameIndex);
            trueTip=truth.tipForce([1 3],frameIndex);
            delta=force-trueForce;
            entry.contactForceRmseN=sqrt(mean(sum(delta.^2,1)));
            entry.contactForceMaeN=mean(vecnorm(delta));
            entry.contactMagnitudeMaeN=mean(abs(vecnorm(force)-vecnorm(trueForce)));
            entry.contactArcRmseMm=sqrt(mean((arc-truth.contactS(:,frameIndex)).^2));
            entry.tipForceErrorN=norm(tip-trueTip);
            entry.totalForceErrorN=norm(sum(force,2)+tip-sum(trueForce,2)-trueTip);
            entry.seconds=estimate.seconds;
            entry.requiresReview=estimate.requiresReview;
            entry.exitflag=estimate.exitflag;
            entry.completed=all(isfinite([entry.contactForceRmseN entry.contactArcRmseMm ...
                entry.tipForceErrorN entry.totalForceErrorN entry.seconds]));
            raw(rawIndex).results.(method)=struct('contactS',arc,'contactForceXZ',force, ...
                'tipForceXZ',tip,'diagnostics',compact);
        catch errorInfo
            entry.errorIdentifier=errorInfo.identifier;
            entry.error=errorInfo.message;
            fprintf(2,'BENCHMARK FAILED %s %s seed %d state %d: %s\n', ...
                scene.id,method,seed,frameIndex,errorInfo.message);
        end
        report.cases{caseIndex}=entry;
    end
end
