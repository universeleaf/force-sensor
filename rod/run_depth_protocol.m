function protocol = run_depth_protocol(mode)
%RUN_DEPTH_PROTOCOL Synthetic depth-to-plane-to-force integration experiment.
% This is not a real camera experiment or a new independent rod truth model.
% Saved FBG fixtures are unchanged. Only environment sensing is replaced.
root=fileparts(fileparts(mfilename('fullpath'))); folder=fullfile(root,'out','depth');
if ~isfolder(folder), mkdir(folder); end
if nargin>0
    assert(strcmpi(mode,'ablation'),'Use no argument or ablation.');
    protocol=runCorrelationAblation(root,folder);
    return;
end
protocol=struct('state','running','runRecord',new_run_record());
writeReport();
d=load(fullfile(root,'out','stage1','video_noise_gated_01','sensor_input.mat'),'sensorInput');
nt=numel(d.sensorInput.packet.timeSeconds);
camera=struct('K',[120 0 48.5;0 120 36.5;0 0 1], ...
    'TWorldCamera',[eye(3),[20;0;0];0 0 0 1], ...
    'freeSpacePointMm',[0;0;0],'timeSeconds',d.sensorInput.packet.timeSeconds);
options=struct('depthStdMm',0.35,'pointFloorMm',0.13,'normalFloorRad',0.002, ...
    'anchorWorldMm',[0;0;0],'inlierThresholdMm',1.5,'seed',17);
% Analytic ray-plane rendering belongs to the sensor simulator only.
% The fitter sees depth/calibration, never the generating plane or rod forces.
[u,v]=meshgrid(1:96,1:72); rays=camera.K\[u(:)';v(:)';ones(1,numel(u))];
renderNormal=[0;0;-1]; renderPoint=[0;0;160];
rayWorld=camera.TWorldCamera(1:3,1:3)*rays;
ideal=reshape((renderNormal'*(renderPoint-camera.TWorldCamera(1:3,4)))./(renderNormal'*rayWorld),size(u));
stream=RandStream('mt19937ar','Seed',41); depthMm=zeros(72,96,nt);
for k=1:nt
    image=ideal+options.depthStdMm*randn(stream,size(ideal));
    ids=randperm(stream,numel(image)); outlier=ids(1:round(0.2*numel(image)));
    image(outlier)=image(outlier)+20+40*rand(stream,size(outlier));
    image(ids(round(0.2*numel(image))+1:round(0.3*numel(image))))=NaN;
    depthMm(:,:,k)=image;
end

atomic_write_artifact(fullfile(folder,'depth_input.mat'),'mat', ...
    struct('depthMm',depthMm,'camera',camera,'options',options));
[~,observations]=attach_depth_environment(d.sensorInput.packet,depthMm,camera,options);
points=cell2mat(cellfun(@(o)o.planePointMm,observations,'UniformOutput',false));
normals=cell2mat(cellfun(@(o)o.planeNormal,observations,'UniformOutput',false));
record=protocol.runRecord;
protocol=struct('state','running','runRecord',record, ...
    'scope','Synthetic metric depth images, one plane/trajectory, six frames, one FBG noise seed. Local covariance with explicit model floors; no hardware or full friction robustness claim.', ...
    'depthSeed',41,'fbgNoiseSeed',11,'depthStdMm',options.depthStdMm, ...
    'outlierPixelFraction',0.2,'missingPixelFraction',0.1,'frameCount',nt, ...
    'maxPlaneOffsetErrorMm',max(abs(renderNormal'*(points-renderPoint))), ...
    'maxNormalErrorDeg',max(acosd(max(-1,min(1,renderNormal'*normals)))));
protocol.unitTests=test_depth_plane();
names={'clean','noisy','fixed_weights'}; entries=cell(1,3);
for k=1:3
    input=[]; output=[]; report=struct;
    try
        caseId=1+(k>1);
        source=fullfile(root,'out','stage1',sprintf('video_noise_gated_%02d',caseId));
        d=load(fullfile(source,'sensor_input.mat'),'sensorInput'); input=d.sensorInput;
        input.config.forceSensor.historyCurvatureStdPerMm=5e-5*(caseId>1);
        input.config.forceSensor.showProgress=false;
        [input.packet,~]=attach_depth_environment(input.packet,depthMm,camera,options);
        if k==3, input.packet=rmfield(input.packet,'planeCovariance'); end
        output=estimate_sensor_forces(input);
        % Truth is loaded after estimation for scoring only.
        reference=load(fullfile(source,'inverse_results.mat'),'results'); f=reference.results.forward;
        o=output.ours;
        report=struct('contactRmseN',sqrt(mean(sum((o.contactForceResultant-f.contactForceResultant).^2,1))), ...
            'tipRmseN',sqrt(mean(sum((o.tipForce-f.tipLoad).^2,1))), ...
            'totalRmseN',sqrt(mean(sum((o.totalForceResultant-f.totalForceResultant).^2,1))), ...
            'maxShapeRmseMm',max(o.shapeRmseMm),'quality',output.quality, ...
            'modes',{o.complementarityMode},'maxActiveConstraintResidual',max(o.activeConstraintResidual), ...
            'maxRawFrictionComplementarity',max(o.frictionComplementarity), ...
            'minRawFrictionWmm',min(o.frictionWMin));
        m=input.config.forceSensor.numFrictionDirs;
        report.minConeSlackN=min(input.packet.frictionMu.*o.state(7,:)-sum(o.state(8:7+m,:),1));
        report.likelihoodCheckError=0;
        for frame=1:nt
            curvature=o.u(:,input.packet.fbgIdx,frame)-input.packet.curvaturePerMm(:,:,frame);
            e=[o.state(1:3,frame)-input.packet.planePointMm(:,frame); ...
                o.planeNormal(:,frame)-input.packet.planeNormal(:,frame)];
            if k<3
                likelihood=e'*(input.packet.planeCovariance(:,:,frame)\e);
            else
                sigma=[input.config.forceSensor.measurementStd.planePointMm(:); ...
                    input.config.forceSensor.measurementStd.normalVector(:)];
                likelihood=sum((e./sigma).^2);
            end
            expected=sqrt(sum((curvature(:)/input.config.forceSensor.measurementStd.curvature).^2)+likelihood);
            report.likelihoodCheckError=max(report.likelihoodCheckError,abs(expected-o.measurementResidualNorm(frame)));
        end
        assert(report.likelihoodCheckError<1e-7,'Environment covariance did not reach the actual MAP likelihood.');
        assert(o.environmentCovarianceUsed==(k<3),'Wrong environment weighting path.');
        assert(all(output.quality.numericalChecksPassed)&&report.minConeSlackN>-1e-7, ...
            'Depth integration violated active physical/numerical checks.');
        if k==1
            assert(report.totalRmseN<0.1&&all(o.frictionKinematicsEnforced), ...
                'Clean FBG/depth integration failed its force or friction regression.');
        else
            assert(all(output.quality.requiresReview(~o.frictionKinematicsEnforced)), ...
                'Unresolved noisy friction was incorrectly certified.');
        end
        entries{k}=struct('name',names{k},'passed',true,'error','','report',report);
    catch err
        entries{k}=struct('name',names{k},'passed',false,'error',err.message,'report',report);
    end
    report.runId=record.runId;
    atomic_write_artifact(fullfile(folder,[names{k},'.mat']),'mat', ...
        struct('input',input,'output',output,'report',report));
    entries{k}.artifact=struct('path',[names{k},'.mat'], ...
        'sha256',file_sha256(fullfile(folder,[names{k},'.mat'])));
    protocol.cases=entries(1:k); writeReport(); disp(entries{k});
end
protocol.allChecksPassed=all(cellfun(@(entry)entry.passed,entries));
protocol.state='complete';protocol.runRecord.state='complete';writeReport();
plot_depth_observations(depthMm(:,:,1),observations{1},fullfile(folder,'depth.png'));
assert(protocol.allChecksPassed,'Depth protocol has failures; see out/depth/protocol.json.');

    function writeReport()
        atomic_write_artifact(fullfile(folder,'protocol.json'),'json',protocol);
    end
end

function report = runCorrelationAblation(root,folder)
% Reuse the already validated noisy depth observations. Change only the
% off-diagonal covariance entries; retain observations and marginal variances.
ledger=jsondecode(fileread(fullfile(folder,'protocol.json')));
assert(strcmp(ledger.state,'complete')&&ledger.allChecksPassed, ...
    'rod:IncompleteDepthProtocol','Complete force(''depth'') before its ablation.');
entry=ledger.cases(strcmp({ledger.cases.name},'noisy'));
assert(isscalar(entry)&&strcmp(entry.artifact.sha256,file_sha256(fullfile(folder,'noisy.mat'))), ...
    'rod:DepthArtifactMismatch','Depth observations do not match the completed protocol.');
d=load(fullfile(folder,'noisy.mat'),'input','output');
assert(isfield(d,'input')&&isfield(d,'output')&&~isempty(d.output), ...
    'Run force(''depth'') before its correlation ablation.');
input=d.input; baseline=d.output; nt=numel(input.packet.timeSeconds);
originalCovariance=input.packet.planeCovariance;
for k=1:nt
    input.packet.planeCovariance(:,:,k)=diag(diag(originalCovariance(:,:,k)));
end
assert(isequaln(rmfield(input.packet,'planeCovariance'),rmfield(d.input.packet,'planeCovariance')) && ...
    isequaln(input.config,d.input.config)&&isequaln(input.tube,d.input.tube), ...
    'Ablation must preserve observations, timing, model and solver settings.');
report=struct('scope','One fixed six-frame noisy input. Only plane covariance off-diagonal entries removed; marginal variances, observations, model and solver settings unchanged.', ...
    'state','running','runRecord',new_run_record(), ...
    'source','out/depth/noisy.mat','sourceSha256',entry.artifact.sha256,'passed',false,'error','');
atomic_write_artifact(fullfile(folder,'ablation.json'),'json',report);
output=[];
try
    output=estimate_sensor_forces(input);
    truth=load(fullfile(root,'out','stage1','video_noise_gated_02','inverse_results.mat'),'results');
    f=truth.results.forward; report.fullCovariance=score(baseline,f);
    report.diagonalCovariance=score(output,f);
    report.maxMarginalVarianceChange=0; report.maxAbsoluteCorrelation=0;
    for k=1:nt
        C=originalCovariance(:,:,k); sd=sqrt(diag(C));
        correlation=C./(sd*sd'); correlation(1:7:end)=0;
        report.maxAbsoluteCorrelation=max(report.maxAbsoluteCorrelation,max(abs(correlation),[],'all'));
        report.maxMarginalVarianceChange=max(report.maxMarginalVarianceChange, ...
            max(abs(diag(C)-diag(input.packet.planeCovariance(:,:,k)))));
    end
    assert(report.maxMarginalVarianceChange==0,'Ablation changed marginal variances.');
    report.likelihoodCheckError=0;
    o=output.ours;
    for k=1:nt
        curvature=o.u(:,input.packet.fbgIdx,k)-input.packet.curvaturePerMm(:,:,k);
        e=[o.state(1:3,k)-input.packet.planePointMm(:,k);o.planeNormal(:,k)-input.packet.planeNormal(:,k)];
        expected=sqrt(sum((curvature(:)/input.config.forceSensor.measurementStd.curvature).^2)+ ...
            e'*(input.packet.planeCovariance(:,:,k)\e));
        report.likelihoodCheckError=max(report.likelihoodCheckError,abs(expected-o.measurementResidualNorm(k)));
    end
    m=input.config.forceSensor.numFrictionDirs;
    report.minConeSlackN=min(input.packet.frictionMu.*o.state(7,:)-sum(o.state(8:7+m,:),1));
    assert(report.likelihoodCheckError<1e-7 && o.environmentCovarianceUsed, ...
        'Diagonal covariance did not reach the MAP likelihood.');
    assert(all(output.quality.numericalChecksPassed)&&report.minConeSlackN>-1e-7, ...
        'Correlation ablation violated active physical/numerical checks.');
    assert(all(output.quality.requiresReview(~o.frictionKinematicsEnforced)), ...
        'Unresolved friction was incorrectly certified.');
    report.passed=true;
catch err
    report.error=err.message;
end
report.state='complete';report.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'diagonal.mat'),'mat',struct('input',input,'output',output,'report',report));
atomic_write_artifact(fullfile(folder,'ablation.json'),'json',report);
disp(report);
assert(report.passed,'Correlation ablation failed; see out/depth/ablation.json.');

    function metric=score(answer,f)
        o=answer.ours;
        metric=struct('contactRmseN',sqrt(mean(sum((o.contactForceResultant-f.contactForceResultant).^2,1))), ...
            'tipRmseN',sqrt(mean(sum((o.tipForce-f.tipLoad).^2,1))), ...
            'totalRmseN',sqrt(mean(sum((o.totalForceResultant-f.totalForceResultant).^2,1))), ...
            'maxShapeRmseMm',max(o.shapeRmseMm),'modes',{o.complementarityMode}, ...
            'unresolvedFrames',find(~o.frictionKinematicsEnforced), ...
            'maxRawFrictionComplementarity',max(o.frictionComplementarity), ...
            'minRawFrictionWmm',min(o.frictionWMin),'quality',answer.quality);
    end
end
