function report = run_fair_baseline_protocol(quickMode, sceneIds)
%RUN_FAIR_BASELINE_PROTOCOL Compare methods on identical sensor packets.
% EnFiRCE, the shape-only point-load ablation, and the Aloi-style Gaussian
% shape fit receive the same tube, sparse FBG samples, history packet, plane
% packet, and noise realization. Baselines never receive truth or labels.
if nargin<1 || isempty(quickMode), quickMode=false; end
if nargin<2 || isempty(sceneIds), sceneIds={'ceiling_hook','inclined_plane','sliding_noisy'}; end
if ischar(sceneIds)||isstring(sceneIds), sceneIds=cellstr(sceneIds); end
if quickMode, sceneIds=sceneIds(1:min(1,numel(sceneIds))); end
root=fileparts(fileparts(mfilename('fullpath'))); demoFolder=fullfile(root,'out','demos');
sourceFile=fullfile(demoFolder,'comparison.json'); assert(isfile(sourceFile), ...
    'rod:MissingDemo','Run force(''demos'') before the fair baseline protocol.');
source=jsondecode(fileread(sourceFile)); record=new_run_record(); folder=fullfile(root,'out','fair_baselines');
if ~isfolder(folder), mkdir(folder); end; runFolder=fullfile(folder,record.runId); mkdir(runFolder);
report=struct('state','running','runRecord',record,'cases',{{}}, ...
    'scope','Identical saved sensor packets; shape-only and Gaussian baselines have no environment observations.'); publish();
for j=1:numel(sceneIds)
    id=sceneIds{j}; idx=find(strcmp({source.cases.id},id),1); assert(~isempty(idx)&&source.cases(idx).completed, ...
        'rod:MissingDemo','Completed demo %s not found.',id);
    saved=load(fullfile(demoFolder,source.cases(idx).artifactFolder,'input_and_truth.mat'),'sensorInput','truth','scene');
    fprintf('\nFAIR BASELINE %s\n',id); timer=tic; entry=struct('id',id,'completed',false,'methods',{{}});
    try
        sensorOutput=estimate_sensor_forces(saved.sensorInput); o=sensorOutput.ours;
        entry.methods{end+1}=scoreMethod('EnFiRCE',o,saved.truth,sensorOutput.quality);
        shapeOnly=estimate_shape_only_point_loads(saved.sensorInput.tube,sensorOutput.measurements,saved.sensorInput.config);
        entry.methods{end+1}=scoreMethod('shape-only point-load',shapeOnly,saved.truth,struct('requiresReview',false(1,size(shapeOnly.contactForceResultant,2))));
        cfg=saved.sensorInput.config; cfg.aloi=baselineAloiConfig();
        aloi=estimate_aloi_gaussian_baseline(saved.sensorInput.tube,sensorOutput.measurements,cfg);
        entry.methods{end+1}=scoreMethod('Aloi-style Gaussian shape fit',aloi,saved.truth,struct('requiresReview',false(1,size(aloi.forceResultant,2))));
        entry.completed=true; entry.seconds=toc(timer);
        atomic_write_artifact(fullfile(runFolder,[id '.mat']),'mat',struct('sensorInput',saved.sensorInput, ...
            'truth',saved.truth,'sensorOutput',sensorOutput,'methods',{entry.methods},'scene',saved.scene));
        for q=1:numel(entry.methods)
            fprintf('  %-30s total RMSE %.5g N; mean frame %.4g s\n',entry.methods{q}.name,entry.methods{q}.totalRmseN,entry.methods{q}.meanFrameSeconds);
        end
    catch err
        entry.errorIdentifier=err.identifier; entry.error=getReport(err,'extended','hyperlinks','off'); entry.seconds=toc(timer);
        fprintf(2,'FAIR_BASELINE_FAILED %s: %s\n',id,entry.error);
    end
    report.cases{end+1}=entry; publish();
end
report.caseCount=numel(report.cases); report.successCount=sum(cellfun(@(x)x.completed,report.cases));
report.state='complete'; if report.successCount<report.caseCount,report.state='complete-with-failures';end
report.runRecord.state=report.state; publish();
fprintf('Fair baseline report saved to %s\n',fullfile(folder,'comparison.json'));

    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end

function item=scoreMethod(name,result,truth,quality)
if isfield(result,'forceResultant'),fc=result.forceResultant; else,fc=result.contactForceResultant;end
if isfield(result,'tipForce'),fe=result.tipForce; else,fe=zeros(3,size(fc,2));end
if isfield(result,'totalForceResultant'),ft=result.totalForceResultant; else,ft=fc+fe;end
if isfield(result,'frameSeconds'),seconds=result.frameSeconds; else,seconds=nan(1,size(fc,2));end
item=struct('name',name,'contactRmseN',rmse(fc-truth.contactForce), ...
    'tipRmseN',rmse(fe-truth.tipForce),'totalRmseN',rmse(ft-truth.totalForce), ...
    'meanFrameSeconds',mean(seconds,'omitnan'),'medianFrameSeconds',median(seconds,'omitnan'), ...
    'reviewRate',mean(quality.requiresReview),'contactForceN',fc,'tipForceN',fe,'totalForceN',ft);
end

function cfg=baselineAloiConfig()
cfg.numCenterCandidates=15; cfg.sigmaCandidatesMm=[3,8,16,30]; cfg.amplitudeBoundN=100;
cfg.positionStdMm=0.2; cfg.maxEquilibriumIterations=15; cfg.equilibriumToleranceMm=1e-3;
cfg.equilibriumRelaxation=0.5; cfg.maxIterations=25; cfg.maxFunctionEvaluations=180;
cfg.numOptimizationStarts=2; cfg.showProgress=false;
end

function value=rmse(e), value=sqrt(mean(sum(e.^2,1))); end
