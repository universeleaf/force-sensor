function report = run_history_map_benchmark(sceneIds)
%RUN_HISTORY_MAP_BENCHMARK Same packets, fixed vs latent predecessor MAP.
% Truth is loaded only for scoring. Each method receives tube/packet/config.
% No re-seeding/tuning by force labels; both use the saved solver budgets.
if nargin<1,sceneIds={'sliding_noisy'};end
if ischar(sceneIds)||isstring(sceneIds),sceneIds=cellstr(sceneIds);end
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
demoFolder=fullfile(root,'out','demos');
source=jsondecode(fileread(fullfile(demoFolder,'comparison.json')));
record=new_run_record();folder=fullfile(root,'out','history_map');
runFolder=fullfile(folder,record.runId);mkdir(runFolder);
report=struct('state','running','runRecord',record,'cases',{{}}, ...
    'historyPointInterpolation','integrated', ...
    'sourceDemoRunId',source.runRecord.runId, ...
    'scope','Paired latent sparse-FBG MAP vs fixed history on identical saved observations; simulation only.');
publish();
for j=1:numel(sceneIds)
    id=sceneIds{j};found=find(strcmp({source.cases.id},id),1);
    assert(~isempty(found)&&source.cases(found).completed,'rod:MissingDemo','Completed demo not found: %s.',id);
    inputFile=fullfile(demoFolder,source.cases(found).artifactFolder,'input_and_truth.mat');
    saved=load(inputFile,'sensorInput','truth','scene');
    entry=struct('id',id,'completed',false,'sourceInputSha256',file_sha256(inputFile),'methods',{{}});
    caseFolder=fullfile(runFolder,id);mkdir(caseFolder);
    atomic_write_artifact(fullfile(caseFolder,'input_and_truth.mat'),'mat',saved);
    for method={'fixed','latent-fbg'}
        name=method{1}; timer=tic;
        item=struct('name',name,'completed',false);
        fprintf('\nHISTORY_MAP %s / %s\n',id,name);
        try
            sensorInput=saved.sensorInput;
            sensorInput.config.forceSensor.historyStateMode=name;
            sensorInput.config.forceSensor.historyPointInterpolation='integrated';
            output=estimate_sensor_forces(sensorInput);
            item=score(name,output,saved.truth);item.seconds=toc(timer);
            atomic_write_artifact(fullfile(caseFolder,[name '.mat']),'mat', ...
                struct('sensorInput',sensorInput,'output',output,'metrics',item));
            fprintf('HISTORY_RESULT %s / %s: contact %.6f, tip %.6f, total %.6f N\n', ...
                id,name,item.contactRmseN,item.tipRmseN,item.totalRmseN);
        catch err
            item.errorIdentifier=err.identifier;item.error=getReport(err,'extended','hyperlinks','off');
            item.seconds=toc(timer);fprintf(2,'%s\n',item.error);
        end
        entry.methods{end+1}=item;report.cases{j}=entry;publish();
    end
    entry.completed=all(cellfun(@(v)v.completed,entry.methods));
    if entry.completed
        old=entry.methods{1};new=entry.methods{2};
        entry.contactRmseChangeN=new.contactRmseN-old.contactRmseN;
        entry.contactRmseReductionPct=100*(old.contactRmseN-new.contactRmseN)/max(old.contactRmseN,eps);
        entry.contactForceMaxDifferenceN=max(vecnorm(new.contactForceN-old.contactForceN));
    end
    report.cases{j}=entry;publish();
end
report.state='complete';
if ~all(cellfun(@(v)v.completed,report.cases)),report.state='complete-with-failures';end
report.runRecord.state=report.state;publish();
fprintf('History MAP comparison: %s\n',fullfile(runFolder,'comparison.json'));
    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end

function item=score(name,out,truth)
o=out.ours;
item=struct('name',name,'completed',true, ...
    'contactRmseN',rmse(o.contactForceResultant-truth.contactForce), ...
    'tipRmseN',rmse(o.tipForce-truth.tipForce), ...
    'totalRmseN',rmse(o.totalForceResultant-truth.totalForce), ...
    'contactMagnitudeRmseN',sqrt(mean((vecnorm(o.contactForceResultant)-vecnorm(truth.contactForce)).^2)), ...
    'contactForceN',o.contactForceResultant,'tipForceN',o.tipForce, ...
    'totalForceN',o.totalForceResultant,'trueContactForceN',truth.contactForce, ...
    'trueTipForceN',truth.tipForce,'trueTotalForceN',truth.totalForce, ...
    'contactMagnitudeN',vecnorm(o.contactForceResultant),'trueContactMagnitudeN',vecnorm(truth.contactForce), ...
    'contactVectorErrorN',vecnorm(o.contactForceResultant-truth.contactForce), ...
    'frameSeconds',o.frameSeconds,'cost',o.cost,'quality',out.quality, ...
    'maxConstraintResidual',max(o.activeConstraintResidual), ...
    'shapeRmseToObservationsMm',o.shapeRmseMm, ...
    'trueShapeRmseMm',sqrt(mean(sum((o.p-truth.p).^2,1),'all')), ...
    'contactArcLengthMm',o.contactArcLength,'trueContactArcLengthMm',truth.contactS, ...
    'mode',{o.complementarityMode},'history',{{}});
for k=1:numel(o.historyEstimate)
    if isempty(o.historyEstimate{k}),item.history{k}=struct('latentDimension',0);continue;end
    h=o.historyEstimate{k};h=rmfield(h,'shape');item.history{k}=h;
end
item.solverTrace=o.solverTrace;
end
function value=rmse(e),value=sqrt(mean(sum(e.^2,1)));end
