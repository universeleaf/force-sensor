function report = run_literature_baseline_protocol(quickMode)
%RUN_LITERATURE_BASELINE_PROTOCOL Replay verified archived sensor observations.
% Shared K is the environment estimator's data-derived candidate count; it
% is explicit hypothesis information, never read from force/location truth.
if nargin<1,quickMode=false;end
root=fileparts(fileparts(mfilename('fullpath')));sourceFolder=fullfile(root,'out','formulation_window');
sourcePath=fullfile(sourceFolder,'comparison.json');source=jsondecode(fileread(sourcePath));
assert(strcmp(source.state,'complete')&&source.failureCount==0,'rod:IncompleteReference','Reference run is incomplete.');
folder=fullfile(root,'out','benchmarks','literature');if quickMode,folder=fullfile(folder,'smoke');end
if ~isfolder(folder),mkdir(folder);end
report=struct('state','running','runRecord',new_run_record(),'quickMode',quickMode, ...
    'referenceRunId',source.runRecord.runId,'referenceComparisonSha256',file_sha256(sourcePath), ...
    'referencePath','out/formulation_window/comparison.json','cases',{{}},'sotaEstablished',false, ...
    'scope','Frozen same-input joint-window EnFiRCE reference vs disclosed independently implemented literature adaptations. Both baselines share only the data-derived count and use their own shape-only initial arcs. Historical timing is not a matched latency comparison.');
selected=1:numel(source.cases);if quickMode,selected=1;end
for i=selected
    item=source.cases(i);caseFolder=fullfile(sourceFolder,item.artifactFolder);
    fields={'input','truth','estimate'};
    for j=1:3
        path=fullfile(caseFolder,[fields{j} '.mat']);
        assert(strcmp(file_sha256(path),item.artifactSha256.(fields{j})), ...
            'rod:ReferenceIntegrity','Reference %s %s hash mismatch.',item.id,fields{j});
    end
    d=load(fullfile(caseFolder,'input.mat'),'sensorInput');input=d.sensorInput;
    ref=load(fullfile(caseFolder,'estimate.mat'),'estimate');count=size(ref.estimate.contactForce,2);
    for method={'point','gaussian'}
        name=method{1};entry=struct('sceneId',item.id,'method',name,'completed',false, ...
            'error','','candidateCount',count,'sharedCountSource','shape/environment candidate inference in reference estimate', ...
            'inputSha256',item.artifactSha256.input,'truthSha256',item.artifactSha256.truth, ...
            'referenceEstimateSha256',item.artifactSha256.estimate,'referenceMetrics',item.metrics, ...
            'metrics',[],'artifactFolder',[item.id '/' name]);
        try
            estimate=estimate_literature_curvature_baseline(input,name,count);
            % Truth is first loaded at this scoring boundary, after estimation.
            d=load(fullfile(caseFolder,'truth.mat'),'truth');entry.metrics=score_formulation_window(estimate,d.truth);
            dest=fullfile(folder,item.id,name);if ~isfolder(dest),mkdir(dest);end
            atomic_write_artifact(fullfile(dest,'estimate.mat'),'mat',struct('estimate',estimate));
            write_formulation_window_csv(estimate,fullfile(dest,'forces.csv'));
            entry.estimateSha256=file_sha256(fullfile(dest,'estimate.mat'));entry.csvSha256=file_sha256(fullfile(dest,'forces.csv'));
            entry.quality=estimate.quality;entry.solver=estimate.solver;entry.completed=true;
        catch err,entry.error=getReport(err,'extended','hyperlinks','off');fprintf('%s %s FAILED: %s\n',item.id,name,err.message);end
        report.cases{end+1}=entry;atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end
report.failureCount=sum(cellfun(@(v)~v.completed,report.cases));report.state='complete';
if report.failureCount>0,report.state='complete-with-failures';end
report.runRecord.state=report.state;atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
