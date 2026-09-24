function report = run_realtime_benchmark(quickMode, sceneId)
%RUN_REALTIME_BENCHMARK Replay identical packets and report wall-clock cost.
% A frame meets the nominal sensor period only when the measured p95 replay
% time is below that period. This is a software benchmark on the current
% machine, not a hardware real-time guarantee.
if nargin<1||isempty(quickMode),quickMode=false;end
if nargin<2||isempty(sceneId),sceneId='sliding_clean';end
root=fileparts(fileparts(mfilename('fullpath'))); demoFolder=fullfile(root,'out','demos');
source=jsondecode(fileread(fullfile(demoFolder,'comparison.json'))); idx=find(strcmp({source.cases.id},sceneId),1);
assert(~isempty(idx)&&source.cases(idx).completed,'rod:MissingDemo','Completed demo %s not found.',sceneId);
saved=load(fullfile(demoFolder,source.cases(idx).artifactFolder,'input_and_truth.mat'),'sensorInput','scene');
nt=numel(saved.sensorInput.packet.timeSeconds); indices=1:nt;
if quickMode, indices=1:min(2,nt); end
record=new_run_record();folder=fullfile(root,'out','realtime');if ~isfolder(folder),mkdir(folder);end
runFolder=fullfile(folder,record.runId);mkdir(runFolder); report=struct('state','running','runRecord',record, ...
    'scene',sceneId,'sensorPeriodSeconds',saved.sensorInput.packet.processReferencePeriodSeconds, ...
    'scope','Wall-clock MATLAB replay of the formulation-aligned estimator; no hardware scheduling claim.','frames',{{}});publish();
for j=1:numel(indices)
    k=indices(j); packet=subset_sensor_packet(saved.sensorInput.packet,k);
    input=saved.sensorInput;input.packet=packet;timer=tic;
    try
        output=estimate_sensor_forces(input);elapsed=toc(timer);o=output.ours;
        frame=struct('index',k,'elapsedSeconds',elapsed,'reportedFrameSeconds',o.frameSeconds(1), ...
            'solverRhsEvaluations',nan,'requiresReview',output.quality.requiresReview(1), ...
            'shapeConsistencyMm',output.quality.shapeConsistencyMm(1));
        if ~isempty(o.mechanicsDiagnostics{1}) && isfield(o.mechanicsDiagnostics{1},'model'),frame.mechanicsModel=o.mechanicsDiagnostics{1}.model;end
        report.frames{end+1}=frame;fprintf('REALTIME frame %d: %.4g s (reported inverse %.4g s)\n',k,elapsed,o.frameSeconds(1));
    catch err
        report.frames{end+1}=struct('index',k,'completed',false,'errorIdentifier',err.identifier,'error',getReport(err,'extended','hyperlinks','off'));
        fprintf(2,'REALTIME_FAILED frame %d: %s\n',k,report.frames{end}.error);
    end
    publish();
end
ok=report.frames(cellfun(@(x)isfield(x,'elapsedSeconds'),report.frames));
times=cellfun(@(x)x.elapsedSeconds,ok);p95=percentile(times,95);period=report.sensorPeriodSeconds;
report.completedFrames=numel(ok);report.summary=struct('meanSeconds',mean(times,'omitnan'), ...
    'medianSeconds',median(times,'omitnan'),'p95Seconds',p95,'maxSeconds',max(times,[],'omitnan'), ...
    'effectiveHz',1/max(p95,eps),'meetsNominalPeriod',p95<=period,'periodSeconds',period);
report.state='complete';if isempty(ok),report.state='complete-with-failures';end;report.runRecord.state=report.state;publish();
fprintf('Realtime report saved to %s\n',fullfile(folder,'comparison.json'));
    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end
function q=percentile(x,p)
x=sort(x(:));if isempty(x),q=nan;return;end
t=1+(numel(x)-1)*p/100;lo=floor(t);hi=ceil(t);if lo==hi,q=x(lo);else,q=x(lo)+(t-lo)*(x(hi)-x(lo));end
end
