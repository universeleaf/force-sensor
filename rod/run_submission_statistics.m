function report = run_submission_statistics(quickMode, options)
%RUN_SUBMISSION_STATISTICS Multi-seed/noise protocol with honest CI labels.
% The reported interval is a conditional local first-order interval from the
% nonlinear force sensitivity diagnostic. It is not a calibrated global
% posterior interval; coverage is therefore reported only when available.
if nargin < 1 || isempty(quickMode), quickMode = false; end
if nargin < 2, options = struct; end
if ~isfield(options,'scenes') || isempty(options.scenes)
    options.scenes={'ceiling_hook','inclined_plane','sliding_noisy'};
end
if ~isfield(options,'seeds') || isempty(options.seeds), options.seeds=[11,22,33]; end
if ~isfield(options,'noiseLevels') || isempty(options.noiseLevels), options.noiseLevels=[0,5e-5,1e-4]; end
if quickMode
    options.scenes=options.scenes(1:min(2,numel(options.scenes)));
    options.seeds=options.seeds(1:min(2,numel(options.seeds)));
    options.noiseLevels=options.noiseLevels(1:min(2,numel(options.noiseLevels)));
end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(fullfile(root,'rod'));
record=new_run_record(); folder=fullfile(root,'out','submission_statistics');
if ~isfolder(folder), mkdir(folder); end
runFolder=fullfile(folder,record.runId); mkdir(runFolder);
report=struct('state','running','runRecord',record,'cases',{{}}, ...
    'scope','Simulation-only multi-seed/multi-noise protocol; force intervals are conditional local diagnostics.', ...
    'noiseUnit','curvature per mm','coverageDefinition','abs(force error) <= 1.96*conditionalNoiseStdN', ...
    'coverageCertified',false);
publish(); caseNo=0; total=numel(options.scenes)*numel(options.seeds)*numel(options.noiseLevels);
for i=1:numel(options.scenes)
    base=contact_demo_scenes(); scene=base(strcmp({base.id},options.scenes{i}));
    assert(~isempty(scene),'rod:UnknownScene','Unknown statistics scene %s.',options.scenes{i});
    for j=1:numel(options.seeds)
        for q=1:numel(options.noiseLevels)
            caseNo=caseNo+1; timer=tic; sceneCase=scene;
            sceneCase.seed=options.seeds(j); sceneCase.curvatureNoiseStd=options.noiseLevels(q);
            if sceneCase.curvatureNoiseStd>0, sceneCase.planeNoiseStdMm=0.1; else, sceneCase.planeNoiseStdMm=0; end
            id=sprintf('%s_seed%d_noise%g',sceneCase.id,sceneCase.seed,sceneCase.curvatureNoiseStd);
            fprintf('\nSTATISTICS %d/%d: %s\n',caseNo,total,id);
            entry=struct('id',id,'scene',sceneCase.id,'seed',sceneCase.seed, ...
                'curvatureNoiseStd',sceneCase.curvatureNoiseStd,'completed',false);
            try
                [sensorInput,truth]=build_contact_demo_truth(sceneCase);
                output=estimate_sensor_forces(sensorInput); o=output.ours;
                [coverage,available]=localCoverage(o,truth);
                active=vecnorm(truth.contactForce)>0.05;
                entry.completed=true;
                entry.contactRmseN=rmse(o.contactForceResultant-truth.contactForce);
                entry.tipRmseN=rmse(o.tipForce-truth.tipForce);
                entry.totalRmseN=rmse(o.totalForceResultant-truth.totalForce);
                entry.contactLocationRmseMm=sqrt(mean((o.contactArcLength(active)-truth.contactS(active)).^2));
                entry.coverage=coverage; entry.coverageAvailableFrames=available;
                entry.reviewRate=mean(output.quality.requiresReview);
                entry.shapeConsistencyMedianMm=median(output.quality.shapeConsistencyMm);
                entry.frameSeconds=o.frameSeconds; entry.seconds=toc(timer);
                atomic_write_artifact(fullfile(runFolder,[id '.mat']),'mat', ...
                    struct('sensorInput',sensorInput,'truth',truth,'output',output,'entry',entry));
                fprintf('STAT_RESULT %s: total RMSE %.4g N; review %.1f%%; conditional coverage %.1f%% (%d frames)\n', ...
                    id,entry.totalRmseN,100*entry.reviewRate,100*coverage.rate,available);
            catch err
                entry.errorIdentifier=err.identifier; entry.error=getReport(err,'extended','hyperlinks','off');
                entry.seconds=toc(timer); fprintf(2,'STAT_FAILED %s: %s\n',id,entry.error);
            end
            report.cases{end+1}=entry; publish();
        end
    end
end
report.caseCount=numel(report.cases); report.successCount=sum(cellfun(@(x)x.completed,report.cases));
report.aggregate=aggregate(report.cases); report.state='complete';
if report.successCount<report.caseCount, report.state='complete-with-failures'; end
report.runRecord.state=report.state; publish();
fprintf('Submission statistics saved to %s\n',fullfile(folder,'comparison.json'));

    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end

function [coverage,available] = localCoverage(o,truth)
count=0; hit=0; frames=size(o.state,2);
for k=1:frames
    d=o.forceSensitivity{k};
    if ~isstruct(d) || ~isfield(d,'conditionalNoiseStdAvailable') || ~d.conditionalNoiseStdAvailable, continue; end
    stdN=d.conditionalNoiseStdN(:); err=[o.contactForceResultant(:,k)-truth.contactForce(:,k); ...
        o.tipForce(:,k)-truth.tipForce(:,k)];
    if numel(stdN)==6 && all(isfinite(stdN))
        count=count+1; hit=hit+all(abs(err)<=1.96*stdN);
    end
end
available=count; coverage=struct('rate',hit/max(count,1),'hits',hit,'availableFrames',count, ...
    'totalFrames',frames,'scope','Conditional local first-order force sensitivity; no global posterior or mode-mixture coverage.');
end

function out=aggregate(cases)
ok=cases(cellfun(@(x)x.completed,cases)); out=struct('completed',numel(ok));
if isempty(ok), out.contactRmseN=nan; out.tipRmseN=nan; out.totalRmseN=nan; out.reviewRate=nan; out.coverageRate=nan; return; end
out.contactRmseN=mean(cellfun(@(x)x.contactRmseN,ok)); out.tipRmseN=mean(cellfun(@(x)x.tipRmseN,ok));
out.totalRmseN=mean(cellfun(@(x)x.totalRmseN,ok)); out.reviewRate=mean(cellfun(@(x)x.reviewRate,ok));
available=cellfun(@(x)x.coverageAvailableFrames,ok); values=cellfun(@(x)x.coverage.rate,ok);
out.coverageRate=sum(values.*available)/max(sum(available),1); out.coverageAvailableFrames=sum(available);
end

function value=rmse(e), value=sqrt(mean(sum(e.^2,1))); end
