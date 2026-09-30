function report = run_model_mismatch_protocol(quickMode, kinds)
%RUN_MODEL_MISMATCH_PROTOCOL Stress-test the declared single-plane model.
% Independent contact-closure truth, with an omitted environment face or a
% wrong friction coefficient. Reactions are not arbitrary imposed loads.
if nargin<1||isempty(quickMode),quickMode=false;end
if nargin<2||isempty(kinds),kinds={'two-contact','curved-surface','friction-mismatch'};end
if ischar(kinds)||isstring(kinds),kinds=cellstr(kinds);end
if quickMode,kinds=kinds(1:min(1,numel(kinds)));end
root=fileparts(fileparts(mfilename('fullpath'))); record=new_run_record(); folder=fullfile(root,'out','model_mismatch');
if ~isfolder(folder),mkdir(folder);end; runFolder=fullfile(folder,record.runId);mkdir(runFolder);
report=struct('state','running','runRecord',record,'cases',{{}}, ...
    'scope','Independent fixed-surface contact truth; inverse loses a face or uses wrong friction; not nominal accuracy.');publish();
for j=1:numel(kinds)
    kind=kinds{j}; timer=tic; entry=struct('kind',kind,'completed',false); fprintf('\nMISMATCH %s\n',kind);
    try
        [sensorInput,truth]=build_multi_contact_truth(kind,401+j);
        output=estimate_formulation_forces(sensorInput);
        entry.completed=true; entry.contactRmseN=rmse(output.contactForceResultant-truth.contactForce);
        entry.totalRmseN=rmse(output.totalForceResultant-truth.totalForce);
        entry.tipRmseN=rmse(output.tipForce-truth.tipForce); entry.reviewRate=mean(output.quality.requiresReview);
        entry.maxPlanePenetrationMm=max(0,max(-output.quality.minimumRodGapMm,[],'omitnan'));
        entry.trueContactCount=size(truth.contactForces,2); entry.estimatedContactCount=size(output.contactForce,2);
        entry.nonplanarAngleDeg=truth.nonplanarAngleDeg;
        entry.secondaryPlaneIsObserved=size(sensorInput.packet.planePointMm,2)>1;
        entry.meanFrameSeconds=output.optimizationSeconds/numel(output.timeSeconds); entry.seconds=toc(timer);
        entry.interpretation=truth.scope;
        atomic_write_artifact(fullfile(runFolder,[kind '.mat']),'mat',struct('sensorInput',sensorInput,'truth',truth,'output',output,'entry',entry));
        fprintf('MISMATCH_RESULT %s: total RMSE %.5g N; review %.1f%%; frame %.4g s\n',kind,entry.totalRmseN,100*entry.reviewRate,entry.meanFrameSeconds);
    catch err
        entry.errorIdentifier=err.identifier;entry.error=getReport(err,'extended','hyperlinks','off');entry.seconds=toc(timer);
        fprintf(2,'MISMATCH_FAILED %s: %s\n',kind,entry.error);
    end
    report.cases{end+1}=entry;publish();
end
report.caseCount=numel(report.cases);report.successCount=sum(cellfun(@(x)x.completed,report.cases));
report.state='complete';if report.successCount<report.caseCount,report.state='complete-with-failures';end
report.runRecord.state=report.state;publish();
fprintf('Model mismatch report saved to %s\n',fullfile(folder,'comparison.json'));
    function publish()
        atomic_write_artifact(fullfile(runFolder,'comparison.json'),'json',report);
        atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    end
end
function value=rmse(e),value=sqrt(mean(sum(e.^2,1)));end
