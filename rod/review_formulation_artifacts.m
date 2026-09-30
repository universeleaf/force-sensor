function report = review_formulation_artifacts(folder)
%REVIEW_FORMULATION_ARTIFACTS Reassess saved fits without changing force results.
% Writes an explicitly derived report with its own source snapshot and the
% exact SHA-256 of each original estimate. No force optimization or truth
% scoring is repeated; inference files and original quality remain intact.
comparison=fullfile(folder,'comparison.json'); manifest=fullfile(folder,'manifest.json');
if isfile(comparison),original=jsondecode(fileread(comparison));
else,original=jsondecode(fileread(manifest));end
assert(any(strcmp(original.state,{'complete','complete-with-failures'})), ...
    'rod:IncompleteExperiment','Do not reassess a running or failed experiment as completed.');
report=struct('state','running','runRecord',new_run_record(), ...
    'originalRunId',original.runRecord.runId,'cases',{{}}, ...
    'scope','Derived observation-fit review only; original forces, solver provenance and quality are unchanged.');
if isfield(original,'cases')
    items=original.cases;if isstruct(items),items=num2cell(items);end
else,items={original};end
for k=1:numel(items)
    item=items{k};row=struct('id','','completed',false);
    if isfield(item,'id')
        row.id=item.id;path=fullfile(folder,item.artifactFolder,'estimate.mat');variable='estimate';
    elseif isfield(item,'kind')
        row.id=item.kind;path=fullfile(folder,original.runRecord.runId,[item.kind '.mat']);variable='output';
    else,row.id='workflow';path=fullfile(folder,'estimate.mat');variable='estimate';end
    if item.completed
        try
            data=load(path,variable);estimate=data.(variable);
            row.estimateSha256=file_sha256(path);row.observationFit=audit_formulation_window_fit(estimate);
            row.originalRequiresReview=estimate.quality.requiresReview;
            row.combinedRequiresReview=row.originalRequiresReview|row.observationFit.requiresReview;
            row.completed=true;
        catch err,row.error=getReport(err,'extended','hyperlinks','off');end
    else,row.error='Original inference did not complete; no force result reassessed.';end
    report.cases{end+1}=row;
end
report.failureCount=sum(cellfun(@(c)~c.completed,report.cases));
report.state='complete';report.runRecord.state='complete';
atomic_write_artifact(fullfile(folder,'observation_fit.json'),'json',report);
end
