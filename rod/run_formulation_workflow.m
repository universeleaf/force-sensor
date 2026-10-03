function [estimate, record] = run_formulation_workflow(sensorInput, outputFolder, options)
%RUN_FORMULATION_WORKFLOW Complete calibrated-observation-to-artifact workflow.
% All supplied times participate in one offline joint solve. No samples are
% silently dropped. Truth belongs only in separate benchmark/scoring code.
if nargin<2||isempty(outputFolder)
    outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'out','formulation_workflow');
end
if nargin<3, options=struct; end
if ~isfolder(outputFolder), mkdir(outputFolder); end
record=struct('state','running','runRecord',new_run_record(),'completed',false, ...
    'scope','Offline joint formulation MAP from calibrated model and shape/environment observations.');
manifest=fullfile(outputFolder,'manifest.json'); atomic_write_artifact(manifest,'json',record);
atomic_write_artifact(fullfile(outputFolder,'input.mat'),'mat',struct('sensorInput',sensorInput));
try
    estimate=estimate_formulation_forces(sensorInput,options);
    atomic_write_artifact(fullfile(outputFolder,'estimate.mat'),'mat',struct('estimate',estimate));
    write_formulation_window_csv(estimate,fullfile(outputFolder,'forces.csv'));
    record.completed=true; record.quality=estimate.quality; record.solver=estimate.solver;
    record.frameCount=numel(estimate.timeSeconds); record.candidateCount=size(estimate.contactForce,2);
    record.seconds=estimate.optimizationSeconds; record.candidateAudit=estimate.candidateAudit;
    if isfield(estimate,'endToEndSeconds'),record.endToEndSeconds=estimate.endToEndSeconds;end
    record.artifactSha256=struct('input',file_sha256(fullfile(outputFolder,'input.mat')), ...
        'estimate',file_sha256(fullfile(outputFolder,'estimate.mat')),'forces',file_sha256(fullfile(outputFolder,'forces.csv')));
catch err
    record.state='failed';record.runRecord.state='failed';record.error=getReport(err,'extended','hyperlinks','off');
    atomic_write_artifact(manifest,'json',record); rethrow(err);
end
record.state='complete'; record.runRecord.state='complete'; atomic_write_artifact(manifest,'json',record);
end
