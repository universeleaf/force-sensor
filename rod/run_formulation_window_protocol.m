function report = run_formulation_window_protocol(quickMode)
%RUN_FORMULATION_WINDOW_PROTOCOL Reproducible end-to-end formulation evidence.
% Smoke outputs are isolated from full protocols. Failed cases stay in JSON.
if nargin<1, quickMode=false; end
root=fileparts(fileparts(mfilename('fullpath'))); folder=fullfile(root,'out','formulation_window');
if quickMode, folder=fullfile(folder,'smoke'); end
if ~isfolder(folder), mkdir(folder); end
report=struct('state','running','runRecord',new_run_record(),'quickMode',quickMode, ...
    'scope','Independent truth, sparse sensor packets, shape-derived candidates, full 3-D window MPCC, explicit quality/scoring.', ...
    'cases',{{}},'sotaEstablished',false);
scenes=multi_contact_demo_scenes(); ids={'two_contact','tapered_two_contact','three_contact','rotated_two_contact','three_contact_noisy','spatial_sliding','spatial_sliding_noisy'};
if quickMode, ids=ids(1); end
for j=1:numel(ids)
    item=struct('id',ids{j},'completed',false,'error','','metrics',[],'artifactFolder',ids{j});
    try
        if contains(ids{j},'spatial')
            sigma=0; if contains(ids{j},'noisy'), sigma=2.5e-5; end
            [sensorInput,truth]=build_spatial_friction_packet(sigma,811);
        else
            scene=scenes(1); if contains(ids{j},'three'), scene=scenes(3); end
            if contains(ids{j},'tapered'), scene=scenes(2); end
            if contains(ids{j},'noisy'), scene.curvatureNoiseStd=2.5e-5; end
            Q=eye(3); if contains(ids{j},'rotated'), a=0.6; b=0.4; Q=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1]*[1 0 0;0 cos(b) -sin(b);0 sin(b) cos(b)]; end
            indices=[3 4]; if quickMode, indices=3; end
            [sensorInput,truth]=build_formulation_multi_packet(scene,indices,Q);
        end
        caseFolder=fullfile(folder,ids{j}); if ~isfolder(caseFolder), mkdir(caseFolder); end
        atomic_write_artifact(fullfile(caseFolder,'input.mat'),'mat',struct('sensorInput',sensorInput));
        atomic_write_artifact(fullfile(caseFolder,'truth.mat'),'mat',struct('truth',truth));
        % Keep the exact failed input too; successful estimates alone would
        % make numerical failures impossible to reproduce.
        estimate=estimate_formulation_forces(sensorInput,struct('showProgress',true,'computeCovariance',true));
        item.metrics=score_formulation_window(estimate,truth); item.completed=true;
        atomic_write_artifact(fullfile(caseFolder,'estimate.mat'),'mat',struct('estimate',estimate));
        write_formulation_window_csv(estimate,fullfile(caseFolder,'forces.csv'));
        item.quality=estimate.quality; item.solver=estimate.solver; item.candidateAudit=estimate.candidateAudit;
        item.artifactSha256=struct('input',file_sha256(fullfile(caseFolder,'input.mat')), ...
            'truth',file_sha256(fullfile(caseFolder,'truth.mat')),'estimate',file_sha256(fullfile(caseFolder,'estimate.mat')));
    catch err, item.error=getReport(err,'extended','hyperlinks','off'); end
    report.cases{end+1}=item;
    atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
    fprintf('Formulation window %s: completed=%d\n',ids{j},item.completed);
end
report.state='complete'; report.runRecord.state='complete';
report.failureCount=sum(cellfun(@(c)~c.completed,report.cases));
atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
