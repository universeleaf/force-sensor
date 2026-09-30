function report=run_formulation_cache_benchmark()
% Identical cold starts/constraints, sequential same-machine cache ablation.
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'out','benchmarks','mechanics_cache');
if ~isfolder(folder),mkdir(folder);end
report=struct('state','running','runRecord',new_run_record(),'cases',{{}}, ...
    'scope','Sequential cache on/off solves with identical cold starts and original formulation. End-to-end solver calls exclude truth generation, startup and IO. One run per condition; no real-time claim.');
[free,~]=build_formulation_free_packet();loaded=load(fullfile(root,'out','formulation_window','two_contact','input.mat'));
packets={free,loaded.sensorInput};ids={'free_space','two_contact'};
for i=1:2
    input=packets{i};dest=fullfile(folder,ids{i});if ~isfolder(dest),mkdir(dest);end
    atomic_write_artifact(fullfile(dest,'input.mat'),'mat',struct('sensorInput',input));
    estimates=cell(1,2);timing=zeros(1,2);
    for j=1:2
        enabled=j==2;clock=tic;
        estimates{j}=estimate_formulation_window(input,struct('cacheMechanics',enabled,'showProgress',true,'computeCovariance',false));
        timing(j)=toc(clock);name='disabled';if enabled,name='enabled';end
        atomic_write_artifact(fullfile(dest,[name '.mat']),'mat',struct('estimate',estimates{j}));
    end
    a=estimates{1};b=estimates{2};delta=max(abs([a.contactForce(:)-b.contactForce(:);a.tipForce(:)-b.tipForce(:)]));
    entry=struct('id',ids{i},'secondsDisabled',timing(1),'secondsEnabled',timing(2), ...
        'mechanicsCallsDisabled',a.solver.mechanicalEvaluations,'mechanicsCallsEnabled',b.solver.mechanicalEvaluations, ...
        'cacheHits',b.solver.mechanicalCacheHits,'maxForceDifferenceN',delta, ...
        'objectives',[a.objective b.objective],'qualityMatches',isequal(a.quality,b.quality), ...
        'artifactSha256',struct('input',file_sha256(fullfile(dest,'input.mat')), ...
        'disabled',file_sha256(fullfile(dest,'disabled.mat')),'enabled',file_sha256(fullfile(dest,'enabled.mat'))));
    assert(delta<1e-8&&entry.qualityMatches&&a.solver.mechanicalEvaluations>b.solver.mechanicalEvaluations, ...
        'rod:CacheEquivalence','Cache changed results or did not reduce ODE solves.');
    report.cases{end+1}=entry;atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
report.state='complete';report.runRecord.state='complete';atomic_write_artifact(fullfile(folder,'comparison.json'),'json',report);
end
