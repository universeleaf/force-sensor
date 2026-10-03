function report=run_formulation_derivative_benchmark()
% Matched cold starts, geometry/arc domain/factors/covariance all identical.
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'out','benchmarks','formulation_derivatives');
if ~isfolder(folder),mkdir(folder);end
loaded=load(fullfile(root,'out','formulation_window','two_contact','input.mat'),'sensorInput');
sensorInput=loaded.sensorInput;atomic_write_artifact(fullfile(folder,'input.mat'),'mat',struct('sensorInput',sensorInput));
report=struct('state','running','runRecord',new_run_record(),'cases',{{}}, ...
    'inputSha256',file_sha256(fullfile(folder,'input.mat')), ...
    'scope','Two sequential, identical cold-start full-window solves including covariance. The disabled version estimates constraint derivatives separately; enabled packs all Jacobians into one shared numerical central-difference sweep. One repeat and fixed order; no latency significance/real-time claim.');
path=fullfile(folder,'comparison.json');atomic_write_artifact(path,'json',report);estimates=cell(1,2);
for j=1:2
    enabled=j==2;name='separate';if enabled,name='shared';end
    clock=tic;estimate=estimate_formulation_window(sensorInput,struct('useSharedDerivatives',enabled, ...
        'showProgress',true,'computeCovariance',true));seconds=toc(clock);estimates{j}=estimate;
    dest=fullfile(folder,[name '.mat']);atomic_write_artifact(dest,'mat',struct('estimate',estimate));
    entry=struct('id',name,'seconds',seconds,'objective',estimate.objective,'solver',estimate.solver, ...
        'quality',estimate.quality,'estimateSha256',file_sha256(dest));
    report.cases{end+1}=entry;atomic_write_artifact(path,'json',report);
end
a=estimates{1};b=estimates{2};report.maxForceDifferenceN=max(abs([a.contactForce(:)-b.contactForce(:);a.tipForce(:)-b.tipForce(:)]));
report.maxArcDifferenceMm=max(abs(a.contactArcLength(:)-b.contactArcLength(:)));
assert(report.maxForceDifferenceN<1e-4&&report.maxArcDifferenceMm<1e-4&& ...
    ~any(a.quality.requiresReview)&&~any(b.quality.requiresReview), ...
    'rod:DerivativeBenchmarkRegression','Derivative implementation changed clean accuracy or failed the physical audit.');
report.state='complete';report.runRecord.state='complete';atomic_write_artifact(path,'json',report);
end
