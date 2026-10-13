function export_relaxed_rotation
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');pub=fullfile(paths.published,getenv('TSFS_RUN_ID'));if ~isfolder(pub),mkdir(pub);end
for solver={'ipopt','fmincon'}
 name=solver{1};a=load(fullfile(folder,[name '.mat']));copyfile(fullfile(folder,[name '_frames.csv']),pub);traces=[];blocks=[];
 for k=1:numel(a.runs)
  s=a.runs{k}.mechanics;e=s.evaluation;N=numel(e.r)-6; % This demo has exactly one contact; generic solver does not assume it.
  assert(numel(a.runs{k}.geometry.s)==1);
  blocks=[blocks;struct('frame',k,'sensorObjective',0.5*sum(e.r(1:N).^2),'tipPriorObjective',0.5*sum(e.r(N+(1:3)).^2),'contactPriorObjective',0.5*sum(e.r(N+3+(1:3)).^2), ...
   'auxiliaryBetaMm',s.auxiliaryBeta,'seedDistance',norm(s.z-s.initialZ),'stationarity',s.stationarity)];
  if strcmp(name,'ipopt')
   t=s.trace;for j=1:numel(t.obj)
    traces=[traces;struct('frame',k,'iteration',j-1,'objective',t.obj(j)/s.objectiveScale,'primalViolation',t.inf_pr(j),'stationarity',t.inf_du(j),'stepLength',t.alpha_pr(j))];
   end
  else
   t=s.trace;for j=1:numel(t)
    traces=[traces;struct('frame',k,'iteration',t(j).iteration,'objective',t(j).objective,'primalViolation',t(j).constraintViolation,'stationarity',t(j).firstorderopt,'stepLength',NaN)];
   end
  end
 end
 writetable(struct2table(traces,'AsArray',true),fullfile(pub,[name '_iterations.csv']));writetable(struct2table(blocks,'AsArray',true),fullfile(pub,[name '_objective_blocks.csv']));
end
if isfile(fullfile(folder,'sensor_reconstruction.csv')),copyfile(fullfile(folder,'sensor_reconstruction.csv'),pub);end
if isfile(fullfile(folder,'truth_mu_300_audit.csv')),copyfile(fullfile(folder,'truth_mu_300_audit.csv'),pub);end
meta=struct('matlabVersion',version,'casadiVersion',casadi.CasadiMeta.version(),'epsilon',1e-6,'objectiveScale',1e-4,'forceScaleN',1,'momentScaleNmm',200, ...
 'gapScaleMm',1,'slipScaleMm',1,'maxIterations',300,'optimalityTolerance',1e-8,'constraintTolerance',1e-9,'fminconStepTolerance',1e-12, ...
 'ipoptAcceptableIter',0,'injectedNoiseStd',0,'sensorDeclaredStdPerMm',1e-6,'samplePeriodSeconds',0.02, ...
 'scope','Direct relaxation; no continuation/restarts. Native solver statuses retained. Previous finite states persist for history/priors even after nonconvergence, as in the existing method.');
atomic_write_artifact(fullfile(pub,'settings.json'),'json',meta);
end
