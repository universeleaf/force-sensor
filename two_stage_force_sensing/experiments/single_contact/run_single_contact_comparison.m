function run_single_contact_comparison(part)
% Paired noiseless sliding comparison. No upstream algorithm is modified.
if nargin<1,part='ours';end
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'single_contact');if ~isfolder(folder),mkdir(folder);end
a=load(fullfile(root,'datasets','sliding_clean.mat'));input=a.sensorInput;truth=a.truth;frames=tsfs.read_input(input);o=tsfs.defaults;
observed=truth.u(input.config.forceSensor.curvatureObservedAxes,input.packet.fbgIdx,:);
measured=input.packet.curvaturePerMm(input.config.forceSensor.curvatureObservedAxes,:,:);
assert(max(abs(observed-measured),[],'all')<1e-12,'Input is not noiseless.');
assert(all(input.packet.frictionMu==0.3));
cfg=input.config;
% Identical to Tongyu's run_fair_baseline_protocol baselineAloiConfig.
cfg.aloi=struct('numCenterCandidates',15,'sigmaCandidatesMm',[3 8 16 30],'amplitudeBoundN',100, ...
 'positionStdMm',0.2,'maxEquilibriumIterations',15,'equilibriumToleranceMm',1e-3, ...
 'equilibriumRelaxation',0.5,'maxIterations',25,'maxFunctionEvaluations',180, ...
 'numOptimizationStarts',2,'showProgress',true);
if strcmp(part,'ours')
 metadata=struct('trueMu',0.3,'noiseMaximumPerMm',max(abs(observed-measured),[],'all'),'frames',numel(frames), ...
  'axes',frames{1}.axes,'fbgStdPerMm',sqrt(diag(frames{1}.CU)),'options',o,'aloiOptions',cfg.aloi, ...
  'scope','Same recorded input/truth. Only estimator friction coefficient changes. No contact count, geometry, forces or modes from truth are estimator inputs.', ...
  'provenance',baseline_provenance(paths.baseline,paths.lcp));
 atomic_write_artifact(fullfile(folder,'metadata.json'),'json',metadata);
 mus=[0.3 0];runs=cell(2,2,numel(frames));rows=[];
 for rep=1:2
  for j=1:2
   previous=[];
   for k=1:numel(frames)
    fr=frames{k};fr.mu(:)=mus(j);if ~isempty(fr.previous),fr.previous.mu(:)=mus(j);end
    out=tsfs.step(fr,previous,o);previous=out;
    row=score(out,truth,k,mus(j),rep);rows=[rows;row];
    out.mechanics.evaluation.shape.pieces={};out.geometry.kinematicShape.Jp=[];out.geometry.kinematicShape.Jt=[];runs{j,rep,k}=out;
    fprintf('OURS mu %.2f rep %d frame %d: %.3fs SQP %d IVP %d tip %.5g contact %.5g shape %.5g flags %s\n',mus(j),rep,k,row.seconds,row.sqpIterations,row.totalIVPs,row.tipErrorN,row.contactErrorN,row.shapeRmsMm,row.flags);
   end
   save(fullfile(folder,'ours.mat'),'runs','rows','mus','o','-v7.3');writetable(struct2table(rows),fullfile(folder,'ours_frames.csv'));
  end
 end
elseif ismember(part,{'aloi','profile'})
 clock=tic;measurements=measurements_from_sensor_packet(input.tube,input.packet,cfg);preparationSeconds=toc(clock);
 if strcmp(part,'profile'),profile clear;profile on;cleanup=onCleanup(@()profile('off'));end
 clock=tic;aloi=estimate_aloi_gaussian_baseline(input.tube,measurements,cfg);seconds=toc(clock);
 if strcmp(part,'profile')
  profile off;profileInfo=profile('info');save(fullfile(folder,'aloi_profile.mat'),'profileInfo','seconds','-v7.3');
  pt=profileInfo.FunctionTable;keep=contains({pt.FunctionName},{'solveShape','predictGaussianShape','positionResidual','fitGaussianToSparsePositions','lsqnonlin','ode45'});
  pr=pt(keep);summary=[];
  for q=1:numel(pr),summary=[summary;struct('name',pr(q).FunctionName,'calls',pr(q).NumCalls,'seconds',pr(q).TotalTime,'file',pr(q).FileName)];end
  atomic_write_artifact(fullfile(folder,'aloi_profile.json'),'json',struct('profiledSeconds',seconds,'functions',summary));
 else
  rows=[];
  for k=1:numel(frames)
   err=aloi.p(:,:,k)-truth.p(:,:,k);
   rows=[rows;struct('frame',k,'totalErrorN',norm(aloi.totalForceResultant(:,k)-truth.totalForce(:,k)), ...
    'totalFx',aloi.totalForceResultant(1,k),'totalFy',aloi.totalForceResultant(2,k),'totalFz',aloi.totalForceResultant(3,k), ...
    'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipErrorMm',norm(err(:,end)), ...
    'observedShapeRmsMm',aloi.shapeRmseMm(k),'iterationsWinningStart',aloi.solverIterations(k), ...
    'starts',aloi.optimizationStarts(k),'exitFlag',aloi.solverExitFlag(k),'equilibriumIterationsFinal',aloi.equilibriumIterations(k), ...
    'equilibriumResidualMm',aloi.equilibriumResidualMm(k),'equilibriumConverged',aloi.equilibriumConverged(k), ...
    'centerMm',aloi.centerMm(k),'widthMm',aloi.sigmaMm(k))];
  end
  save(fullfile(folder,'aloi.mat'),'aloi','rows','seconds','preparationSeconds','cfg','-v7.3');
  writetable(struct2table(rows),fullfile(folder,'aloi_frames.csv'));
  atomic_write_artifact(fullfile(folder,'aloi_timing.json'),'json',struct('sequenceSeconds',seconds,'meanFrameSeconds',seconds/numel(frames),'preparationSeconds',preparationSeconds,'frames',numel(frames), ...
   'note','Unprofiled full-sequence call preserves previous-frame warm starts. Per-frame timing and total iterations across both starts are not exposed by the API.'));
 end
else,error('Unknown part');end
end
function row=score(out,t,k,mu,rep)
m=out.mechanics;d=out.diagnostics;sh=m.evaluation.shape;c=m.counts;fc=sum(out.contactForce,2);tip=out.tipForce;err=sh.p-t.p(:,:,k);n=t.planeNormal;
row=struct('mu',mu,'repeat',rep,'frame',k,'seconds',out.seconds,'geometrySeconds',out.geometry.seconds,'mechanicsSeconds',m.seconds,'diagnosticSeconds',d.seconds, ...
 'geometryIterations',out.geometry.iterations,'sqpIterations',m.iterations,'qpIterations',c.qpIterations,'totalIVPs',c.stateIVPs+c.augmentedIVPs,'stateIVPs',c.stateIVPs,'augmentedIVPs',c.augmentedIVPs,'rhsEvaluations',c.rhsEvaluations,'rejectedTrials',c.trialRejections, ...
 'converged',m.converged,'contactErrorN',norm(fc-t.contactForce(:,k)),'tipErrorN',norm(tip-t.tipForce(:,k)),'totalErrorN',norm(fc+tip-t.totalForce(:,k)), ...
 'contactFx',fc(1),'contactFy',fc(2),'contactFz',fc(3),'tipFx',tip(1),'tipFy',tip(2),'tipFz',tip(3), ...
 'trueContactFx',t.contactForce(1,k),'trueContactFy',t.contactForce(2,k),'trueContactFz',t.contactForce(3,k), ...
 'trueTipFx',t.tipForce(1,k),'trueTipFy',t.tipForce(2,k),'trueTipFz',t.tipForce(3,k), ...
 'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipErrorMm',norm(err(:,end)),'fbgRms',d.fbgRms, ...
 'arcMm',out.geometry.s(1),'trueArcMm',t.contactS(k),'penetrationMm',d.maximumPenetrationMm,'truthPlanePenetrationMm',max(0,-min(n'*(sh.p-t.planePoint))), ...
 'projectionN',max(abs(d.projectionN),[],'all'),'tangency',max(abs(d.tangency)),'tipMomentNmm',norm(d.tipMomentNmm),'condition',d.forceCondition,'flags',strjoin(d.flags,';'));
end
