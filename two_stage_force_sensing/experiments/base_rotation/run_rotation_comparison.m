function run_rotation_comparison(part)
% Separate current-method replay on the rotation packet; unchanged core.
if nargin<1,part='ours';end
[root,paths]=setup_tsfs;assert(startsWith(getenv('TSFS_RUN_ID'),'base_rotation_'));
folder=fullfile(paths.results,'rotation');a=load(fullfile(folder,'truth_mu_300.mat'));input=a.input;t=a.truth;
frames=tsfs.read_input(input);o=tsfs.defaults;cfg=input.config;
cfg.aloi=struct('numCenterCandidates',15,'sigmaCandidatesMm',[3 8 16 30],'amplitudeBoundN',100, ...
 'positionStdMm',0.2,'maxEquilibriumIterations',15,'equilibriumToleranceMm',1e-3,'equilibriumRelaxation',0.5, ...
 'maxIterations',25,'maxFunctionEvaluations',180,'numOptimizationStarts',2,'showProgress',true);
assert(max(abs(input.packet.curvaturePerMm(1:2,:,:)-t.u(1:2,input.packet.fbgIdx,:)),[],'all')==0);
if ismember(part,{'ours','inverse'})
 if strcmp(part,'ours'),mus=[0.3 0];reps=2;else,mus=[0.29 0.31];reps=1;end
 runs=cell(numel(mus),reps,numel(frames));rows=[];
 meta=struct('anglesDeg',t.angleDeg,'trueMu',t.mu,'insertionMm',18,'tipForceWorldN',[1;0;-1], ...
  'noiseStd',0,'observedAxes',[1 2],'options',o,'aloiOptions',cfg.aloi,'initialization',t.initialization, ...
  'provenance',baseline_provenance(paths.baseline,paths.lcp));
 atomic_write_artifact(fullfile(folder,'metadata.json'),'json',meta);
 for rep=1:reps
  for j=1:numel(mus)
   previous=[];
   for k=1:numel(frames)
    fr=frames{k};fr.mu(:)=mus(j);fr.previous.mu(:)=mus(j);
    try
     out=tsfs.step(fr,previous,o);previous=out;row=score(out,t,k,mus(j),rep);
     out.mechanics.evaluation.shape.pieces={};out.geometry.kinematicShape.Jp=[];out.geometry.kinematicShape.Jt=[];runs{j,rep,k}=out;
    catch err
     failure=struct('angleDeg',t.angleDeg(k),'mu',mus(j),'repeat',rep,'identifier',err.identifier,'message',err.message);
     atomic_write_artifact(fullfile(folder,sprintf('%s_failure_mu_%03d_rep%d_frame%d.json',part,round(mus(j)*1000),rep,k)),'json',failure);
     rethrow(err); % Keep every completed row; a thrown model error is not a valid estimate.
    end
    rows=[rows;row];writetable(struct2table(rows,'AsArray',true),fullfile(folder,[part '_frames.csv']));save(fullfile(folder,[part '.mat']),'runs','rows','mus','o','-v7.3');
    fprintf('%s mu %.2f rep %d angle %.2f: %.3fs SQP %d IVP %d tip %.3g shape %.3g flags %s\n',part,mus(j),rep,t.angleDeg(k),row.seconds,row.sqpIterations,row.totalIVPs,row.tipErrorN,row.shapeRmsMm,row.flags);
   end
  end
 end
elseif ismember(part,{'aloi','profile'})
 clock=tic;m=measurements_from_sensor_packet(input.tube,input.packet,cfg);preparationSeconds=toc(clock);
 if strcmp(part,'profile'),profile clear;profile on;clean=onCleanup(@()profile('off'));end
 clock=tic;aloi=estimate_aloi_gaussian_baseline(input.tube,m,cfg);seconds=toc(clock);
 if strcmp(part,'profile')
  profile off;profileInfo=profile('info');pt=profileInfo.FunctionTable;keep=contains({pt.FunctionName},{'solveShape','predictGaussianShape','positionResidual','fitGaussianToSparsePositions','lsqnonlin','ode45'});pr=pt(keep);summary=[];
  for q=1:numel(pr),summary=[summary;struct('name',pr(q).FunctionName,'calls',pr(q).NumCalls,'seconds',pr(q).TotalTime,'file',pr(q).FileName)];end
  save(fullfile(folder,'aloi_profile.mat'),'profileInfo','seconds','-v7.3');atomic_write_artifact(fullfile(folder,'aloi_profile.json'),'json',struct('profiledSeconds',seconds,'functions',summary));
 else
  rows=[];
  for k=1:numel(frames)
   err=aloi.p(:,:,k)-t.p(:,:,k);w=rotation_material_twist(aloi.R(:,:,:,k),t.sMm);
   rows=[rows;struct('frame',k,'angleDeg',t.angleDeg(k),'totalErrorN',norm(aloi.totalForceResultant(:,k)-t.totalForce(:,k)), ...
    'totalFx',aloi.totalForceResultant(1,k),'totalFy',aloi.totalForceResultant(2,k),'totalFz',aloi.totalForceResultant(3,k), ...
    'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipErrorMm',norm(err(:,end)),'twistIntegralDeg',rad2deg(sum(w.*diff(t.sMm))), ...
    'observedShapeRmsMm',aloi.shapeRmseMm(k),'iterationsWinningStart',aloi.solverIterations(k),'starts',aloi.optimizationStarts(k), ...
    'exitFlag',aloi.solverExitFlag(k),'equilibriumIterationsFinal',aloi.equilibriumIterations(k),'equilibriumResidualMm',aloi.equilibriumResidualMm(k), ...
    'equilibriumConverged',aloi.equilibriumConverged(k),'centerMm',aloi.centerMm(k),'widthMm',aloi.sigmaMm(k))];
  end
  save(fullfile(folder,'aloi.mat'),'aloi','rows','seconds','preparationSeconds','cfg','-v7.3');writetable(struct2table(rows,'AsArray',true),fullfile(folder,'aloi_frames.csv'));
  atomic_write_artifact(fullfile(folder,'aloi_timing.json'),'json',struct('sequenceSeconds',seconds,'meanFrameSeconds',seconds/numel(frames),'preparationSeconds',preparationSeconds,'frames',numel(frames)));
 end
elseif strcmp(part,'forward')
 for mu=[0 0.29 0.31],build_rotation_truth(mu);end
else,error('Unknown part');end
end
function row=score(out,t,k,mu,rep)
m=out.mechanics;d=out.diagnostics;sh=m.evaluation.shape;c=m.counts;fc=sum(out.contactForce,2);tip=out.tipForce;err=sh.p-t.p(:,:,k);n=t.planeNormal;
arc=NaN;if numel(out.geometry.s)==1,arc=out.geometry.s(1);end
ge=out.geometry.kinematicShape.p-t.p(:,:,k);kt=out.geometry.normal;
row=struct('mu',mu,'repeat',rep,'frame',k,'angleDeg',t.angleDeg(k),'seconds',out.seconds,'geometrySeconds',out.geometry.seconds,'mechanicsSeconds',m.seconds,'diagnosticSeconds',d.seconds, ...
 'geometryIterations',out.geometry.iterations,'sqpIterations',m.iterations,'qpIterations',c.qpIterations,'totalIVPs',c.stateIVPs+c.augmentedIVPs,'stateIVPs',c.stateIVPs,'augmentedIVPs',c.augmentedIVPs,'odeSegments',c.odeSegments,'rhsEvaluations',c.rhsEvaluations,'rejectedTrials',c.trialRejections, ...
 'converged',m.converged,'contactCount',numel(out.geometry.s),'contactErrorN',norm(fc-t.contactForce(:,k)),'tipErrorN',norm(tip-t.tipForce(:,k)),'totalErrorN',norm(fc+tip-t.totalForce(:,k)), ...
 'contactFx',fc(1),'contactFy',fc(2),'contactFz',fc(3),'tipFx',tip(1),'tipFy',tip(2),'tipFz',tip(3), ...
 'frictionMagnitudeN',norm(fc-n*(n'*fc)),'trueContactFx',t.contactForce(1,k),'trueContactFy',t.contactForce(2,k),'trueContactFz',t.contactForce(3,k), ...
 'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeXRmsMm',sqrt(mean(err(1,:).^2)),'shapeYRmsMm',sqrt(mean(err(2,:).^2)),'shapeZRmsMm',sqrt(mean(err(3,:).^2)), ...
 'shapeTipErrorMm',norm(err(:,end)),'fbgRms',d.fbgRms,'stage1ShapeRmsMm',sqrt(mean(sum(ge.^2,1))), ...
 'twistIntegralDeg',rad2deg(trapz(t.sMm,sh.u(3,:))),'trueTwistIntegralDeg',rad2deg(trapz(t.sMm,t.u(3,:,k))),'twistRmsErrorPerMm',sqrt(mean((sh.u(3,:)-t.u(3,:,k)).^2)), ...
 'arcMm',arc,'trueArcMm',t.contactS(k),'normalErrorRad',acos(max(-1,min(1,n'*kt(:,1)))),'penetrationMm',d.maximumPenetrationMm, ...
 'truthPlanePenetrationMm',max(0,-min(n'*(sh.collisionP-t.planePoint))),'projectionN',max([0;abs(d.projectionN(:))]), ...
 'tangency',max([0;abs(d.tangency(:))]),'frictionWorkNmm',sum(d.frictionWorkNmm),'tipMomentNmm',norm(d.tipMomentNmm),'condition',d.forceCondition,'flags',strjoin(d.flags,';'));
end
function w=rotation_material_twist(R,s)
w=zeros(1,numel(s)-1);
for k=1:numel(w),A=real(logm(R(:,:,k)'*R(:,:,k+1)));w(k)=A(2,1)/(s(k+1)-s(k));end
end
