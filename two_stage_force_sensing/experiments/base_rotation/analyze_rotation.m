function analyze_rotation
% Saved-output analysis only. No new assumptions supplied to any estimator.
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');
a=load(fullfile(folder,'truth_mu_300.mat'));t=a.truth;d=load(fullfile(folder,'ours.mat'));iv=load(fullfile(folder,'inverse.mat'));b=load(fullfile(folder,'aloi.mat'));
f0=load(fullfile(folder,'truth_mu_000.mat'));fm=load(fullfile(folder,'truth_mu_290.mat'));fp=load(fullfile(folder,'truth_mu_310.mat'));
rows=[];physics=[];shapeRows=[];traceRows=[];local=cell(1,numel(t.angleDeg));frames=tsfs.read_input(a.input);
for k=1:numel(t.angleDeg)
 plus=iv.runs{2,1,k};minus=iv.runs{1,1,k};dt=(plus.tipForce-minus.tipForce)/0.02;df=(sum(plus.contactForce,2)-sum(minus.contactForce,2))/0.02;
 di=(plus.mechanics.evaluation.shape.p-minus.mechanics.evaluation.shape.p)/0.02;
 dp=(fp.truth.p(:,:,k)-fm.truth.p(:,:,k))/0.02;dc=(fp.truth.contactForce(:,k)-fm.truth.contactForce(:,k))/0.02;
 twistDerivative=rad2deg(trapz(t.sMm,(fp.truth.u(3,:,k)-fm.truth.u(3,:,k))/0.02));
 delta=t.p(:,:,k)-f0.truth.p(:,:,k);
 % Offline attribution only: true twist is never supplied to an estimator.
 kin=tsfs.reconstruct(frames{k},false);restoredU=kin.u;restoredU(3,:)=t.u(3,:,k);
 [~,restoredP]=integrate_curvature_field(a.input.tube,restoredU,frames{k}.base);
 restoredError=restoredP-t.p(:,:,k);
 contactDelta=t.contactForce(:,k)-f0.truth.contactForce(:,k);
 rows=[rows;struct('frame',k,'angleDeg',t.angleDeg(k),'dInverseTipX_dMu',dt(1),'dInverseTipY_dMu',dt(2),'dInverseTipZ_dMu',dt(3), ...
  'dInverseContactX_dMu',df(1),'dInverseContactY_dMu',df(2),'dInverseContactZ_dMu',df(3),'dInverseShapeRmsMm_dMu',sqrt(mean(sum(di.^2,1))), ...
  'inverseBothConverged',plus.mechanics.converged&&minus.mechanics.converged,'offlineTrueTwistReconstructionRmsMm',sqrt(mean(sum(restoredError.^2,1))), ...
  'dForwardContactX_dMu',dc(1),'dForwardContactY_dMu',dc(2),'dForwardContactZ_dMu',dc(3),'dForwardShapeRmsMm_dMu',sqrt(mean(sum(dp.^2,1))), ...
  'dForwardTipXmm_dMu',dp(1,end),'dForwardTipYmm_dMu',dp(2,end),'dForwardTipZmm_dMu',dp(3,end),'dForwardTwistDeg_dMu',twistDerivative, ...
  'frictionInducedShapeRmsMm',sqrt(mean(sum(delta.^2,1))),'frictionInducedTipXmm',delta(1,end),'frictionInducedTipYmm',delta(2,end),'frictionInducedTipZmm',delta(3,end), ...
  'frictionInducedContactChangeN',norm(contactDelta),'zeroTruthTwistDeg',rad2deg(trapz(t.sMm,f0.truth.u(3,:,k))))];
 for j=1:2
  out=d.runs{j,2,k};q=out.diagnostics;e=out.mechanics.evaluation;g=out.geometry;
  actualSlip=zeros(3,1);
  if k>1,old=t.shapes{k-1};actualSlip=t.shapes{k}.contactPoints-interp1(old.collisionS,old.collisionP',t.contactS(k),'pchip')';end
  actualSlip=actualSlip-t.planeNormal*(t.planeNormal'*actualSlip);worldF=sum(out.contactForce,2);
  physics=[physics;struct('mu',d.mus(j),'frame',k,'angleDeg',t.angleDeg(k),'slipMm',norm(e.slip(:)), ...
   'gapMm',max([0;abs(q.gapMm(:))]),'coneExcessN',max([0;q.coneExcessN(:)]),'normalProductNmm',max([0;abs(q.normalProductNmm(:))]), ...
   'frictionWorkNmm',sum(q.frictionWorkNmm),'slidingLawErrorN',max([0;q.slidingLawErrorN(:)]),'contactPointMismatchMm',max([0;q.contactPointMismatchMm(:)]), ...
   'estimatedFrictionN',norm(e.shape.x(8:9)),'estimatedNormalN',e.shape.x(7),'inferredContactWorkAgainstTrueSlipNmm',worldF'*actualSlip,'trueSlipMm',norm(actualSlip),'historySource',out.mechanics.historySource,'flags',strjoin(q.flags,';'))];
  for h=1:numel(out.mechanics.trace)
   tr=out.mechanics.trace(h);traceRows=[traceRows;struct('mu',d.mus(j),'frame',k,'angleDeg',t.angleDeg(k),'iteration',tr.iteration, ...
    'objective',tr.objective,'equalityInf',tr.eqInf,'inequalityInf',tr.inequalityInf,'stationarity',tr.stationarity,'stepInf',tr.stepInf,'alpha',tr.alpha,'accepted',tr.accepted)];
  end
 end
 out=d.runs{1,2,k};zero=d.runs{2,2,k};g=out.geometry;m=out.mechanics;sh=m.evaluation.shape;scale=m.scale;
 entry=struct('frame',k,'angleDeg',t.angleDeg(k),'converged',m.converged,'contactCount',numel(g.s), ...
  'stage1ArcDifferenceMm',norm(zero.geometry.s-g.s),'stage1NormalDifference',norm(zero.geometry.normal-g.normal));
 if numel(g.s)==1
  A=[g.normal(:,g.plane(1)) g.B(:,:,1)];Ju=sh.Jprediction./scale';E=sh.JtipMoment./scale';eliminate=E(:,1:3)\E(:,4:end);transform=blkdiag(eye(3),A);
  G=(Ju(:,4:end)-Ju(:,1:3)*eliminate)/transform;sv=svd(G);[~,~,V]=svd(G,0);
  entry.forceCondition=sv(1)/sv(end);entry.singularValuesPerMmPerN=sv;entry.weakForceDirection=V(:,end);
  entry.tipYContactYCorrelation=dot(G(:,2),G(:,5))/(norm(G(:,2))*norm(G(:,5)));
  Jc=m.evaluation.Jc./scale';Cf=(Jc(4:end,4:end)-Jc(4:end,1:3)*eliminate)/transform;N=null(Cf);sf=svd(G*N);
  entry.feasibleTangentDimension=size(N,2);entry.feasibleTangentCondition=sf(1)/sf(end);
 end
 local{k}=entry;
 if ismember(k,[1 7 numel(t.angleDeg)])
  q=d.runs{1,2,k}.mechanics.evaluation.shape;z=d.runs{2,2,k}.mechanics.evaluation.shape;
  for i=1:numel(t.sMm)
   shapeRows=[shapeRows;struct('frame',k,'angleDeg',t.angleDeg(k),'arcMm',t.sMm(i), ...
    'trueX',t.p(1,i,k),'trueY',t.p(2,i,k),'trueZ',t.p(3,i,k),'oursX',q.p(1,i),'oursY',q.p(2,i),'oursZ',q.p(3,i), ...
    'zeroX',z.p(1,i),'zeroY',z.p(2,i),'zeroZ',z.p(3,i),'aloiX',b.aloi.p(1,i,k),'aloiY',b.aloi.p(2,i,k),'aloiZ',b.aloi.p(3,i,k), ...
    'zeroTruthX',f0.truth.p(1,i,k),'zeroTruthY',f0.truth.p(2,i,k),'zeroTruthZ',f0.truth.p(3,i,k), ...
    'trueTwistPerMm',t.u(3,i,k),'oursTwistPerMm',q.u(3,i),'zeroTwistPerMm',z.u(3,i),'zeroTruthTwistPerMm',f0.truth.u(3,i,k))];
  end
 end
end
writetable(struct2table(rows,'AsArray',true),fullfile(folder,'sensitivity.csv'));writetable(struct2table(physics,'AsArray',true),fullfile(folder,'physics.csv'));writetable(struct2table(shapeRows,'AsArray',true),fullfile(folder,'shape_profiles.csv'));
writetable(struct2table(traceRows,'AsArray',true),fullfile(folder,'iteration_trace.csv'));
atomic_write_artifact(fullfile(folder,'local_sensitivity.json'),'json',local);
% A tighter independent map at the final state checks reference ODE error.
tube=a.input.tube;tube.T_base=t.basePose(:,:,end);opts=a.mapOptions;opts.relativeTolerance=2e-10;opts.momentToleranceNmm=1e-6;
check=solve_cosserat_multi_contact_map(tube,t.contactS(end),t.contactForce(:,end),t.tipForce(:,end),opts);
v=struct('finalReferenceTighterPositionMaxMm',max(vecnorm(check.p-t.p(:,:,end))),'finalReferenceTighterCurvatureMaxPerMm',max(abs(check.u-t.u(:,:,end)),[],'all'), ...
 'finalReferenceTerminalMomentNmm',check.tipMomentResidualNmm,'noiseMaximumPerMm',max(abs(a.input.packet.curvaturePerMm(1:2,:,:)-t.u(1:2,a.input.packet.fbgIdx,:)),[],'all'));
assert(v.finalReferenceTighterPositionMaxMm<1e-4&&v.finalReferenceTighterCurvatureMaxPerMm<1e-7&&v.noiseMaximumPerMm==0);
atomic_write_artifact(fullfile(folder,'reference_validation.json'),'json',v);
fprintf('Rotation analysis exported.\n');
end
