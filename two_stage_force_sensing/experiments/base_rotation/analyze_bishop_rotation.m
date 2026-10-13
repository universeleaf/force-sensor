function analyze_bishop_rotation
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');a=load(fullfile(folder,'truth_mu_300.mat'));t=a.truth;d=load(fullfile(folder,'ours.mat'));b=load(fullfile(folder,'aloi.mat'));frames=tsfs.read_input(a.input);
physics=[];traces=[];shapes=[];geometry=[];
for k=1:numel(frames)
 for j=1:2
  out=d.runs{j,2,k};q=out.diagnostics;e=out.mechanics.evaluation;g=out.geometry;sh=e.shape;
  physics=[physics;struct('mu',d.mus(j),'angleDeg',t.angleDeg(k),'gapMm',max(abs(q.gapMm)),'coneExcessN',max(q.coneExcessN),'normalProductNmm',max(abs(q.normalProductNmm)),'frictionWorkNmm',sum(q.frictionWorkNmm),'slidingLawErrorN',max(q.slidingLawErrorN),'contactPointMismatchMm',max(q.contactPointMismatchMm),'slipMm',norm(e.slip),'normalForceN',sh.x(7),'frictionForceN',norm(sh.x(8:9)),'flags',strjoin(q.flags,';'))];
  for h=1:numel(out.mechanics.trace)
   tr=out.mechanics.trace(h);traces=[traces;struct('mu',d.mus(j),'angleDeg',t.angleDeg(k),'iteration',tr.iteration,'objective',tr.objective,'equalityInf',tr.eqInf,'inequalityInf',tr.inequalityInf,'stationarity',tr.stationarity,'alpha',tr.alpha,'accepted',tr.accepted)];
  end
 end
 out=d.runs{1,2,k};g=out.geometry;sh=out.mechanics.evaluation.shape;z=d.runs{2,2,k}.mechanics.evaluation.shape;ge=g.kinematicShape.p-t.p(:,:,k);
 ttrue=reshape(t.R(:,3,:,k),3,[]);tapprox=g.kinematicShape.t;angles=atan2(vecnorm(cross(ttrue,tapprox,1)),sum(ttrue.*tapprox,1));
 geometry=[geometry;struct('angleDeg',t.angleDeg(k),'shapeRmsMm',sqrt(mean(sum(ge.^2,1))),'shapeMaxMm',max(vecnorm(ge)),'shapeTipMm',norm(ge(:,end)),'tangentRmsDeg',rad2deg(sqrt(mean(angles.^2))),'tangentMaxDeg',rad2deg(max(angles)), ...
 'arcErrorMm',g.s-t.contactS(k),'normalErrorDeg',rad2deg(acos(max(-1,min(1,g.normal'*t.planeNormal)))),'contactPointErrorMm',norm(g.predictedPoint-t.shapes{k}.contactPoints),'planeOffsetMm',g.offset,'geometryExitFlag',g.exitflag,'geometryIterations',g.iterations,'muGeometryDifference',norm(g.s-d.runs{2,2,k}.geometry.s)+norm(g.normal-d.runs{2,2,k}.geometry.normal))];
 if k==numel(frames)
  for i=1:numel(t.sMm)
   shapes=[shapes;struct('sMm',t.sMm(i),'trueX',t.p(1,i,k),'trueY',t.p(2,i,k),'trueZ',t.p(3,i,k),'oursX',sh.p(1,i),'oursY',sh.p(2,i),'oursZ',sh.p(3,i),'zeroX',z.p(1,i),'zeroY',z.p(2,i),'zeroZ',z.p(3,i),'aloiX',b.aloi.p(1,i,k),'aloiY',b.aloi.p(2,i,k),'aloiZ',b.aloi.p(3,i,k),'stage1X',g.kinematicShape.p(1,i),'stage1Y',g.kinematicShape.p(2,i),'stage1Z',g.kinematicShape.p(3,i),'trueTwist',t.u(3,i,k),'oursTwist',sh.u(3,i),'zeroTwist',z.u(3,i))];
  end
 end
end
writetable(struct2table(physics),fullfile(folder,'physics.csv'));writetable(struct2table(traces),fullfile(folder,'iteration_trace.csv'));writetable(struct2table(shapes),fullfile(folder,'final_shape.csv'));writetable(struct2table(geometry),fullfile(folder,'stage1_accuracy.csv'));
% Discretization attribution: same 24 observations, finer integration only.
refinement=[];fr=frames{end};s=fr.tube.s;
for step=[1 .5 .25 .1]
 ff=fr;ff.tube.s=unique([s s(1):step:s(end)]);ff.tube.uhat=interp1(s,fr.tube.uhat',ff.tube.s,'previous')';rr=tsfs.reconstruct(ff,false);[~,ix]=ismember(s,rr.s);err=rr.p(:,ix)-t.p(:,:,end);
 refinement=[refinement;struct('integrationStepMm',step,'sensorCount',numel(fr.arcs),'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipMm',norm(err(:,end)))];
end
writetable(struct2table(refinement),fullfile(folder,'stage1_integration_refinement.csv'));
% The baseline consumes exactly the same reconstructed sensor positions.
m=tsfs.bishop_baseline_measurements(a.input);maxBaselineDifference=0;
for k=1:numel(frames),sh=tsfs.reconstruct(frames{k},false);maxBaselineDifference=max(maxBaselineDifference,max(abs(m.pSparse(:,:,k)-sh.p(:,a.input.packet.fbgIdx)),[],'all'));end
sigma=sqrt(diag(fr.CU));report=struct('sameBaselineInputMaxMm',maxBaselineDifference,'injectedNoiseStd',0,'likelihoodStdPerMm',sigma(1),'environmentCovariance',fr.CE,'observationType',fr.observationType,'baseFrameConvention','B(base)=R(base), psi(base)=0','referenceSource',t.sensorSource);
atomic_write_artifact(fullfile(folder,'observation_validation.json'),'json',report);
disp(struct2table(geometry));disp(struct2table(refinement));
end
