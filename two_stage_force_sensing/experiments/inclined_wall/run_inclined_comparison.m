function run_inclined_comparison(maxFrames)
if nargin<1,maxFrames=Inf;end
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'inclined_wall');a=load(fullfile(folder,'truth.mat'));frames=tsfs.read_input(a.input);t=a.truth;o=tsfs.defaults;cfg=struct('sensorStd',1e-4,'forcePriors',false,'constraintTolerance',1e-6,'deduplicateGap',true);rows=[];runs=cell(2,min(maxFrames,numel(frames)));mus=[0.3 0];
for j=1:2
 previous=[];
 for k=1:min(maxFrames,numel(frames))
  fr=frames{k};fr.mu(:)=mus(j);fr.previous.mu(:)=mus(j);fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;
  clock=tic;g=tsfs.geometry(fr,o);sol=tsfs.relaxed_solve(fr,g,previous,o,'fmincon',cfg);d=tsfs.diagnose_relaxed(fr,g,sol,o);sh=sol.evaluation.shape;
  out=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sol,'diagnostics',d,'contactForce',sh.force,'tipForce',sh.x(4:6),'seconds',toc(clock));previous=out;fc=sum(sh.force,2);err=sh.p-t.p(:,:,k);sc=NaN;arcerr=NaN;if numel(g.s)==1,sc=g.s;arcerr=sc-t.contactS(k);end
  n=t.planeNormal;ft=sh.x(4:6);truthF=t.contactForce(:,k);normal=n'*fc;tangent=[1;0;1]'/sqrt(2)*fc;
  rows=[rows;struct('method',j,'mu',mus(j),'frame',k,'distanceMm',t.distanceMm(k),'seconds',out.seconds,'geometrySeconds',g.seconds,'mechanicsSeconds',sol.seconds,'iterations',sol.iterations,'ivps',sol.counts.augmentedIVPs,'rhsEvaluations',sol.counts.rhsEvaluations,'converged',sol.converged,'reason',sol.reason,'stationarity',sol.stationarity,'objective',sol.evaluation.objective,'objectiveTerms',numel(sol.evaluation.r),'candidateCount',numel(g.s),'activeCount',d.activeCount,'estimatedArcMm',sc,'arcErrorMm',arcerr, ...
   'normalErrorDeg',rad2deg(acos(max(-1,min(1,n'*g.normal(:,1))))),'stage1ShapeRmsMm',sqrt(mean(sum((g.kinematicShape.p-t.p(:,:,k)).^2,1))),'tipErrorN',norm(ft-t.tipForce(:,k)),'contactErrorN',norm(fc-truthF),'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipErrorMm',norm(err(:,end)), ...
   'tipFx',ft(1),'tipFy',ft(2),'tipFz',ft(3),'contactFx',fc(1),'contactFy',fc(2),'contactFz',fc(3),'normalN',normal,'tangentN',tangent,'fbgRms',d.fbgRms,'curvatureRmsPerMm',sqrt(mean((sh.prediction(:)-fr.u(:)).^2)),'forceCondition',d.forceCondition, ...
   'relaxedViolation',sol.relaxed.violation,'relaxedFeasible',d.relaxedFeasible,'gapMm',max([0;abs(d.gapMm(:))]),'projectionN',max([0;abs(d.projectionN(:))]),'coneExcessN',max([0;d.coneExcessN(:)]),'workNmm',sum(d.frictionWorkNmm),'tangency',max([0;abs(d.tangency(:))]),'penetrationMm',d.maximumPenetrationMm,'truthPlanePenetrationMm',max(0,-min(n'*(sh.collisionP-t.planePoint))),'tipMomentNmm',norm(sh.tipMoment,inf),'flags',strjoin(d.flags,';'),'exactLawFlags',strjoin(d.exactLawFlags,';'))];
  out.mechanics.evaluation.shape.pieces={};out.geometry.kinematicShape.Jp=[];out.geometry.kinematicShape.Jt=[];runs{j,k}=out;
  writetable(struct2table(rows,'AsArray',true),fullfile(folder,'estimates.csv'));save(fullfile(folder,'comparison.mat'),'runs','rows','mus','cfg','o','-v7.3');fprintf('EST mu %.1f frame %d/%d d %.0f: %.2fs %d it %d IVP C%d tip %.4f contact %.4f shape %.5f %s\n',mus(j),k,numel(frames),t.distanceMm(k),out.seconds,sol.iterations,sol.counts.augmentedIVPs,numel(g.s),rows(end).tipErrorN,rows(end).contactErrorN,rows(end).shapeRmsMm,sol.reason);
 end
end
end
