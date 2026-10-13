function audit_bishop_solutions
% Frozen-history residual audit. No production solver or benchmark is modified.
[root,paths]=setup_tsfs;src=fullfile(root,'results','runs','base_rotation_bishop_20261007','rotation');dst=paths.results;
a=load(fullfile(src,'truth_mu_300.mat'));d=load(fullfile(src,'ours.mat'));frames=tsfs.read_input(a.input);o=tsfs.defaults;rows=[];traceRows=[];cases=cell(2,2);continued=cell(2,2);
% Instrument an exact solver copy only to restart from an explicit final state
% and retain its original objective scaling. Stored in ignored diagnostic output.
code=fileread(which('tsfs.sqp'));code=strrep(code,'function sol=sqp(fr,g,previous,o)','function sol=audit_resume_sqp(fr,g,previous,o,seed,fixedScale)');
code=strrep(code,'nx=numel(z);C=numel(g.s);','z=seed;nx=numel(z);C=numel(g.s);');
code=strrep(code,'objectiveScale=1/max(1,max(diag(e.J''*e.J)));','objectiveScale=fixedScale;');
fid=fopen(fullfile(dst,'audit_resume_sqp.m'),'w');fprintf(fid,'%s',code);fclose(fid);addpath(dst);clean=onCleanup(@()rmpath(dst));
for ii=1:2
 k=[7 12];k=k(ii);fr=frames{k};prev=d.runs{1,2,k-1};warm=d.runs{1,2,k};coldOptions=o;coldOptions.bishopWarmStart=false;cold=tsfs.step(fr,prev,coldOptions);
 assert(isequaln(warm.geometry.s,cold.geometry.s)&&isequaln(warm.geometry.normal,cold.geometry.normal));
 assert(isequaln(warm.mechanics.prior,cold.mechanics.prior));
 history=prev.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];
 for j=1:2
  if j==1,out=warm;seedName='previous-material';else,out=cold;seedName='linear';end
  cases{ii,j}=out;row=score(fr,out.geometry,out.mechanics,takeTruth(a.truth,k),seedName,'original',history,o);rows=[rows;row];disp(row);
  tr=out.mechanics.trace;for h=1:numel(tr),traceRows=[traceRows;traceRow(tr(h),row.angleDeg,seedName,'original')];end
  tight=o;tight.maxIterations=300;tight.maxTrials=3000;tight.stationarityTolerance=1e-9;
  % Same ODE and feasibility tolerances. Only numerical stopping/budget change.
  sol=audit_resume_sqp(fr,out.geometry,prev,tight,out.mechanics.z,out.mechanics.objectiveScale);continued{ii,j}=sol;
  rr=score(fr,out.geometry,sol,takeTruth(a.truth,k),seedName,'tight-resume',history,tight);rows=[rows;rr];disp(rr);
  tr=sol.trace;for h=1:numel(tr),traceRows=[traceRows;traceRow(tr(h),rr.angleDeg,seedName,'tight-resume')];end
  writetable(struct2table(rows),fullfile(dst,'residual_blocks.csv'));writetable(struct2table(traceRows),fullfile(dst,'solver_trace.csv'));save(fullfile(dst,'audit.mat'),'cases','continued','rows','traceRows','o','-v7.3');
 end
end
end
function t=takeTruth(t,k)
t=struct('angle',t.angleDeg(k),'tip',t.tipForce(:,k),'contact',t.contactForce(:,k),'p',t.p(:,:,k));
end
function row=score(fr,g,sol,truth,seed,label,history,o)
e=sol.evaluation;sh=e.shape;N=numel(fr.u);r=e.r;raw=sh.prediction-fr.u;fbg=r(1:N);nt=norm(sh.x(8:9));fn=sh.x(7);tr=sol.trace(end);
% Independent multiplier fit at the RETURNED iterate, excluding trust bounds.
active=find(e.h>=-1e-8);M=[e.Jc' e.Jh(active,:)'];grad=e.J'*e.r;
opts=optimoptions('lsqlin','Display','off','OptimalityTolerance',1e-12,'StepTolerance',1e-14,'MaxIterations',500);
[lambda,~,~,flag]=lsqlin(M,-grad,[],[],[],[],[-Inf(size(e.Jc,1),1);zeros(numel(active),1)],[],[],opts);
kkt=norm(grad+M*lambda,inf);projected=null([e.Jc;e.Jh(active,:)]);pg=norm(projected'*grad);
row=struct('angleDeg',truth.angle,'seed',seed,'run',label,'objective',e.objective,'sensorCost',.5*sum(fbg.^2),'tipPriorCost',.5*sum(r(N+1:N+3).^2),'contactPriorCost',.5*sum(r(N+4:end).^2), ...
 'w1Cost',.5*sum(fbg(1:2:end).^2),'w2Cost',.5*sum(fbg(2:2:end).^2),'whitenedSensorRms',e.fbgRms,'rawSensorRmsPerMm',sqrt(mean(raw(:).^2)),'rawSensorMaxPerMm',max(abs(raw),[],'all'), ...
 'tipErrorN',norm(sh.x(4:6)-truth.tip),'contactErrorN',norm(sh.force-truth.contact),'shapeRmsMm',sqrt(mean(sum((sh.p-truth.p).^2,1))), ...
 'tipX',sh.x(4),'tipY',sh.x(5),'tipZ',sh.x(6),'contactX',sh.force(1),'contactY',sh.force(2),'contactZ',sh.force(3),'baseMomentX',sh.x(1),'baseMomentY',sh.x(2),'baseMomentZ',sh.x(3), ...
 'tipMomentInfNmm',norm(sh.tipMoment,inf),'gapMm',e.gap,'normalProjectionN',abs(e.projection(1)),'tangentProjectionN',norm(e.projection(2:3),inf),'penetrationMm',max([0;e.h(1:end-1)]),'coneExcessN',max(0,nt-g.mu*fn),'coneMarginN',g.mu*fn-nt,'normalProductNmm',abs(fn*e.gap),'slipMm',norm(e.slip),'frictionWorkNmm',sum(e.work),'normalForceN',fn,'frictionForceN',nt,'branch',e.branches{1}, ...
 'converged',sol.converged,'reason',sol.reason,'iterations',sol.iterations,'ivps',sol.counts.stateIVPs+sol.counts.augmentedIVPs,'objectiveScale',sol.objectiveScale,'reportedStationarity',sol.stationarity,'stationarityTolerance',o.stationarityTolerance,'unscaledToleranceEquivalent',o.stationarityTolerance/sol.objectiveScale,'fittedUnscaledKktInf',kkt,'fittedScaledKktInf',kkt*sol.objectiveScale,'kktFitExitFlag',flag,'feasibleProjectedGradientNorm',pg,'tangentDimension',size(projected,2),'lastTraceObjective',tr.objective,'lastTraceStepInf',tr.stepInf,'lastTraceAlpha',tr.alpha,'lastTraceDamping',tr.damping,'lastTraceTrustRadius',tr.trustRadius);
end
function row=traceRow(tr,angle,seed,label)
row=struct('angleDeg',angle,'seed',seed,'run',label,'iteration',tr.iteration,'objective',tr.objective,'scaledObjective',tr.scaledObjective,'equalityInf',tr.eqInf,'inequalityInf',tr.inequalityInf,'stationarity',tr.stationarity,'stepInf',tr.stepInf,'elasticInf',tr.elasticInf,'trustRadius',tr.trustRadius,'damping',tr.damping,'penalty',tr.penalty,'qpExit',tr.qpExit,'alpha',tr.alpha,'accepted',tr.accepted);
end
