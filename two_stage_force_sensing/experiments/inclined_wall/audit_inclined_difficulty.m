function audit_inclined_difficulty
% Offline audit only: no estimator settings or saved results are changed.
[root,~]=setup_tsfs;o=tsfs.defaults;folder=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall');a=load(fullfile(folder,'truth.mat'));b=load(fullfile(folder,'comparison_unreduced_attempt.mat'));t=a.truth;frames=tsfs.read_input(a.input);pub=fullfile(root,'results','published','inclined_wall_difficulty_20261008');if ~isfolder(pub),mkdir(pub);end
refrows=[];traceRows=[];constraintRows=[];counter=[];checks=[];detail={};
for k=4:38
 fr=frames{k};fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;out=b.runs{1,k};sol=out.mechanics;g=out.geometry;scale=sol.scale;
 ideal=g;ideal.s=t.contactS(k);ideal.normal=t.planeNormal;ideal.offset=0;ideal.B=tsfs.basis(t.planeNormal);ideal.mu=.3;
 A=[ideal.normal ideal.B];ft=t.tipForce(:,k);fc=t.contactForce(:,k);pc=interp1(t.sMm,t.p(:,:,k)',ideal.s,'pchip')';m=cross(t.p(:,end,k)-t.p(:,1,k),ft)+cross(pc-t.p(:,1,k),fc);z=[m;ft;A'*fc]./scale;
 sh=tsfs.ivp(fr,ideal,z,scale,o,true);[G,P,E]=reduced(sh,scale,A);[~,S,V]=svd(G,0);sv=diag(S);
 old=interp1(t.sMm,t.p(:,:,k-1)',ideal.s,'pchip')';v=ideal.B'*(sh.contactP-old);R=.3*(ideal.normal'*fc);unit=v/norm(v);
 Cf=[ideal.normal'*P;[zeros(2,3) ideal.B'+.3*unit*ideal.normal']+R/norm(v)*(eye(2)-unit*unit')*ideal.B'*P];N=null(Cf);sf=svd(G*N);
 chord=sh.p(:,end)-sh.contactP;dir=chord/norm(chord);tail=sh.p(:,sh.s>=ideal.s)-sh.contactP;sag=max(vecnorm(tail-dir*(dir'*tail)));
 refrows=[refrows;struct('frame',k,'distanceMm',t.distanceMm(k),'contactArcMm',ideal.s,'normalN',ideal.normal'*fc,'slipMm',norm(v),'freeTailMm',200-ideal.s,'tailSagMm',sag,'sensorsAfterContact',sum(fr.arcs>ideal.s),'sminPerMmN',sv(end),'smaxPerMmN',sv(1),'forceCondition',sv(1)/sv(end),'weakYFraction',norm(V([2 5],end)),'weakTipChordCosine',abs(dot(V(1:3,end),dir))/norm(V(1:3,end)),'lawSminPerMmN',sf(end),'lawCondition',sf(1)/sf(end),'momentCondition',cond(E(:,1:3)),'shapeMatchMaxMm',max(vecnorm(sh.p-t.p(:,:,k))),'sensorMatchRmsPerMm',rms(sh.prediction(:)-fr.u(:)),'terminalMomentNmm',norm(sh.tipMoment,inf))];
 detail{end+1}=struct('frame',k,'G',G,'lawJacobian',Cf,'weakForceDirection',V(:,end),'singularValues',sv,'lawSingularValues',sf);
 tr=sol.trace;zs=vertcat(tr.z);steps=[NaN;vecnorm(diff(zs),2,2)];fun=vertcat(tr.funcCount);obj=vertcat(tr.objective);viol=vertcat(tr.constraintViolation);
 for it=1:numel(tr)
  xx=zs(it,1:numel(scale))'.*scale;traceRows=[traceRows;struct('frame',k,'iteration',tr(it).iteration,'objective',obj(it),'scaledObjective',obj(it)*sol.objectiveScale,'violation',viol(it),'optimality',tr(it).firstorderopt,'functionCount',fun(it),'stepNormScaled',steps(it),'seconds',tr(it).seconds,'tipNormN',norm(xx(4:6)),'lambdaN',xx(7),'tauNormN',norm(xx(8:9)),'betaMm',zs(it,end))];
 end
 [val,J,meta]=tsfs.relaxed_constraints(sol.evaluation,g,scale,sol.auxiliaryBeta,sol.epsilon,true);ng=numel(sol.evaluation.h)-1;ci=3+ng+(1:4);e=sol.evaluation;Jfr=J(ci(2:4),:);jr=vecnorm(Jfr,2,2);ss=svd(Jfr./max(jr,realmin));gapJ=[g.normal'*reshape(e.shape.JcontactP,3,[]) 0];prodJ=J(ci(1),:);xx=e.shape.x;fR=.3*xx(7);vnorm=norm(e.slip);tau=xx(8:9);
 align=NaN;if norm(tau)*vnorm>0,align=-dot(tau,e.slip)/(norm(tau)*vnorm);end
 constraintRows=[constraintRows;struct('frame',k,'normalN',xx(7),'tauN',norm(tau),'slipMm',vnorm,'betaMm',sol.auxiliaryBeta,'coneSlackN',fR-norm(tau),'betaMinusSlipMm',sol.auxiliaryBeta-vnorm,'antislipCosine',align,'normalProductNmm',meta.normalProduct,'frictionGapNmm',meta.frictionGap,'gapMm',e.gap,'normalProductJacNorm',norm(prodJ),'gapJacNorm',norm(gapJ),'forceConeJacNorm',jr(1),'slipConeJacNorm',jr(2),'frictionGapJacNorm',jr(3),'normalizedFrictionJacSmin',ss(end),'gapProductCosine',abs(dot(prodJ,gapJ))/(norm(prodJ)*norm(gapJ)),'relaxedViolation',meta.violation)];
 if ismember(k,[4 6 7 11 15 21 23 27 34 37])
  % Evaluate known reference loads with estimated frozen geometry. Only solve
  % base moment to enforce terminal equilibrium; this is not an estimator rerun.
  Ag=[g.normal g.B];zg=[m;ft;Ag'*fc]./scale;
  for it=1:8
   sg=tsfs.ivp(fr,g,zg,scale,o,true);if norm(sg.tipMoment,inf)<1e-9,break,end
   zg(1:3)=zg(1:3)-sg.JtipMoment(:,1:3)\sg.tipMoment;
  end
  ht=struct('s',t.sMm,'p',t.p(:,:,k-1),'t',zeros(3,numel(t.sMm)),'Jp',[],'Jt',[]);hp=b.runs{1,k-1}.mechanics.evaluation.shape;hp.t=reshape(hp.R(:,3,:),3,[]);hp.Jp=[];hp.Jt=[];
  for ih=1:2
   if ih==1,history=ht;tag='true-history';else,history=hp;tag='estimated-history';end
   eg=tsfs.evaluate(fr,g,zg,scale,sol.prior,history,o,true);eg.relaxedHasHistory=true;beta=norm(eg.slip);[vv,~,mm]=tsfs.relaxed_constraints(eg,g,scale,beta,1e-6,true);ru=eg.r(1:numel(fr.u));
   counter=[counter;struct('frame',k,'history',tag,'sensorCost',.5*(ru'*ru),'scaledSensorCost',1e-4*.5*(ru'*ru),'sensorRmsPerMm',rms(eg.shape.prediction(:)-fr.u(:)),'shapeRmsMm',sqrt(mean(sum((eg.shape.p-t.p(:,:,k)).^2,1))),'gapMm',eg.gap,'slipMm',norm(eg.slip),'slipFirst',eg.slip(1),'slipSecond',eg.slip(2),'normalProductNmm',mm.normalProduct,'frictionGapNmm',mm.frictionGap,'relaxedViolation',mm.violation,'terminalMomentNmm',norm(eg.shape.tipMoment,inf),'historyPointErrorMm',norm(tsfs.shape_at(history,g.s)-tsfs.shape_at(ht,g.s)))];
  end
 end
 if ismember(k,[4 15 21 34])
  dirf=V(:,end);h=1e-3;pred=zeros(numel(fr.u),2);
  for j=1:2
   zz=z;zz(4:9)=zz(4:9)+(2*j-3)*h*blkdiag(eye(3),A')*dirf;
   for it=1:8
    shc=tsfs.ivp(fr,ideal,zz,scale,o,true);if norm(shc.tipMoment,inf)<1e-10,break,end;zz(1:3)=zz(1:3)-shc.JtipMoment(:,1:3)\shc.tipMoment;
   end
   pred(:,j)=shc.prediction(:);
  end
  fd=(pred(:,2)-pred(:,1))/(2*h);checks=[checks;struct('frame',k,'relativeError',norm(fd-G*dirf)/norm(G*dirf))];
 end
 fprintf('AUDIT frame %d cond %.1f constrained %.1f friction J min %.3g\n',k,sv(1)/sv(end),sf(1)/sf(end),ss(end));
end
writetable(struct2table(refrows),fullfile(pub,'reference_sensitivity.csv'));writetable(struct2table(traceRows),fullfile(pub,'convergence_traces.csv'));writetable(struct2table(constraintRows),fullfile(pub,'constraint_geometry.csv'));writetable(struct2table(counter),fullfile(pub,'reference_load_counterfactuals.csv'));writetable(struct2table(checks),fullfile(pub,'derivative_checks.csv'));atomic_write_artifact(fullfile(pub,'sensitivity_matrices.json'),'json',detail);
assert(max([checks.relativeError])<1e-3,'Sensitivity finite difference check failed');
fprintf('Done: max derivative relative error %.3g\n',max([checks.relativeError]));
end
function [G,P,E]=reduced(sh,scale,A)
J=sh.Jprediction./scale';E=sh.JtipMoment./scale';M=E(:,1:3)\E(:,4:end);T=blkdiag(eye(3),A);G=(J(:,4:end)-J(:,1:3)*M)/T;J=reshape(sh.JcontactP(:,1,:),3,[])./scale';P=(J(:,4:end)-J(:,1:3)*M)/T;
end
