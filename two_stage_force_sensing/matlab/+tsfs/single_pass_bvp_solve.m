function sol=single_pass_bvp_solve(fr,g,previous,o,cfg)
% Exhaust all circular modes once, then one fixed-load nonlinear BVP call.
clock=tic;sol=tsfs.single_linearization_solve(fr,g,previous,o,cfg);before=sol.evaluation;zBefore=sol.z;
sol.beforeBvp=struct('z',zBefore,'tipMomentNmm',norm(before.shape.tipMoment,inf),'gapMm',before.gap,'slipMm',before.slip,'projectionN',before.projection,'fbgRms',before.fbgRms,'objective',before.objective,'modeViolation',sol.modeViolation,'p',before.shape.p,'prediction',before.shape.prediction,'mechanicsSeconds',sol.seconds);
failed=false;exception='';auditSeconds=0;info=struct('converged',false,'seconds',0,'rhsEvaluations',0,'jacobianEvaluations',0,'boundaryEvaluations',0,'initialNodes',0,'finalNodes',0,'regions',0,'boundaryResidual',Inf,'collocationTipMomentNmm',Inf,'shapeChangeRmsMm',NaN,'rotationOrthogonality',NaN,'forceChangeN',0,'warning','','warningId','','stats',struct,'configuration',cfg.bvp);
bvpClock=tic;
try
 assert(~sol.nonlinearAuditFailed,'tsfs:BvpSeed','Nonlinear seed unavailable.');
 [z,info]=tsfs.fixed_load_bvp(fr,g,sol.z,sol.scale,before.shape,cfg.bvp);
 assert(isequal(z(4:end),zBefore(4:end)),'tsfs:BvpForceChanged','Fixed forces changed.');
 history=[];if ~isempty(fr.previous)
  if ~isempty(previous)&&previous.valid&&abs(previous.time-fr.previous.time)<1e-9,history=previous.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];else,history=tsfs.reconstruct(fr.previous,false);end
 end
 t=tic;sol.counts.augmentedIVPs=sol.counts.augmentedIVPs+1;
 e=tsfs.evaluate(fr,g,z,sol.scale,sol.prior,history,o,true);auditSeconds=toc(t);N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=.5*(e.r'*e.r);
 info.ivpBvpShapeRmsMm=sqrt(mean(sum((e.shape.p-info.p).^2,1)));info.ivpTipMomentNmm=norm(e.shape.tipMoment,inf);info.forceChangeN=norm((z(4:end)-zBefore(4:end)).*sol.scale(4:end),inf);
 sol.z=z;sol.modeY(1:3)=z(1:3);sol.evaluation=e;sol.counts.rhsEvaluations=sol.counts.rhsEvaluations+e.shape.rhsEvaluations;sol.counts.odeSegments=sol.counts.odeSegments+e.shape.segments;
 [sol.modeViolation,~]=tsfs.mode_residual(e,g,sol.mode,sol.modeY,sol.scale);sol.circularTrueViolation=sol.modeViolation;sol.modeFeasible=sol.modeViolation<=cfg.constraintTolerance;
catch err
 if info.seconds==0,info.seconds=toc(bvpClock);end
 failed=true;exception=[err.identifier ': ' err.message];info.converged=false;if sol.counts.augmentedIVPs>2,sol.counts.failedIVPs=sol.counts.failedIVPs+1;end
end
info.exception=exception;info.failed=failed;sol.converged=sol.affineConverged&&info.converged;info.finalAuditSeconds=auditSeconds;sol.bvpCorrection=info;sol.modeAccepted=sol.affineConverged&&info.converged&&sol.modeFeasible&&sol.evaluation.fbgRms<=cfg.fbgRmsAcceptance;sol.reason=['fixed-load-BVP/' sol.reason];sol.solver=['fixed-load-BVP/' sol.solver];sol.seconds=toc(clock);
end
