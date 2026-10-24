function result=run_robust_trajectory(source,tag,cfg,options)
% Recurrent replay of one physical trajectory with any single-pass solver.
% Same per-frame logic as run_inclined_modes (geometry -> solver ->
% diagnostics -> history), but outputs stay under results/runs/<tag>/.
% options.initialEstimate restores a saved state as frame 1 (rotation);
% options.keepFrames lists frames whose full output is saved for restores.
if nargin<4,options=struct;end
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs',tag);
if isfile(fullfile(folder,'result.mat')),fprintf('SKIP COMPLETE %s\n',tag);result=load(fullfile(folder,'result.mat'));return,end
if ~isfolder(folder),mkdir(folder);end
a=load(source,'input','truth');frames=tsfs.read_input(a.input);t=a.truth;T=numel(frames);
if isfield(options,'maxFrames'),T=min(T,options.maxFrames);end
o=tsfs.defaults;o.gapTolerance=1e-6;o.momentTolerance=200e-6;mu=cfg.estimatorMus;
if isfield(cfg,'optionOverrides'),f=fieldnames(cfg.optionOverrides);for j=1:numel(f),o.(f{j})=cfg.optionOverrides.(f{j});end,end
keep=[];if isfield(options,'keepFrames'),keep=options.keepFrames;end
previous=[];restore=isfield(options,'initialEstimate')&&~isempty(options.initialEstimate);if restore,previous=options.initialEstimate;end
rows=[];kept=struct('frame',{},'out',{});shapes=zeros(3,numel(t.sMm),T);forces=zeros(6,T);branchRows=[];
for k=1:T
 fr=frames{k};fr.mu(:)=mu;fr.previous.mu(:)=mu;fr.CU=eye(numel(fr.u))*cfg.sensorStd^2;fr.previous.CU=fr.CU;clock=tic;
 failure='';
 if k==1&&restore
  out=previous;out.time=fr.time;out.seconds=0;out.restored=true;g=out.geometry;sol=out.mechanics;d=out.diagnostics;
 else
  try
   g=tsfs.geometry(fr,o);sol=feval(cfg.solverFunction,fr,g,previous,o,cfg);
   assert(all(isfinite(sol.evaluation.shape.p(:)))&&all(isfinite(sol.evaluation.shape.x(:))),'tsfs:NonFiniteState','Returned state is not finite.');
   d=tsfs.diagnose(fr,g,sol,o);
   out=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sol,'diagnostics',d,'contactForce',sol.evaluation.shape.force,'tipForce',sol.evaluation.shape.x(4:6),'seconds',toc(clock),'restored',false);
  catch err
   % Same policy as tsfs.estimate: the last valid output stays the history
   % (its older time makes the next frame use Stage 1 history); the held
   % estimate is scored for this frame and the failure is recorded.
   assert(~isempty(previous),'tsfs:FirstFrameFailed','%s',err.message);failure=[err.identifier ': ' err.message];
   out=previous;out.seconds=toc(clock);out.restored=false;g=out.geometry;sol=out.mechanics;d=out.diagnostics;
  end
 end
 previous=out;sh=sol.evaluation.shape;fc=sum(sh.force,2);ft=sh.x(4:6);err=sh.p-t.p(:,:,k);
 shapes(:,:,k)=sh.p;forces(:,k)=[ft;fc];
 bvp=struct('converged',NaN,'rejected',false,'failed',false,'finalNodes',NaN);if isfield(sol,'bvpCorrection'),b=sol.bvpCorrection;bvp=struct('converged',b.converged,'rejected',isfield(b,'rejected')&&b.rejected,'failed',b.failed,'finalNodes',b.finalNodes);end
 usedPrior=isfield(cfg,'forcePriors')&&cfg.forcePriors;
 angle=NaN;if isfield(t,'angleDeg'),angle=t.angleDeg(k);end
 sc=NaN;if numel(g.s)==1,sc=g.s;end
 preFbg=NaN;preMoment=NaN;if isfield(sol,'beforeBvp'),preFbg=sol.beforeBvp.fbgRms;preMoment=sol.beforeBvp.tipMomentNmm;end
 row=struct('frame',k,'restored',k==1&&restore,'failed',~isempty(failure),'failure',string(failure),'distanceMm',t.distanceMm(k),'angleDeg',angle,'seconds',out.seconds,'geometrySeconds',g.seconds,'mechanicsSeconds',sol.seconds, ...
  'mode',mat2str(sol.mode),'candidateCount',numel(g.s),'estimatedArcMm',sc,'arcErrorMm',sc-t.contactS(k), ...
  'tipErrorN',norm(ft-t.tipForce(:,k)),'contactErrorN',norm(fc-t.contactForce(:,k)),'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'shapeTipErrorMm',norm(err(:,end)), ...
  'tipFx',ft(1),'tipFy',ft(2),'tipFz',ft(3),'contactFx',fc(1),'contactFy',fc(2),'contactFz',fc(3), ...
  'trueTipFx',t.tipForce(1,k),'trueTipFy',t.tipForce(2,k),'trueTipFz',t.tipForce(3,k),'trueContactFx',t.contactForce(1,k),'trueContactFy',t.contactForce(2,k),'trueContactFz',t.contactForce(3,k), ...
  'fbgRms',d.fbgRms,'preBvpFbgRms',preFbg,'preBvpMomentNmm',preMoment,'tipMomentNmm',norm(sh.tipMoment,inf),'gapMm',max([0;abs(d.gapMm(:))]),'penetrationMm',d.maximumPenetrationMm, ...
  'affineConverged',sol.affineConverged,'modeFeasible',sol.modeFeasible,'bvpConverged',bvp.converged,'bvpRejected',bvp.rejected,'bvpFailed',bvp.failed,'bvpNodes',bvp.finalNodes, ...
  'linearizeSeconds',field(sol,'linearizeSeconds'),'branchSeconds',sum([sol.attemptSummary.seconds]),'auditSeconds',field(sol,'auditSeconds'),'bvpSeconds',field(sol.bvpCorrection,'seconds'),'finalAuditSeconds',field(sol.bvpCorrection,'finalAuditSeconds'),'diagnoseSeconds',d.seconds,'iterations',sol.iterations,'selectedIterations',sol.selectedIterations,'priorUsed',usedPrior,'tipPriorStdN',sol.prior.tipStd,'forcePriorStdN',sol.prior.forceStd,'flags',strjoin(d.flags,';'));
 rows=[rows;row];
 for ia=1:numel(sol.attemptSummary)*~row.restored*~row.failed,r=sol.attemptSummary(ia);branchRows=[branchRows;struct('frame',k,'attempt',ia,'modes',r.modes,'feasible',r.feasible,'objective',r.objective,'violation',r.violation,'iterations',r.iterations,'seconds',r.seconds,'selected',ia==sol.selectedAttempt)];end
 if ismember(k,keep),kept(end+1)=struct('frame',k,'out',compact(out));end
 fprintf('%s frame %d/%d mode%s %.2fs tip %.4g contact %.4g shape %.4g fbg %.3g bvp %d/%d\n',tag,k,T,mat2str(sol.mode),out.seconds,row.tipErrorN,row.contactErrorN,row.shapeRmsMm,row.fbgRms,bvp.converged,bvp.rejected);
end
frames=struct2table(rows,'AsArray',true);writetable(frames,fullfile(folder,'frames.csv'));
if ~isempty(branchRows),writetable(struct2table(branchRows,'AsArray',true),fullfile(folder,'branches.csv'));end
publicCfg=cfg;result=struct('tag',tag,'source',source,'configuration',publicCfg,'frames',frames,'shapes',shapes,'forces',forces,'kept',kept);
save(fullfile(folder,'result_pending.mat'),'-struct','result','-v7.3');movefile(fullfile(folder,'result_pending.mat'),fullfile(folder,'result.mat'),'f');
end

function v=field(s,name)
v=NaN;if isstruct(s)&&isfield(s,name)&&~isempty(s.(name)),v=s.(name);end
end

function out=compact(out)
% Remove dense ODE objects and per-branch states; history needs p,R,s and posterior.
out.mechanics.evaluation.shape.pieces={};out.mechanics.attempts={};
if isfield(out.mechanics,'affineEvaluation'),out.mechanics.affineEvaluation.shape.pieces={};end
out.geometry.kinematicShape.Jp=[];out.geometry.kinematicShape.Jt=[];
end
