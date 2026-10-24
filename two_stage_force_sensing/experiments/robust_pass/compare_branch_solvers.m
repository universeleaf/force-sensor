function result=compare_branch_solvers(source,tag,options)
% Shadow comparison on identical inputs. The recurrent trajectory follows
% the reference estimator (fmincon SQP branches + fixed-load BVP); at every
% frame the profiled-QP branch solver is run on the same frame, geometry,
% previous state, initializer and rod linearization, and is never propagated.
if nargin<3,options=struct;end
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs',tag);if ~isfolder(folder),mkdir(folder);end
a=load(source,'input','truth','sensorStd');frames=tsfs.read_input(a.input);T=numel(frames);if isfield(options,'maxFrames'),T=min(T,options.maxFrames);end
o=tsfs.defaults;o.gapTolerance=1e-6;o.momentTolerance=200e-6;ref=robust_configurations('reference',a.sensorStd);qp=robust_configurations('qp',a.sensorStd);
previous=[];if isfield(options,'initialEstimate'),previous=options.initialEstimate;end;first=1+isfield(options,'initialEstimate');
rows=[];frameRows=[];
for k=first:T
 fr=frames{k};fr.mu(:)=ref.estimatorMus;fr.previous.mu(:)=ref.estimatorMus;fr.CU=eye(numel(fr.u))*a.sensorStd^2;fr.previous.CU=fr.CU;
 g=tsfs.geometry(fr,o);
 sR=tsfs.single_pass_bvp_solve(fr,g,previous,o,ref);sQ=tsfs.single_linearization_solve(fr,g,previous,o,qp);
 assert(norm(sR.initialZ-sQ.initialZ,inf)<1e-12,'Expansion points differ.');
 R=sR.attemptSummary;Q=sQ.attemptSummary;
 for j=1:numel(R)
  m=find(strcmp({Q.modes},R(j).modes),1);q=Q(m);
  rows=[rows;struct('frame',k,'modes',R(j).modes,'sqpFeasible',R(j).feasible,'qpFeasible',q.feasible,'sqpObjective',R(j).objective,'qpObjective',q.objective, ...
   'qpMinusSqp',q.objective-R(j).objective,'sqpViolation',R(j).violation,'qpViolation',q.violation,'sqpIterations',R(j).iterations,'sqpAtCap',R(j).iterations>=ref.maxIterationsPerAssignment, ...
   'sqpExit',string(R(j).reason),'qpCount',q.iterations,'qpCertificate',sQ.attempts{m}.certificate,'sqpSeconds',R(j).seconds,'qpSeconds',q.seconds,'sqpSelected',j==sR.selectedAttempt,'qpSelected',m==sQ.selectedAttempt)];
 end
 xR=sR.beforeBvp.z.*sR.scale;xQ=sQ.z.*sQ.scale;
 frameRows=[frameRows;struct('frame',k,'sqpMode',mat2str(sR.mode),'qpMode',mat2str(sQ.mode),'sameMode',isequal(sR.mode,sQ.mode),'sqpObjective',R(sR.selectedAttempt).objective,'qpObjective',Q(sQ.selectedAttempt).objective, ...
  'forceDifferenceN',norm(xR(4:end)-xQ(4:end)),'sqpBranchSeconds',sum([R.seconds]),'qpBranchSeconds',sum([Q.seconds]),'qpCertificate',all(cellfun(@(a)a.certificate,sQ.attempts)))];
 sh=sR.evaluation.shape;previous=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sR,'diagnostics',[],'contactForce',sh.force,'tipForce',sh.x(4:6),'seconds',0);
 fprintf('%s frame %d/%d sqp%s qp%s dObj %.3g dF %.3g time %.3f/%.3f\n',tag,k,T,mat2str(sR.mode),mat2str(sQ.mode),frameRows(end).qpObjective-frameRows(end).sqpObjective,frameRows(end).forceDifferenceN,frameRows(end).sqpBranchSeconds,frameRows(end).qpBranchSeconds);
end
branches=struct2table(rows,'AsArray',true);framesT=struct2table(frameRows,'AsArray',true);writetable(branches,fullfile(folder,'branches.csv'));writetable(framesT,fullfile(folder,'frames.csv'));
result=struct('branches',branches,'frames',framesT,'previous',previous);save(fullfile(folder,'result.mat'),'-struct','result','-v7.3');
end
