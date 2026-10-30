function rows=compare_multicontact(names,tag,options)
% Multi-contact / frictionless check of tsfs.qp_branch on the recorded
% fixtures (material-curvature packets, 2 planes, 2-3 contacts). Two
% recurrent chains (reference fmincon branches vs profiled QP, both with the
% single fixed-load BVP), plus a branch-by-branch comparison of both
% optimizers on the reference chain's inputs. Scoring follows run_benchmarks.
% options.qpOnly=true runs only the QP chain (the reference chain does not
% depend on qp_branch, so an earlier reference run can be reused).
if nargin<1||isempty(names),names={'two_contact','three_contact','spatial_sliding','spatial_sliding_noisy','spatial_three_channel','s_channel_two_contact','sliding_clean'};end
if nargin<2,tag='multicontact_check';end
if nargin<3,options=struct;end
qpOnly=isfield(options,'qpOnly')&&options.qpOnly;
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs',tag);if ~isfolder(folder),mkdir(folder);end
o=tsfs.defaults;o.gapTolerance=1e-6;o.momentTolerance=200e-6;rows=[];branchRows=[];
for j=1:numel(names)
 name=names{j};file=fullfile(root,'datasets',[name '.mat']);
 if isfile(file),a=load(file);t=a.truth;else,a=load(fullfile(root,'datasets',name,'input.mat'));b=load(fullfile(root,'datasets',name,'truth.mat'));t=b.truth;end
 frames=tsfs.read_input(a.sensorInput);std=sqrt(frames{1}.CU(1,1));
 cfg={robust_configurations('reference',std),robust_configurations('qp',std)};previous={[],[]};
 for k=1:numel(frames)
  fr=frames{k};out=cell(1,2);
  for m=1:2
   clock=tic;
   if m==1&&qpOnly,out{m}=struct('valid',false,'error','skipped','seconds',0);continue,end
   try
    g=tsfs.geometry(fr,o);sol=tsfs.single_pass_bvp_solve(fr,g,previous{m},o,cfg{m});d=tsfs.diagnose(fr,g,sol,o);
    out{m}=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sol,'diagnostics',d,'contactForce',sol.evaluation.shape.force,'tipForce',sol.evaluation.shape.x(4:6),'seconds',toc(clock));previous{m}=out{m};
   catch err
    out{m}=struct('valid',false,'error',[err.identifier ': ' err.message],'seconds',toc(clock));
   end
  end
  % Branch comparison on the reference chain's inputs (same g, previous, linearization).
  if out{1}.valid
   g=out{1}.geometry;prevRef=[];if k>1&&rows(end).referenceValid,prevRef=saved;end
   sQ=tsfs.single_linearization_solve(fr,g,prevRef,o,cfg{2});R=out{1}.mechanics.attemptSummary;Q=sQ.attemptSummary;
   for i=1:numel(R)
    q=Q(find(strcmp({Q.modes},R(i).modes),1));
    branchRows=[branchRows;struct('dataset',string(name),'frame',k,'modes',string(R(i).modes),'sqpFeasible',R(i).feasible,'qpFeasible',q.feasible,'sqpObjective',R(i).objective,'qpObjective',q.objective,'qpMinusSqp',q.objective-R(i).objective,'sqpIterations',R(i).iterations,'sqpSeconds',R(i).seconds,'qpSeconds',q.seconds)];
   end
   saved=out{1};
  end
  r=struct('dataset',string(name),'frame',k,'contacts',NaN,'referenceValid',out{1}.valid,'qpValid',out{2}.valid,'referenceMode',"",'qpMode',"",'referenceTipErrorN',NaN,'qpTipErrorN',NaN,'referenceContactErrorN',NaN,'qpContactErrorN',NaN,'referenceShapeMm',NaN,'qpShapeMm',NaN,'referenceFbg',NaN,'qpFbg',NaN,'referenceSeconds',out{1}.seconds,'qpSeconds',out{2}.seconds,'qpCertificate',false,'error',"");
  labels={'reference','qp'};
  for m=1:2
   if ~out{m}.valid,if ~(m==1&&qpOnly),r.error=string(out{m}.error);end,continue,end
   s=score(out{m},t,k);r.contacts=numel(out{m}.geometry.s);r.([labels{m} 'Mode'])=string(mat2str(out{m}.mechanics.mode));
   r.([labels{m} 'TipErrorN'])=s.tip;r.([labels{m} 'ContactErrorN'])=s.contact;r.([labels{m} 'ShapeMm'])=s.shape;r.([labels{m} 'Fbg'])=out{m}.diagnostics.fbgRms;
  end
  if out{2}.valid,r.qpCertificate=all(cellfun(@(x)x.certificate,out{2}.mechanics.attempts));end
  rows=[rows;r];
  fprintf('%s frame %d C=%g ref %s tip %.4g contact %.4g | qp %s tip %.4g contact %.4g | %.2fs/%.2fs %s\n',name,k,r.contacts,r.referenceMode,r.referenceTipErrorN,r.referenceContactErrorN,r.qpMode,r.qpTipErrorN,r.qpContactErrorN,r.referenceSeconds,r.qpSeconds,r.error);
 end
end
writetable(struct2table(rows,'AsArray',true),fullfile(folder,'frames.csv'));if ~isempty(branchRows),writetable(struct2table(branchRows,'AsArray',true),fullfile(folder,'branches.csv'));end
end

function s=score(out,t,k)
% Same matching as run_benchmarks: contacts sorted by true arc length.
if isfield(t,'contactForces'),fc=t.contactForces(:,:,k);else,fc=t.contactForce(:,k);end
trueS=t.contactS(:,min(k,size(t.contactS,2)))';[trueS,order]=sort(trueS);fc=fc(:,order);
s.contact=NaN;if numel(trueS)==numel(out.geometry.s),s.contact=norm(out.contactForce-fc,'fro');end
s.tip=norm(out.tipForce-t.tipForce(:,k));p=out.mechanics.evaluation.shape.p;pt=t.p(:,:,k);s.shape=NaN;if isequal(size(p),size(pt)),s.shape=sqrt(mean(sum((p-pt).^2,1)));end
end
