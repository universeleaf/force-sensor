function analyze_single_contact_mu(runFinite)
if nargin<1,runFinite=true;end
% Local sensitivities and finite coefficient perturbations, outside timing.
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'single_contact');a=load(fullfile(root,'datasets','sliding_clean.mat'));d=load(fullfile(folder,'ours.mat'));
frames=tsfs.read_input(a.sensorInput);o=tsfs.defaults;local=cell(1,numel(frames));
for k=1:numel(frames)
 out=d.runs{1,2,k};zero=d.runs{2,2,k};m=out.mechanics;sh=m.evaluation.shape;g=out.geometry;scale=m.scale;
 assert(numel(g.s)==1);A=[g.normal(:,g.plane(1)) g.B(:,:,1)];n=A(:,1);
 Ju=sh.Jprediction./scale';E=sh.JtipMoment./scale';eliminate=E(:,1:3)\E(:,4:end);
 transform=blkdiag(eye(3),A);
 G=(Ju(:,4:end)-Ju(:,1:3)*eliminate)/transform;
 Pc=reshape(sh.JcontactP(:,1,:),3,[])./scale';P=(Pc(:,4:end)-Pc(:,1:3)*eliminate)/transform;gap=n'*P;
 Pfull=reshape(sh.JcollisionP,[],numel(scale))./scale';Pfull=(Pfull(:,4:end)-Pfull(:,1:3)*eliminate)/transform;
 singular=svd(G);[~,~,V]=svd(G,0);weak=V(:,end);
 Jc=m.evaluation.Jc./scale';Cforce=(Jc(4:end,4:end)-Jc(4:end,1:3)*eliminate)/transform;
 feasibleBasis=null(Cforce);feasibleSv=svd(G*feasibleBasis);
 fc=out.contactForce(:,1);tangent=fc-n*(n'*fc);removed=-tangent;
 % Contact loses its tangential force. Adjust tip XYZ and contact normal,
 % while preserving terminal equilibrium (eliminated above) and active gap.
 H=[G(:,1:3) G(:,4:6)*n];C=[gap(1:3) gap(4:6)*n];b=-G(:,4:6)*removed;target=-gap(4:6)*removed;
 origin=pinv(C)*target;N=null(C);comp=origin+N*((H*N)\(b-H*origin));
 delta=[comp(1:3);n*comp(4)+removed];residual=G*delta;
 shapeDelta=reshape(Pfull*delta,3,[]);
 sigma=sqrt(frames{k}.CU(1,1));
 corr=dot(G(:,1),G(:,4))/(norm(G(:,1))*norm(G(:,4)));
 local{k}=struct('frame',k,'forceOrder',{{'tipX','tipY','tipZ','contactX','contactY','contactZ'}}, ...
  'singularValuesPerMmPerN',singular,'condition',singular(1)/singular(end),'weakForceDirection',weak, ...
  'feasibleForceTangentDimension',size(feasibleBasis,2),'feasibleForceSingularValues',feasibleSv,'feasibleForceCondition',feasibleSv(1)/feasibleSv(end), ...
  'tipXContactXCorrelation',corr,'contactTangentRemovedN',removed,'linearTipChangeN',comp(1:3),'linearNormalChangeN',comp(4), ...
  'linearResidualFbgRms',sqrt(mean((residual/sigma).^2)),'curvatureMismatchReductionFactor',norm(G(:,4:6)*removed)/max(norm(residual),eps), ...
  'linearGapChangeMm',gap*delta,'linearShapeChangeRmsMm',sqrt(mean(sum(shapeDelta.^2,1))), ...
  'actualTipChangeN',zero.tipForce-out.tipForce,'actualContactChangeN',zero.contactForce-out.contactForce, ...
  'actualShapeChangeRmsMm',sqrt(mean(sum((zero.mechanics.evaluation.shape.collisionP-sh.collisionP).^2,1))), ...
  'stage1ArcDifferenceMm',zero.geometry.s-out.geometry.s,'stage1NormalDifference',norm(zero.geometry.normal-g.normal), ...
  'scope','Shape-change RMS uses the same collision mesh for linear and nonlinear differences. Local FBG-only force sensitivity; terminal moment eliminated, normal gap held for compensation. Linear prediction from removing the full tangential force is a finite-change approximation, not an exact nonlinear solution.');
end
atomic_write_artifact(fullfile(folder,'local_sensitivity.json'),'json',local);
if ~runFinite
 if isfile(fullfile(folder,'sensitivity.mat')),save(fullfile(folder,'sensitivity.mat'),'local','-append');end
 return
end
% Matched sequential runs at +/-0.01; both reset tracking at frame 1.
values=[0.29 0.31];finite=cell(2,numel(frames));rows=[];
for j=1:2
 previous=[];
 for k=1:numel(frames)
  fr=frames{k};fr.mu(:)=values(j);fr.previous.mu(:)=values(j);
  out=tsfs.step(fr,previous,o);previous=out;
  finite{j,k}=struct('tip',out.tipForce,'contact',out.contactForce,'p',out.mechanics.evaluation.shape.p,'converged',out.mechanics.converged,'flags',{out.diagnostics.flags});
 end
end
for k=1:numel(frames)
 dm=finite{2,k};dp=finite{1,k};df=(dm.contact-dp.contact)/0.02;dt=(dm.tip-dp.tip)/0.02;
 rows=[rows;struct('frame',k,'dTipX_dMu',dt(1),'dTipY_dMu',dt(2),'dTipZ_dMu',dt(3),'dContactX_dMu',df(1),'dContactY_dMu',df(2),'dContactZ_dMu',df(3), ...
  'shapeDerivativeRmsMmPerMu',sqrt(mean(sum(((dm.p-dp.p)/0.02).^2,1))),'bothConverged',dm.converged&&dp.converged,'flagsMinus',strjoin(dp.flags,';'),'flagsPlus',strjoin(dm.flags,';'))];
end
writetable(struct2table(rows),fullfile(folder,'mu_sensitivity.csv'));save(fullfile(folder,'sensitivity.mat'),'local','finite','rows','values','-v7.3');
fprintf('Sensitivity analysis complete.\n');
end
