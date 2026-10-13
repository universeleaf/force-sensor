function prepare_relaxed_rotation(source)
% Reuse the validated actuation-driven mechanical trajectory; replace only sensing.
[root,paths]=setup_tsfs;assert(startsWith(getenv('TSFS_RUN_ID'),'base_rotation_relaxed_'));
folder=fullfile(paths.results,'rotation');if ~isfolder(folder),mkdir(folder);end

assert(isfile(source),'Run build_rotation_truth in the original reference run first.');a=load(source);input=a.input;truth=a.truth;
p=input.packet;N=numel(truth.sMm);T=numel(truth.angleDeg);w=zeros(2,N,T);psi=zeros(N,T);
conversionRows=[];
for k=1:T
 % Independent upstream map at a finer output grid; same loads/actuation/root.
 fine=input.tube;fine.s=unique([truth.sMm truth.sMm(1):0.1:truth.sMm(end) truth.contactS(k)]);
 fine.uhat=interp1(truth.sMm,input.tube.uhat',fine.s,'previous')';fine.T_base=truth.basePose(:,:,k);
 opts=struct('relativeTolerance',2e-10,'momentToleranceNmm',1e-5,'collisionStepMm',0.1,'maxRhsEvaluations',1000000);
 q=solve_cosserat_multi_contact_map(fine,truth.contactS(k),truth.contactForce(:,k),truth.tipForce(:,k),opts);
 [ww,pp]=tsfs.material_to_bishop(fine.s,q.u);[found,ix]=ismember(truth.sMm,fine.s);assert(all(found));
 w(:,:,k)=ww(:,ix);psi(:,k)=pp(ix)';
 conversionRows=[conversionRows;struct('angleDeg',truth.angleDeg(k),'refinedShapeMaxMm',max(vecnorm(q.p(:,ix)-truth.p(:,:,k))),'refinedCurvatureMaxPerMm',max(abs(q.u(:,ix)-truth.u(:,:,k)),[],'all'))];
end
writetable(struct2table(conversionRows),fullfile(folder,'sensor_conversion_refinement.csv'));
p.observationType='bishop-curvature';p.bishopCurvaturePerMm=w(:,p.fbgIdx,:);p.previousBishopCurvaturePerMm=p.bishopCurvaturePerMm(:,:,[1 1:T-1]);
% Native two-channel packet: do not expose a fictitious third curvature channel.
p=rmfield(p,{'curvaturePerMm','previousCurvaturePerMm'});
p.provenance='Ideal Bishop geometric sensor: w=(tangent derivative components), B(base)=R(base); independent mechanical truth converted outside estimator; noiseless.';
input.packet=p;truth.bishopW=w;truth.integratedPsi=psi;truth.sensorSource=source;
save(fullfile(folder,'truth_mu_300.mat'),'input','truth','-v7.3');
copyfile(fullfile(fileparts(source),'truth_mu_300_audit.csv'),folder);
frames=tsfs.read_input(input);rows=[];
for k=1:T
 sh=tsfs.reconstruct(frames{k},false);err=sh.p-truth.p(:,:,k);
 rows=[rows;struct('angleDeg',truth.angleDeg(k),'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'tipErrorMm',norm(err(:,end)))];
end
writetable(struct2table(rows),fullfile(folder,'sensor_reconstruction.csv'));disp(struct2table(rows));
end
