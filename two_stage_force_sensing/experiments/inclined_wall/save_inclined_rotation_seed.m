function save_inclined_rotation_seed
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs','inclined_wall_rotation_20261010');if ~isfolder(folder),mkdir(folder);end
source=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall','truth.mat');estimateSource=fullfile(root,'results','runs','inclined_wall_modes_conditional_1e6_20261010','comparison.mat');a=load(source);b=load(estimateSource,'runs');k=21;
seed=struct('frame',k,'source',source,'estimateSource',estimateSource,'input',a.input,'basePose',a.truth.basePose(:,:,k),'p',a.truth.p(:,:,k),'u',a.truth.u(:,:,k),'bishopW',a.truth.bishopW(:,:,k),'contactS',a.truth.contactS(k),'contactForce',a.truth.contactForce(:,k),'tipForce',a.truth.tipForce(:,k),'planePoint',a.truth.planePoint,'planeNormal',a.truth.planeNormal,'distanceMm',a.truth.distanceMm(k),'mu',a.truth.mu,'referenceShape',a.truth.shapes{k},'estimates',{{b.runs{1,k},b.runs{2,k}}});
save(fullfile(folder,'maximum_insertion_seed.mat'),'seed','-v7.3');
tube=a.input.tube;tube.T_base=seed.basePose;opts=struct('relativeTolerance',2e-9,'momentToleranceNmm',1e-5,'collisionStepMm',0.1,'maxRhsEvaluations',1000000);
clock=tic;q=solve_cosserat_multi_contact_map(tube,seed.contactS,seed.contactForce,seed.tipForce,opts);audit=struct('seconds',toc(clock),'shapeMaxMm',max(vecnorm(q.p-seed.p)),'curvatureMaxPerMm',max(abs(q.u-seed.u),[],'all'),'gapMm',seed.planeNormal'*(q.contactPoints-seed.planePoint),'tangency',seed.planeNormal'*q.contactTangents,'tipMomentNmm',q.tipMomentResidualNmm);
assert(audit.shapeMaxMm<1e-3&&abs(audit.gapMm)<2e-5&&abs(audit.tangency)<1e-6,'3D map disagrees with saved planar reference');
save(fullfile(folder,'seed_map_validation.mat'),'q','audit','opts','-v7.3');disp(audit);
end
