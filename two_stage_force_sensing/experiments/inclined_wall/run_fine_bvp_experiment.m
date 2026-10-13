function run_fine_bvp_experiment(stamp,phase)
% Five-times-finer physical trajectory, same one-pass/BVP estimator settings.
if nargin<1,stamp='20261012_step5';end;if nargin<2,phase='all';end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));insertionFolder=fullfile(root,'results','runs',['fine_bvp_insertion_truth_' stamp]);rotationFolder=fullfile(root,'results','runs',['fine_bvp_rotation_truth_' stamp]);inputFolder=fullfile(root,'results','runs',['fine_bvp_inputs_' stamp]);
if strcmp(phase,'all')||strcmp(phase,'truth')
 if ~isfile(fullfile(insertionFolder,'truth.mat')),build_inclined_truth(struct('stepMm',.4,'folder',insertionFolder));end
 if ~isfolder(rotationFolder),mkdir(rotationFolder);end
 if ~isfile(fullfile(rotationFolder,'maximum_insertion_seed.mat'))
  a=load(fullfile(insertionFolder,'truth.mat'));k=101;seed=struct('frame',k,'source',fullfile(insertionFolder,'truth.mat'),'input',a.input,'basePose',a.truth.basePose(:,:,k),'p',a.truth.p(:,:,k),'u',a.truth.u(:,:,k),'bishopW',a.truth.bishopW(:,:,k),'contactS',a.truth.contactS(k),'contactForce',a.truth.contactForce(:,k),'tipForce',a.truth.tipForce(:,k),'planePoint',a.truth.planePoint,'planeNormal',a.truth.planeNormal,'distanceMm',a.truth.distanceMm(k),'mu',a.truth.mu,'referenceShape',a.truth.shapes{k},'timeSeconds',a.input.packet.timeSeconds(k));
  tube=a.input.tube;tube.T_base=seed.basePose;opts=struct('relativeTolerance',2e-9,'momentToleranceNmm',1e-5,'collisionStepMm',.1,'maxRhsEvaluations',1000000);q=solve_cosserat_multi_contact_map(tube,seed.contactS,seed.contactForce,seed.tipForce,opts);
  assert(max(vecnorm(q.p-seed.p))<1e-3&&abs(seed.planeNormal'*(q.contactPoints-seed.planePoint))<2e-5,'Fine insertion seed validation failed.');
  save(fullfile(rotationFolder,'maximum_insertion_seed.mat'),'seed','-v7.3');save(fullfile(rotationFolder,'seed_map_validation.mat'),'q','opts','-v7.3');
 end
 if ~isfile(fullfile(rotationFolder,'truth.mat')),build_inclined_rotation_truth(Inf,struct('stepDeg',.75,'folder',rotationFolder));end
end
if strcmp(phase,'truth'),return,end
noise=load(fullfile(inputFolder,'noise_payload.mat'));
for target=[0 .5 1 2]
 level=strrep(num2str(target),'.','p');sigma=noise.sigma2*target/2;sensorStd=sigma;if target==0,sensorStd=1e-4;end
 for scenario={'insertion','rotation'}
  name=scenario{1};source=fullfile(inputFolder,[name '_' level 'mm.mat']);if isfile(source),continue,end
  folder=insertionFolder;if strcmp(name,'rotation'),folder=rotationFolder;end;a=load(fullfile(folder,'truth.mat'),'input','truth');input=a.input;truth=a.truth;clean=input.packet.bishopCurvaturePerMm;
  input.packet.bishopCurvaturePerMm=clean+sigma*noise.([name 'StandardNoise']);T=size(clean,3);input.packet.previousBishopCurvaturePerMm=input.packet.bishopCurvaturePerMm(:,:,[1 1:T-1]);input.packet.curvatureNoiseStdPerMm=sensorStd;input.config.forceSensor.measurementStd.curvature=sensorStd;input.packet.provenance=sprintf('Refinement factor 5; independent Gaussian Bishop noise, original draws at coarse frames; sigma %.12g',sigma);save(source,'input','truth','sigma','sensorStd','target','-v7.3');
 end
 cfg=linearized_mode_settings;cfg.modeSearch='exhaustive';cfg.allAssignments=true;cfg.fbgRmsAcceptance=1.24;cfg.sensorStd=sensorStd;cfg.exactCircularFriction=true;cfg.estimatorMus=.3;cfg.reportFrameStart=true;cfg.targetNoiseTipRmsMm=target;cfg.maxIterationsPerAssignment=100;cfg.solverFunction='tsfs.single_pass_bvp_solve';cfg.linearization.initialRadius=Inf;cfg.singlePassMethod='full-cone-sqp';cfg.selectionPolicy='lowest affine-feasible objective, then one fixed-load BVP; no reselection';cfg.refinementFactor=5;cfg.checkpointEvery=10;cfg.noiseProtocol='original seed33001 at common frames, added independent seed33005 samples';cfg.bvp=struct('relativeTolerance',1e-7,'absoluteTolerance',1e-9,'maxMeshNodes',4000,'meshStepMm',5,'loadPolicy','all forces fixed, base moment free; contact diagnostics only');
 for scenario={'insertion','rotation'}
  name=scenario{1};tag=sprintf('fine_bvp_%s_%smm_%s',name,level,stamp);path=fullfile(root,'results','runs',tag,'comparison.mat');T=201;if strcmp(name,'rotation'),T=121;end
  if isfile(path),saved=load(path,'runs');if size(saved.runs,2)==T&&all(~cellfun(@isempty,saved.runs),'all'),fprintf('SKIP COMPLETE %s\n',tag);continue,end;error('Partial fine result exists; preserve and use new stamp.');end
  replay=struct('source',fullfile(inputFolder,[name '_' level 'mm.mat']));if strcmp(name,'rotation'),saved=load(fullfile(root,'results','runs',sprintf('fine_bvp_insertion_%smm_%s',level,stamp),'comparison.mat'),'runs');replay.initialEstimates=saved.runs(:,101)';end
  fprintf('START FINE BVP %s\n',tag);run_inclined_modes(Inf,tag,cfg,replay);fprintf('DONE FINE BVP %s\n',tag);
 end
end
end
