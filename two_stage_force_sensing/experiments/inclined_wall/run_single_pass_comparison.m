function run_single_pass_comparison(stamp,methods,targets,maxFrames)
% Matched trajectory replay; same standard-normal noise draws for every solver.
if nargin<1,stamp='20261012';end;if nargin<2,methods={'direct','polygon','current'};end;if nargin<3,targets=[0 .5 1 2];end;if nargin<4,maxFrames=Inf;end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));folder=fullfile(root,'results','runs',['single_pass_inputs_' stamp]);if ~isfolder(folder),mkdir(folder);end
noise=load(fullfile(root,'results','runs','noisy_linearized_20261011','noise_payload.mat'));
for target=targets
 level=strrep(num2str(target),'.','p');sigma=noise.sigma2*target/2;
 for scenario={'insertion','rotation'}
  name=scenario{1};source=fullfile(folder,[name '_' level 'mm.mat']);if isfile(source),continue,end
  if strcmp(name,'insertion'),original=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall','truth.mat');else,original=fullfile(root,'results','runs','inclined_wall_rotation_20261010','truth.mat');end
  a=load(original);input=a.input;truth=a.truth;clean=input.packet.bishopCurvaturePerMm;delta=noise.([name 'StandardNoise']);input.packet.bishopCurvaturePerMm=clean+sigma*delta;
  input.packet.previousBishopCurvaturePerMm=input.packet.bishopCurvaturePerMm(:,:,[1 1:size(clean,3)-1]);sensorStd=sigma;if target==0,sensorStd=1e-4;end
  input.packet.curvatureNoiseStdPerMm=sensorStd;input.config.forceSensor.measurementStd.curvature=sensorStd;input.packet.provenance=sprintf('Matched Bishop Gaussian seed33001; noise target %.2gmm; sigma%.12g; zero-noise covariance floor1e-4',target,sigma);save(source,'input','truth','sigma','sensorStd','target','-v7.3');
 end
end
for im=1:numel(methods)
 method=methods{im};
 for target=targets
  % The existing exhaustive 2 mm reference has identical data and settings.
  if strcmp(method,'current')&&target==2,fprintf('REUSE current 2mm exhaustive 20261011\n');continue,end
  level=strrep(num2str(target),'.','p');cfg=linearized_mode_settings;cfg.modeSearch='exhaustive';cfg.allAssignments=true;cfg.fbgRmsAcceptance=1.24;cfg.sensorStd=noise.sigma2*target/2;if target==0,cfg.sensorStd=1e-4;end
  cfg.exactCircularFriction=~strcmp(method,'polygon');cfg.estimatorMus=.3;cfg.reportFrameStart=true;cfg.targetNoiseTipRmsMm=target;cfg.noiseSeed=33001;cfg.maxIterationsPerAssignment=300;cfg.selectionPolicy='lowest affine feasible objective for single pass; true physics diagnosis only';cfg.polygonEdges=8;
  if ~strcmp(method,'current'),cfg.solverFunction='tsfs.single_linearization_solve';cfg.linearization.initialRadius=Inf;cfg.singlePassTrustRegion='none, user approved 2026-10-12';cfg.maxIterationsPerAssignment=100;if strcmp(method,'direct'),cfg.singlePassMethod='full-cone-sqp';else,cfg.singlePassMethod='polygon-qp';end;end
  for scenario={'insertion','rotation'}
   name=scenario{1};tag=sprintf('singlepass_%s_%s_%smm_%s',method,name,level,stamp);path=fullfile(root,'results','runs',tag,'comparison.mat');T=41;if strcmp(name,'rotation'),T=25;end;T=min(T,maxFrames);
   if isfile(path),a=load(path,'runs');if size(a.runs,2)==T&&all(~cellfun(@isempty,a.runs),'all'),fprintf('SKIP COMPLETE %s\n',tag);continue,end;error('Partial result exists: preserve and choose fresh stamp.');end
   replay=struct('source',fullfile(folder,[name '_' level 'mm.mat']));
   if strcmp(name,'rotation')
    if maxFrames<21,continue,end
    a=load(fullfile(root,'results','runs',sprintf('singlepass_%s_insertion_%smm_%s',method,level,stamp),'comparison.mat'),'runs');replay.initialEstimates=a.runs(:,21)';
   end
   fprintf('START SINGLEPASS %s\n',tag);run_inclined_modes(maxFrames,tag,cfg,replay);fprintf('DONE SINGLEPASS %s\n',tag);
  end
 end
end
end
