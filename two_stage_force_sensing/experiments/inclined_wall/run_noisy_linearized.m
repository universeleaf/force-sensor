function run_noisy_linearized(stamp)
% Paired iid Gaussian Bishop measurements; noise-aware covariance in both stages.
if nargin<1,stamp='20261011';end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));
folder=fullfile(root,'results','runs',['noisy_linearized_' stamp]);noise=load(fullfile(folder,'noise_payload.mat'));
assert(isfile(fullfile(folder,'noise_calibration.json')));
for target=[2 5]
 sigma=noise.(['sigma' num2str(target)]);cfg=linearized_mode_settings;cfg.fbgRmsAcceptance=1.24;cfg.sensorStd=sigma;cfg.allAssignments=false;cfg.targetNoiseTipRmsMm=target;cfg.noiseSeed=33001;cfg.noiseModel='iid Gaussian Bishop w1,w2 per station and time';cfg.muUncertainty=false;
 for scenario={'insertion','rotation'}
  name=scenario{1};tag=sprintf('inclined_%s_noisy%dmm_%s',name,target,stamp);outpath=fullfile(root,'results','runs',tag,'comparison.mat');
  if strcmp(name,'insertion'),source=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall','truth.mat');
  else,source=fullfile(root,'results','runs','inclined_wall_rotation_20261010','truth.mat');end
  a=load(source);input=a.input;truth=a.truth;clean=input.packet.bishopCurvaturePerMm;delta=noise.([name 'StandardNoise']);assert(isequal(size(clean),size(delta)));
  input.packet.bishopCurvaturePerMm=clean+sigma*delta;
  input.packet.previousBishopCurvaturePerMm=input.packet.bishopCurvaturePerMm(:,:,[1 1:size(clean,3)-1]);
  input.packet.curvatureNoiseStdPerMm=sigma;input.config.forceSensor.measurementStd.curvature=sigma;
  input.packet.provenance=sprintf('Original physical truth, iid Gaussian Bishop noise; seed33001; target%dmm geometric tip RMS; std%.12g/mm',target,sigma);
  frames=tsfs.read_input(input);expected=noise.([name 'LocalTips' num2str(target)]);maxTipDiscrepancy=0;
  for k=1:numel(frames)
   sh=tsfs.reconstruct_bishop(frames{k});world=frames{k}.base(1:3,1:3)*expected(:,k)+frames{k}.base(1:3,4);maxTipDiscrepancy=max(maxTipDiscrepancy,norm(sh.p(:,end)-world));
   assert(norm(frames{k}.CU-eye(numel(frames{k}.u))*sigma^2,'fro')<1e-18);
   if k>1,assert(isequal(frames{k}.previous.u,frames{k-1}.u));end
  end
  assert(maxTipDiscrepancy<1e-8,'Python/MATLAB geometric integration mismatch');
  fprintf('NOISE VALIDATED %s target%d sigma%.9g reconstruction difference%.3g mm\n',name,target,sigma,maxTipDiscrepancy);
  sourceNoisy=fullfile(folder,sprintf('%s_%dmm_truth.mat',name,target));save(sourceNoisy,'input','truth','sigma','target','maxTipDiscrepancy','-v7.3');replay=struct('source',sourceNoisy);
  if strcmp(name,'rotation')
   ins=load(fullfile(root,'results','runs',sprintf('inclined_insertion_noisy%dmm_%s',target,stamp),'comparison.mat'),'runs');
   replay.initialEstimates=ins.runs(:,21)';
  end
  if isfile(outpath)
   old=load(outpath,'runs');if all(~cellfun(@isempty,old.runs),'all')&&size(old.runs,2)==numel(frames),fprintf('SKIP COMPLETE %s\n',tag);continue,end
   error('Partial run exists; preserve it and use a new stamp or explicit recovery.');
  end
  fprintf('START NOISY %s\n',tag);run_inclined_modes(Inf,tag,cfg,replay);fprintf('DONE NOISY %s\n',tag);
 end
end
end
