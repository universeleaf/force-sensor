function run_robust_suite(configNames,seeds,levels,runStamp,scenarios,truthStamp)
% Paired replay: every configuration sees identical references, noise draws,
% sensor covariance and runner logic. Seed 0 is the published 20261012_step5
% noise payload; other seeds are new iid draws with the same protocol
% (restored rotation state shares the insertion maximum-insertion draw).
% scenarios: any of 'insertion','rotation' (needs insertion),'varying'.
if nargin<5||isempty(scenarios),scenarios={'insertion','rotation'};end
if nargin<6,truthStamp='repro_20261024';end
if ischar(configNames),configNames={configNames};end
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'),fullfile(root,'experiments','robust_pass'));
for seed=seeds
 for target=levels
  [insertion,rotation,sensorStd]=robust_inputs(root,truthStamp,seed,target); % empty configNames: inputs only
  if any(strcmp(scenarios,'varying')),varying=varying_input(root,truthStamp,seed,target,insertion);end
  for c=1:numel(configNames)
   name=configNames{c};level=strrep(num2str(target),'.','p');cfg=robust_configurations(name,sensorStd);
   tagI=sprintf('robust_%s_%s_s%d_insertion_%smm',runStamp,name,seed,level);tagR=strrep(tagI,'insertion','rotation');
   if any(strcmp(scenarios,'insertion'))||any(strcmp(scenarios,'rotation'))
    rI=run_robust_trajectory(insertion,tagI,cfg,struct('keepFrames',101));
    if any(strcmp(scenarios,'rotation')),seedOut=rI.kept([rI.kept.frame]==101).out;run_robust_trajectory(rotation,tagR,cfg,struct('initialEstimate',seedOut));end
   end
   if any(strcmp(scenarios,'varying')),run_robust_trajectory(varying,strrep(tagI,'insertion','varying'),cfg);end
  end
 end
end
end

function path=varying_input(root,truthStamp,seed,target,insertion)
% Time-varying tip load reference with exactly the insertion noise draws.
path=strrep(insertion,'insertion_','varying_');if isfile(path),return,end
b=load(insertion,'input','sigma','sensorStd','target','seed');a=load(fullfile(root,'results','runs',['varying_tip_insertion_truth_' truthStamp],'truth.mat'),'input','truth');
input=a.input;truth=a.truth;clean=input.packet.bishopCurvaturePerMm;noise=b.input.packet.bishopCurvaturePerMm-load_clean(root,truthStamp);T=size(clean,3);
input.packet.bishopCurvaturePerMm=clean+noise;input.packet.previousBishopCurvaturePerMm=input.packet.bishopCurvaturePerMm(:,:,[1 1:T-1]);
input.packet.curvatureNoiseStdPerMm=b.sensorStd;input.config.forceSensor.measurementStd.curvature=b.sensorStd;input.packet.provenance=[b.input.packet.provenance '; time-varying tip load reference'];
sigma=b.sigma;sensorStd=b.sensorStd;target=b.target;seed=b.seed;save(path,'input','truth','sigma','sensorStd','target','seed','-v7.3');
end

function clean=load_clean(root,truthStamp)
a=load(fullfile(root,'results','runs',['fine_bvp_insertion_truth_' truthStamp],'truth.mat'),'input');clean=a.input.packet.bishopCurvaturePerMm;
end

function [insertion,rotation,sensorStd]=robust_inputs(root,truthStamp,seed,target)
folder=fullfile(root,'results','runs',sprintf('robust_inputs_%s',truthStamp),sprintf('seed%d',seed));if ~isfolder(folder),mkdir(folder);end
level=strrep(num2str(target),'.','p');insertion=fullfile(folder,['insertion_' level 'mm.mat']);rotation=fullfile(folder,['rotation_' level 'mm.mat']);
noise=load(fullfile(root,'results','runs',['fine_bvp_inputs_' truthStamp],'noise_payload.mat'));sigma=noise.sigma2*target/2;sensorStd=sigma;if target==0,sensorStd=1e-4;end
if isfile(insertion)&&isfile(rotation),return,end
if seed==0
 draws=struct('insertion',noise.insertionStandardNoise,'rotation',noise.rotationStandardNoise);
else
 rng(1000*seed+7,'twister');draws.insertion=randn(size(noise.insertionStandardNoise));draws.rotation=randn(size(noise.rotationStandardNoise));draws.rotation(:,:,1)=draws.insertion(:,:,101);
end
for scenario={'insertion','rotation'}
 name=scenario{1};a=load(fullfile(root,'results','runs',sprintf('fine_bvp_%s_truth_%s',name,truthStamp),'truth.mat'),'input','truth');input=a.input;truth=a.truth;
 clean=input.packet.bishopCurvaturePerMm;T=size(clean,3);input.packet.bishopCurvaturePerMm=clean+sigma*draws.(name);
 input.packet.previousBishopCurvaturePerMm=input.packet.bishopCurvaturePerMm(:,:,[1 1:T-1]);input.packet.curvatureNoiseStdPerMm=sensorStd;input.config.forceSensor.measurementStd.curvature=sensorStd;
 input.packet.provenance=sprintf('Fine step; iid Gaussian Bishop noise; seed %d (0 = published payload); sigma %.12g',seed,sigma);
 path=insertion;if strcmp(name,'rotation'),path=rotation;end;save(path,'input','truth','sigma','sensorStd','target','seed','-v7.3');
end
end
