function make_validation_fixtures
% Synthetic added twist sensor; NOT a replacement for the original replay.
[root,paths]=setup_tsfs;a=load(fullfile(root,'datasets','spatial_sliding','input.mat'));b=load(fullfile(root,'datasets','spatial_sliding','truth.mat'));
sensorInput=a.sensorInput;truth=b.truth;p=sensorInput.packet;T=numel(p.timeSeconds);u=zeros(3,numel(p.sFbgMm),T);
for k=1:T,u(:,:,k)=interp1(sensorInput.tube.s,truth.u(:,:,k)',p.sFbgMm(:),'linear')';end
assert(max(abs(u(1:2,:,:)-p.curvaturePerMm),[],'all')<1e-12);
sensorInput.packet.curvaturePerMm=u;sensorInput.packet.observedCurvatureAxes=[1 2 3];
sensorInput.packet.provenance='Synthetic extra measured twist channel from existing independent forward truth, same sparse arclengths. Additional-information ablation, not original replay.';
folder=fullfile(root,'datasets','spatial_three_channel');if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'input.mat'),'sensorInput','-v7.3');save(fullfile(folder,'truth.mat'),'truth','-v7.3');
end
