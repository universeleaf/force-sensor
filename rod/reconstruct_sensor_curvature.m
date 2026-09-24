function shape = reconstruct_sensor_curvature(tube,sparseU,idx,base,cfg)
%RECONSTRUCT_SENSOR_CURVATURE Shared measurement/latent-history shape map.
% This kinematic map does not impose previous-frame force equilibrium.
assert(isequal(size(base),[4 4])&&norm(base(4,:)-[0 0 0 1])<1e-9,'Invalid base pose.');
assert(norm(base(1:3,1:3)'*base(1:3,1:3)-eye(3),'fro')<1e-6,'Base rotation must be orthonormal.');
assert(abs(det(base(1:3,1:3))-1)<1e-6,'Base rotation cannot be a reflection.');
intrinsic=strcmpi(cfg.sensing.curvatureInterpolation,'intrinsic-delta');
values=sparseU;
if intrinsic,values=values-tube.uhat(:,idx);end
u=zeros(size(tube.uhat));
for axis=1:3,u(axis,:)=interp1(tube.s(idx),values(axis,:),tube.s,'pchip');end
if cfg.sensing.shapeSmoothing>0
    u=movmean(u,max(3,round(cfg.sensing.shapeSmoothing*numel(idx))),2);
end
if intrinsic,u=u+tube.uhat;end
if isfield(cfg.forceSensor,'mechanicsModel')&&strcmp(cfg.forceSensor.mechanicsModel,'cosserat-shooting')
    [R,p]=integrate_curvature_field(tube,u,base);
else
    [~,R,p]=solveShape(base,u,tube.s);
end
shape=struct('u',u,'R',R,'p',p,'T_base',base,'sparseU',sparseU,'fbgIdx',idx);
end
