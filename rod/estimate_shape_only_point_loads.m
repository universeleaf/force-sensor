function estimate = estimate_shape_only_point_loads(tube, measurements, cfg)
%ESTIMATE_SHAPE_ONLY_POINT_LOADS Explicit in-repository shape-only ablation.
% Same sparse bending samples and reconstructed predecessor as the fused
% method. No plane, friction, contact truth or load labels are read.
% Searches a point-load arclength plus a distinct tip load with a Gaussian
% force prior. This is not an implementation of Aloi or Ferguson.
nt=size(measurements.uSparse,3); ns=numel(tube.s);
axes=1:3;
if isfield(cfg.forceSensor,'curvatureObservedAxes'),axes=cfg.forceSensor.curvatureObservedAxes;end
rows=reshape(3*measurements.fbgIdx(:)'+(axes(:)-3),[],1);
% Match the estimator's actual observation channels; historical configs
% without this field retain their planar calibrated-torsion assumption.
keep=true(size(rows));
sigma=cfg.forceSensor.measurementStd.curvature;
forceStd=[repmat(cfg.forceSensor.priorStd.normalForceN,3,1); ...
          repmat(cfg.forceSensor.priorStd.tipForceN,3,1)];
priorInfo=diag(1./forceStd.^2);
estimate.contactForceResultant=zeros(3,nt); estimate.tipForce=zeros(3,nt);
estimate.contactArcLength=zeros(1,nt); estimate.frameSeconds=zeros(1,nt);
estimate.objective=zeros(1,nt);
for k=1:nt
    timer=tic; previous=measurements.previousShape{k};
    oldBase=previous.T_base; newBase=measurements.baseTraj(:,:,k);
    Q=newBase(1:3,1:3)*oldBase(1:3,1:3)';
    p=Q*(previous.p-oldBase(1:3,4))+newBase(1:3,4);
    R=zeros(size(previous.R));
    for j=1:ns, R(:,:,j)=Q*previous.R(:,:,j); end
    J=computeJacobian(R,p); K=getTubeK(tube);
    tip=bsxfun(@rdivide,J(end-2:end,:)',K);
    z=measurements.uSparse(axes,:,k)-tube.uhat(axes,measurements.fbgIdx);
    z=z(:); z=z(keep)/sigma;
    best=inf;
    for j=1:ns
        contact=bsxfun(@rdivide,J(3*j-2:3*j,:)',K);
        G=[contact(rows,:),tip(rows,:)]/sigma;
        f=(G'*G+priorInfo)\(G'*z);
        cost=sum((G*f-z).^2)+f'*priorInfo*f;
        if cost<best
            best=cost; bestForce=f; bestIndex=j;
        end
    end
    estimate.contactForceResultant(:,k)=bestForce(1:3);
    estimate.tipForce(:,k)=bestForce(4:6);
    estimate.contactArcLength(k)=tube.s(bestIndex);
    estimate.objective(k)=best;
    estimate.frameSeconds(k)=toc(timer);
end
estimate.totalForceResultant=estimate.contactForceResultant+estimate.tipForce;
estimate.description='Per-frame shape-only MAP, grid point-load plus tip load; same sparse/history input and calibrated-torsion assumption, no environment or temporal force prior.';
end
