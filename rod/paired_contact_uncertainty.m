function covariance = paired_contact_uncertainty(tube,currentU,currentBase,previous,cfg,arcMm,normal)
%PAIRED_CONTACT_UNCERTAINTY First-order covariance of observed contact slip.
% Independent current/prior bending noise; base, intrinsic shape and plane
% calibration are held fixed. This is a mode-resolution diagnostic, not a
% posterior covariance for the force estimate.
sigma=cfg.forceSensor.historyCurvatureStdPerMm;
assert(isscalar(sigma)&&isfinite(sigma)&&sigma>=0,'Invalid history curvature standard deviation.');
covariance=zeros(3);
if sigma==0, return; end
idx=cfg.forceSensor.fbgIdx; q=2*numel(idx);
normal=normal(:)/norm(normal); tangent=eye(3)-normal*normal';
J=zeros(3,2*q); step=1e-7;
for history=0:1
    if history, sparse=previous.u(:,idx); base=previous.T_base;
    else, sparse=currentU(:,idx); base=currentBase; end
    for column=1:q
        sensor=ceil(column/2); axis=1+mod(column-1,2);
        plus=sparse; minus=sparse;
        plus(axis,sensor)=plus(axis,sensor)+step;
        minus(axis,sensor)=minus(axis,sensor)-step;
        J(:,history*q+column)=tangent*(point(plus,base)-point(minus,base))/(2*step);
    end
end
covariance=sigma^2*(J*J');
covariance=(covariance+covariance')/2;

    function p=point(sparse,base)
        intrinsic=strcmpi(cfg.sensing.curvatureInterpolation,'intrinsic-delta');
        if intrinsic, sparse=sparse-tube.uhat(:,idx); end
        u=zeros(size(tube.uhat));
        for a=1:3
            u(a,:)=interp1(tube.s(idx),sparse(a,:),tube.s,'pchip');
        end
        if cfg.sensing.shapeSmoothing>0
            error('rod:UnsupportedHistorySmoothing', ...
                'History uncertainty propagation requires shapeSmoothing=0.');
        end
        if intrinsic, u=u+tube.uhat; end
        [~,~,pAll]=solveShape(base,u,tube.s);
        p=interp1(tube.s,pAll',arcMm,'linear')';
    end
end
