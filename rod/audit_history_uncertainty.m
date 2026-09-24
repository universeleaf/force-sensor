function report = audit_history_uncertainty(folder,curvatureStdPerMm)
%AUDIT_HISTORY_UNCERTAINTY Linearized propagation of paired bending noise.
% No force/contact truth used for selecting the evaluation point. Assumes
% independent current/prior bending noise and fixed base/plane calibration.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(folder,'sensor_input.mat'),'sensorInput'); input=d.sensorInput;
tube=input.tube; packet=input.packet; cfg=input.config;
assert(isscalar(curvatureStdPerMm) && isfinite(curvatureStdPerMm) && curvatureStdPerMm>=0, ...
    'Curvature noise standard deviation must be finite and nonnegative.');
assert(strcmp(cfg.sensing.curvatureInterpolation,'intrinsic-delta') && cfg.sensing.shapeSmoothing==0, ...
    'This diagnostic currently supports intrinsic-delta interpolation without smoothing only.');
m=measurements_from_sensor_packet(tube,packet,cfg);
nt=size(m.p,3); idx=packet.fbgIdx; q=2*numel(idx);
report=struct('scope','First-order independent bending-noise propagation; known base/plane, calibrated intrinsic shape; no force truth.', ...
    'curvatureStdPerMm',curvatureStdPerMm,'observedSlipMm',zeros(1,nt), ...
    'tangentialDifferenceStdMaxMm',zeros(1,nt),'arcMm',zeros(1,nt));
step=1e-7;
for k=1:nt
    normal=m.planeNormalMeasured(:,k); P=eye(3)-normal*normal';
    [~,j]=min(abs(normal'*(m.p(:,:,k)-m.planePointMeasured(:,k))));
    report.arcMm(k)=tube.s(j);
    delta=P*(m.p(:,j,k)-m.previousShape{k}.p(:,j));
    report.observedSlipMm(k)=norm(delta);
    J=zeros(3,2*q);
    for previous=0:1
        if previous
            sparse=packet.previousCurvaturePerMm(:,:,k); base=packet.previousBasePose(:,:,k);
        else
            sparse=packet.curvaturePerMm(:,:,k); base=packet.basePose(:,:,k);
        end
        for column=1:q
            sensor=ceil(column/2); axis=1+mod(column-1,2);
            plus=sparse;minus=sparse;plus(axis,sensor)=plus(axis,sensor)+step;minus(axis,sensor)=minus(axis,sensor)-step;
            J(:,previous*q+column)=P*(point(plus,base,j)-point(minus,base,j))/(2*step);
        end
    end
    covariance=curvatureStdPerMm^2*(J*J');
    report.tangentialDifferenceStdMaxMm(k)=sqrt(max(eig(covariance)));
end
report.slipOverStd=report.observedSlipMm./max(report.tangentialDifferenceStdMaxMm,eps);
report.slipThresholdMm=cfg.forceSensor.slipToleranceMm;
report.maxNoiseToThreshold=max(report.tangentialDifferenceStdMaxMm)/report.slipThresholdMm;
fid=fopen(fullfile(folder,'history_uncertainty.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
disp(report);
    function p=point(sparse,base,j)
        delta=sparse-tube.uhat(:,idx);
        u=tube.uhat;
        for axis=1:3
            u(axis,:)=u(axis,:)+interp1(tube.s(idx),delta(axis,:),tube.s,'pchip');
        end
        [~,~,shape]=solveShape(base,u,tube.s); p=shape(:,j);
    end
end
