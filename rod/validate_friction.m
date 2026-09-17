function report = validate_friction(results)
%VALIDATE_FRICTION Independently audit saved forces against measured slip.
% Positive work is rejected only above the documented displacement tolerance;
% the incremental linearized forward solver has small nonlinear stick drift.
tolMm = 0.005;
if isfield(results.config.forceSensor, 'slipToleranceMm')
    tolMm = results.config.forceSensor.slipToleranceMm;
end
o = results.ours;
m = results.config.forceSensor.numFrictionDirs;
nt = size(o.state,2);
work = zeros(1,nt);
slip = zeros(1,nt);
modelWork = zeros(1,nt);
minW = zeros(1,nt);
slack = zeros(1,nt);
stepTolerance=tolMm*ones(1,nt);
for k=1:nt
    n=o.planeNormal(:,k); n=n/norm(n);
    fn=o.state(7,k); beta=o.state(8:7+m,k); lambda=o.state(8+m,k);
    ft=o.contactForceResultant(:,k)-n*fn;
    s=o.state(6,k);
    old=interp1(results.forward.s,results.measurements.previousShape{k}.p',s)';
    % Use the shape integrated from measured curvature. Joining sparse
    % positions by chords introduces spatial interpolation error which must
    % not be interpreted as temporal contact slip on a curved rod.
    measured=interp1(results.forward.s,results.measurements.p(:,:,k)',s)';
    predicted=interp1(results.forward.s,o.p(:,:,k)',s)';
    vt=(eye(3)-n*n')*(measured-old);
    vp=(eye(3)-n*n')*(predicted-old);
    baseStep=results.measurements.baseTraj(1:3,4,k)-results.measurements.previousShape{k}.T_base(1:3,4);
    tangentStep=norm((eye(3)-n*n')*baseStep);
    if tangentStep>1e-8, stepTolerance(k)=min(tolMm,0.25*tangentStep); end
    slip(k)=norm(vt); work(k)=ft'*vt; modelWork(k)=ft'*vp;
    t=contact_tangent(n);
    if m==1
        D=t;
    elseif m==2
        D=[t,-t];
    else
        theta=(0:m-1)*2*pi/m;
        D=[t,cross(n,t)]*[cos(theta);sin(theta)];
    end
    minW(k)=min(D'*vp+lambda);
    slack(k)=results.forward.frictionMu(k)*fn-sum(beta);
    assert(all(isfinite([fn;beta;lambda;ft;vt;vp])), 'Nonfinite friction state.');
    assert(min([fn;beta;lambda])>=-1e-8, 'Negative contact variable.');
    assert(norm(ft-D*beta)<1e-6, 'Friction direction basis/reconstruction mismatch.');
    assert(norm(ft)<=results.forward.frictionMu(k)*fn+1e-4, 'Circular friction cone violated.');
end
sliding=slip>stepTolerance & results.forward.frictionMu>0 & o.state(7,:)>0.05;
assert(max([0,work(sliding)])<1e-5, ...
    'Friction assists observed sliding (positive work); check mode threshold and world-frame displacement.');
assert(min(slack)>=-1e-4, 'Polyhedral friction cone violated.');
assert(min(minW)>=-1e-5, 'Tangential complementarity inequality w >= 0 violated.');
assert(max(modelWork)<1e-4, 'Estimated friction produces positive work.');
if isfield(results.forward,'frictionWMin')
    assert(min(results.forward.frictionWMin)>=-1e-7, 'Forward LCP w is negative.');
    assert(max(results.forward.frictionConeViolation)<1e-7, 'Forward polyhedral cone violated.');
    assert(max(results.forward.frictionDirectionViolation)<1e-6, 'Forward LCP friction assists slip.');
end
report.slipToleranceMm=tolMm;
report.observedSlipMm=slip;
report.stepToleranceMm=stepTolerance;
report.observedWorkNmm=work;
report.slidingFrames=find(sliding);
report.maxSlidingWorkNmm=max([0,work(sliding)]);
report.maxModelWorkNmm=max(modelWork);
report.minWmm=min(minW);
report.minConeSlackN=min(slack);
fprintf('Friction audit passed: sliding=%s, max positive sliding work=%.3g N mm, min cone slack=%.3g N, min w=%.3g mm\n', ...
    mat2str(report.slidingFrames),report.maxSlidingWorkNmm,report.minConeSlackN,report.minWmm);
end
