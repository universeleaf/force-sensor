function shape = solve_cosserat_force_map(tube, contactS, contactForce, tipForce, options)
%SOLVE_COSSERAT_FORCE_MAP Nonlinear 3-D static Cosserat boundary-value map.
% Units: mm, N, N mm. Inextensible/unshearable rod, no distributed loads or
% applied couples, calibrated piecewise-constant intrinsic curvature/K.
% p'=R e3; R'=R hat(u); m'=-p' x n; u=u_hat+K\(R' m).
% n=fe+fc before contact and fe after contact. Shoot THREE base moments to
% satisfy m(L)=0. The force jump is integrated at its exact arclength.
% This solves equilibrium on the CURRENT geometry, never a previous Jacobian.
% A converged local equilibrium does not certify uniqueness or stability.
if nargin<5, options=struct; end
if ~isfield(options,'relativeTolerance'), options.relativeTolerance=2e-7; end
if ~isfield(options,'momentToleranceNmm'), options.momentToleranceNmm=2e-4; end
if ~isfield(options,'collisionStepMm'), options.collisionStepMm=0.5; end
if ~isfield(options,'maxRhsEvaluations'), options.maxRhsEvaluations=50000; end
assert(isnumeric(options.maxRhsEvaluations)&&isreal(options.maxRhsEvaluations)&& ...
    isscalar(options.maxRhsEvaluations)&&isfinite(options.maxRhsEvaluations)&& ...
    options.maxRhsEvaluations>=1&&options.maxRhsEvaluations==fix(options.maxRhsEvaluations), ...
    'rod:InvalidSensorConfig','The Cosserat RHS evaluation budget must be a positive integer.');
assert(isscalar(options.collisionStepMm)&&isfinite(options.collisionStepMm)&&options.collisionStepMm>0, ...
    'rod:InvalidSensorConfig','Collision sampling step must be finite and positive.');
s=tube.s(:)'; ns=numel(s); K=reshape(getTubeK(tube),3,[]);
% Reject malformed mechanics inputs at the boundary.  Silently clipping an
% out-of-domain contact location can turn an invalid optimizer trial into a
% plausible-looking equilibrium at the rod end, while zero/NaN stiffness can
% propagate through the constitutive map as an apparently converged shape.
assert(numel(s)>=2 && all(isfinite(s)) && all(diff(s)>0), ...
    'rod:InvalidTubeGeometry','Arc-length samples must be finite and strictly increasing.');
assert(isfield(tube,'uhat') && isequal(size(tube.uhat),[3 ns]) && all(isfinite(tube.uhat),'all'), ...
    'rod:InvalidTubeCurvature','Intrinsic curvature must be a finite 3-by-N field matching arc-length samples.');
assert(isequal(size(K),[3 ns]) && all(isfinite(K),'all') && all(K>0,'all'), ...
    'rod:InvalidTubeStiffness','The Cosserat stiffness must be a finite positive 3-by-N field.');
assert(isscalar(contactS) && isfinite(contactS) && contactS>=s(1) && contactS<=s(end), ...
    'rod:InvalidContactArcLength','Contact arc length %.6g mm is outside [%.6g, %.6g] mm.', ...
    contactS,s(1),s(end));
fc=contactForce(:); fe=tipForce(:);
assert(numel(fc)==3 && numel(fe)==3 && all(isfinite([fc;fe])) && isreal([fc;fe]), ...
    'rod:InvalidForceVector','Contact and tip forces must be finite real 3-vectors.');
% Exact memoization avoids repeated shooting for plane/lambda perturbations.
% Include all mechanics/accuracy inputs; no approximate-key or truth cache.
persistent context keys values
newContext={s,tube.uhat,K,tube.T_base,options};
if isempty(context)||~isequal(context,newContext)
    context=newContext; keys=zeros(7,0); values={};
end
key=[contactS;fc;fe]; found=find(all(keys==key,1),1);
if ~isempty(found), shape=values{found}; return; end
change=any(diff(tube.uhat(:,1:end-1),1,2)~=0,1)|any(diff(K(:,1:end-1),1,2)~=0,1);
breaks=unique([s(1),s(find(change)+1),contactS,s(end)]);
segmentIndex=zeros(1,numel(breaks)-1);
for j=1:numel(segmentIndex)
    segmentIndex(j)=find(s<=0.5*(breaks(j)+breaks(j+1)),1,'last');
end
L=s(end)-s(1); momentScale=mean(K(1:2,:),'all')/L;
Rbase=tube.T_base(1:3,1:3); pbase=tube.T_base(1:3,4);
odeOptions=odeset('RelTol',options.relativeTolerance,'AbsTol',options.relativeTolerance*0.01);
% Deterministic seed from the unloaded continuous rod, not a measured shape.
[~,freeP]=integrate_curvature_field(tube,tube.uhat,tube.T_base);
pc0=interp1(s,freeP',contactS)';
initialMoment=cross(pc0-pbase,fc)+cross(freeP(:,end)-pbase,fe);
rootOptions=optimoptions('fsolve','Display','off','FunctionTolerance',1e-15, ...
    'StepTolerance',1e-10,'OptimalityTolerance',1e-12, ...
    'FiniteDifferenceStepSize',1e-5,'MaxIterations',35,'MaxFunctionEvaluations',180);
% fsolve's evaluation limit does NOT bound work inside ode45. An unbounded
% SQP trial force can otherwise keep one ODE solve busy for hours. Bound the
% total work of this mechanics call; reject, never return a truncated shape.
rhsEvaluations=0;
[baseMoment,~,flag,output]=fsolve(@shoot,initialMoment/momentScale,rootOptions);
[residual,solutions]=shoot(baseMoment);
residualNmm=norm(residual,inf)*momentScale;
if ~isfinite(residualNmm)||residualNmm>options.momentToleranceNmm
    error('rod:CosseratEquilibrium','3-D shooting failed: tip moment residual %.4g N mm (exit %d).',residualNmm,flag);
end
y=zeros(15,ns);
for j=1:numel(solutions)
    take=s>=breaks(j)&s<=breaks(j+1);
    y(:,take)=deval(solutions{j},s(take));
end
R=reshape(y(4:12,:),3,3,ns); u=tube.uhat;
for j=1:ns, u(:,j)=u(:,j)+(R(:,:,j)'*y(13:15,j))./K(:,j); end
seg=find(breaks<=contactS,1,'last'); seg=min(seg,numel(solutions));
yc=deval(solutions{seg},contactS);
% A fixed grid keeps the nonlinear constraint dimension constant as contact
% moves. Evaluate the ODE interpolant, not straight chords between rod nodes.
collisionS=unique([s,linspace(s(1),s(end),ceil(L/options.collisionStepMm)+1)]);
collisionP=zeros(3,numel(collisionS));
for j=1:numel(solutions)
    take=collisionS>=breaks(j)&collisionS<=breaks(j+1);
    samples=deval(solutions{j},collisionS(take));
    collisionP(:,take)=samples(1:3,:);
end
shape=struct('u',u,'R',R,'p',y(1:3,:),'pc',yc(1:3), ...
    'contactTangent',yc(10:12),'momentNmm',y(13:15,:), ...
    'collisionS',collisionS,'collisionP',collisionP,'rodArcBoundsMm',s([1 end]), ...
    'tipMomentResidualNmm',residualNmm,'baseMomentNmm',baseMoment*momentScale, ...
    'exitflag',flag,'shootingIterations',output.iterations, ...
    'rhsEvaluations',rhsEvaluations,'maxRhsEvaluations',options.maxRhsEvaluations, ...
    'relativeTolerance',options.relativeTolerance,'mechanics','nonlinear-cosserat-shooting');
keys(:,end+1)=key; values{end+1}=shape;
if numel(values)>96, keys(:,1)=[]; values(1)=[]; end

    function [r,sol]=shoot(m0)
        yy=[pbase;Rbase(:);momentScale*m0]; sol=cell(size(segmentIndex));
        for a=1:numel(segmentIndex)
            ii=segmentIndex(a); k0=tube.uhat(:,ii); stiffness=K(:,ii);
            load=fe;
            if breaks(a+1)<=contactS, load=load+fc; end
            sol{a}=ode45(@balance,breaks(a:a+1),yy,odeOptions);
            yy=sol{a}.y(:,end);
        end
        r=yy(13:15)/momentScale;
        function dy=balance(~,state)
            rhsEvaluations=rhsEvaluations+1;
            if rhsEvaluations>options.maxRhsEvaluations
                error('rod:CosseratEquilibrium', ...
                    ['Cosserat RHS budget exceeded (%d), |Fc|=%.6g N, |Fe|=%.6g N, s=%.6g mm; ', ...
                    'reject trial forces, no partial equilibrium returned.'], ...
                    options.maxRhsEvaluations,norm(fc),norm(fe),contactS);
            end
            rot=reshape(state(4:12),3,3); tangent=rot(:,3);
            curvature=k0+(rot'*state(13:15))./stiffness;
            hat=[0 -curvature(3) curvature(2);curvature(3) 0 -curvature(1);-curvature(2) curvature(1) 0];
            dR=rot*hat;
            dm=[tangent(3)*load(2)-tangent(2)*load(3); ...
                tangent(1)*load(3)-tangent(3)*load(1); ...
                tangent(2)*load(1)-tangent(1)*load(2)];
            dy=[tangent;dR(:);dm];
        end
    end
end
