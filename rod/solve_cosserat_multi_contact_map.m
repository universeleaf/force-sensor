function shape = solve_cosserat_multi_contact_map(tube, contactS, contactForces, tipForce, options)
%SOLVE_COSSERAT_MULTI_CONTACT_MAP Static Cosserat map with several point loads.
% The force at an arc-length s_i is a jump in the internal force.  This
% helper is deliberately separate from solve_cosserat_force_map: the public
% estimator still has a one-contact state, while this map generates a
% physically independent multi-contact stress test.
if nargin < 5, options = struct; end
if ~isfield(options,'relativeTolerance'), options.relativeTolerance = 2e-7; end
if ~isfield(options,'momentToleranceNmm'), options.momentToleranceNmm = 2e-4; end
if ~isfield(options,'collisionStepMm'), options.collisionStepMm = 0.5; end
if ~isfield(options,'maxRhsEvaluations'), options.maxRhsEvaluations = 50000; end
s = tube.s(:)'; ns = numel(s); K = reshape(getTubeK(tube),3,[]);
assert(numel(s)>=2 && all(isfinite(s)) && all(diff(s)>0), ...
    'rod:InvalidTubeGeometry','Arc-length samples must be finite and strictly increasing.');
assert(isequal(size(tube.uhat),[3 ns]) && all(isfinite(tube.uhat),'all'), ...
    'rod:InvalidTubeCurvature','Intrinsic curvature must be a finite 3-by-N field.');
contactS = contactS(:)';
assert(~isempty(contactS) && all(isfinite(contactS)) && all(diff(contactS)>0) && ...
    all(contactS>s(1)) && all(contactS<s(end)), ...
    'rod:InvalidContactArcLength','Contact arc lengths must be ordered and interior to the rod.');
contactForces = reshape(contactForces,3,[]);
assert(size(contactForces,2)==numel(contactS) && all(isfinite(contactForces),'all'), ...
    'rod:InvalidForceVector','Each contact force must be a finite 3-vector.');
tipForce = tipForce(:);
assert(numel(tipForce)==3 && all(isfinite(tipForce)), ...
    'rod:InvalidForceVector','The tip force must be a finite 3-vector.');
assert(isequal(size(K),[3 ns]) && all(isfinite(K),'all') && all(K>0,'all'), ...
    'rod:InvalidTubeStiffness','The Cosserat stiffness must be finite and positive.');
L=s(end)-s(1); momentScale=mean(K(1:2,:),'all')/L;
Rbase=tube.T_base(1:3,1:3); pbase=tube.T_base(1:3,4);
odeOptions=odeset('RelTol',options.relativeTolerance,'AbsTol',options.relativeTolerance*0.01);
[~,freeP]=integrate_curvature_field(tube,tube.uhat,tube.T_base);
initialMoment=zeros(3,1);
for q=1:numel(contactS)
    pc0=interp1(s,freeP',contactS(q))';
    initialMoment=initialMoment+cross(pc0-pbase,contactForces(:,q));
end
initialMoment=initialMoment+cross(freeP(:,end)-pbase,tipForce);
rootOptions=optimoptions('fsolve','Display','off','FunctionTolerance',1e-15, ...
    'StepTolerance',1e-10,'OptimalityTolerance',1e-12,'FiniteDifferenceStepSize',1e-5, ...
    'MaxIterations',35,'MaxFunctionEvaluations',180);
rhsEvaluations=0;
activeK0=zeros(3,1); activeStiffness=ones(3,1); activeLoad=zeros(3,1);
[baseMoment,~,flag,output]=fsolve(@shoot,initialMoment/momentScale,rootOptions);
[residual,solutions,breaks,segmentLoad]=shoot(baseMoment);
residualNmm=norm(residual,inf)*momentScale;
if ~isfinite(residualNmm) || residualNmm>options.momentToleranceNmm
    error('rod:CosseratEquilibrium','Multi-contact shooting failed: tip moment residual %.4g N mm (exit %d).', ...
        residualNmm,flag);
end
y=zeros(15,ns);
for j=1:numel(solutions)
    take=s>=breaks(j)&s<=breaks(j+1); y(:,take)=deval(solutions{j},s(take));
end
R=reshape(y(4:12,:),3,3,ns); u=tube.uhat;
for j=1:ns, u(:,j)=u(:,j)+(R(:,:,j)'*y(13:15,j))./K(:,j); end
contactPoints=zeros(3,numel(contactS));
for q=1:numel(contactS)
    seg=find(breaks<=contactS(q),1,'last'); seg=min(seg,numel(solutions));
    yc=deval(solutions{seg},contactS(q));
    contactPoints(:,q)=yc(1:3);
end
collisionS=unique([s,linspace(s(1),s(end),ceil(L/options.collisionStepMm)+1)]);
collisionP=zeros(3,numel(collisionS));
for j=1:numel(solutions)
    take=collisionS>=breaks(j)&collisionS<=breaks(j+1);
    yCollision=deval(solutions{j},collisionS(take));
    collisionP(:,take)=yCollision(1:3,:);
end
shape=struct('u',u,'R',R,'p',y(1:3,:),'contactPoints',contactPoints, ...
    'contactS',contactS,'momentNmm',y(13:15,:),'collisionS',collisionS, ...
    'collisionP',collisionP,'tipMomentResidualNmm',residualNmm, ...
    'baseMomentNmm',baseMoment*momentScale,'exitflag',flag, ...
    'shootingIterations',output.iterations,'rhsEvaluations',rhsEvaluations, ...
    'maxRhsEvaluations',options.maxRhsEvaluations,'segmentLoad',{segmentLoad}, ...
    'mechanics','nonlinear-cosserat-shooting-multi-contact');

    function [r,sol,bks,loads]=shoot(m0)
        bks=unique([s,contactS]); sol=cell(numel(bks)-1,1); loads=cell(size(sol));
        yy=[pbase;Rbase(:);momentScale*m0];
        for a=1:numel(sol)
            aa=bks(a); bb=bks(a+1); midpoint=0.5*(aa+bb);
            loads{a}=tipForce+sum(contactForces(:,contactS>=bb-10*eps),2);
            [~,ii]=min(abs(s-midpoint)); activeK0=tube.uhat(:,ii); activeStiffness=K(:,ii); activeLoad=loads{a};
            sol{a}=ode45(@balance,[aa bb],yy,odeOptions); yy=sol{a}.y(:,end);
        end
        r=yy(13:15)/momentScale;
    end
    function dy=balance(~,state)
        rhsEvaluations=rhsEvaluations+1;
        if rhsEvaluations>options.maxRhsEvaluations
            error('rod:CosseratEquilibrium','Multi-contact RHS budget exceeded (%d).',options.maxRhsEvaluations);
        end
        rot=reshape(state(4:12),3,3); tangent=rot(:,3);
        curvature=activeK0+(rot'*state(13:15))./activeStiffness;
        hat=[0 -curvature(3) curvature(2);curvature(3) 0 -curvature(1);-curvature(2) curvature(1) 0];
        dR=rot*hat; dm=cross(tangent,activeLoad); dy=[tangent;dR(:);dm];
    end
end
