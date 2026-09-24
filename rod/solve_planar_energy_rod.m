function result = solve_planar_energy_rod(model,thetaStart)
%SOLVE_PLANAR_ENERGY_ROD Independent nonlinear planar static rod model.
% Nodal tangent angles, midpoint centerline quadrature, bending energy and
% unilateral frictionless nodal plane constraints. Does NOT call solveShape,
% computeJacobian, the LCP implementation, or the inverse mechanical model.
% Units: mm, N, rad; EI in N mm^2; positive theta bends +z toward +x.
s=model.sMm(:); h=diff(s); n=numel(h);
assert(all(h>0) && numel(model.intrinsicCurvaturePerMm)==n);
assert(isfinite(model.EINmm2)&&model.EINmm2>0);
k0=model.intrinsicCurvaturePerMm(:); base=model.baseXZ(:);
force=model.tipForceXZ(:); theta0=model.baseAngleRad;
assert(numel(base)==2 && numel(force)==2);
if nargin<2 || isempty(thetaStart)
    thetaStart=theta0+cumsum(h.*k0);
else
    thetaStart=thetaStart(:); assert(numel(thetaStart)==n);
end
B=eye(n)-diag(ones(n-1,1),-1);
C=0.5*(eye(n)+diag(ones(n-1,1),-1));
prefix=tril(ones(n));
planeActive=isfield(model,'planePointXZ') && ~isempty(model.planePointXZ);
if planeActive
    plane=model.planePointXZ(:); normal=model.planeNormalXZ(:); normal=normal/norm(normal);
    assert(normal'*(base-plane)>=-1e-8,'Clamped base lies inside the obstacle.');
end
options=optimoptions('fmincon','Algorithm','interior-point','Display','off', ...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
    'HessianFcn',@(theta,lambda)hessian(theta,lambda.ineqnonlin), ...
    'MaxIterations',500,'MaxFunctionEvaluations',10000, ...
    'OptimalityTolerance',1e-8,'ConstraintTolerance',1e-9,'StepTolerance',1e-13);
timer=tic;
[theta,value,flag,output,multipliers]=fmincon(@energy,thetaStart,[],[],[],[], ...
    [],[],@constraints,options);
[p,~,~]=geometry(theta); [c,~,dc]=constraints(theta); [~,gradient]=energy(theta);
reaction=zeros(2,n);
if planeActive, reaction=normal*max(multipliers.ineqnonlin(:)',0); end
stationarity=gradient+dc*multipliers.ineqnonlin;
allTheta=[theta0;theta];
result=struct('sMm',s','thetaRad',allTheta','pXZ',[base,p], ...
    'segmentCurvaturePerMm',diff(allTheta)./h,'contactForceXZ',reaction, ...
    'tipForceXZ',force,'contactResultantXZ',sum(reaction,2), ...
    'objectiveNmm',value,'exitflag',flag,'iterations',output.iterations, ...
    'stationarityInfNmm',norm(stationarity,inf), ...
    'maxPenetrationMm',max([0;c]),'seconds',toc(timer), ...
    'complementarityNmm',max([0;abs(c.*multipliers.ineqnonlin)]));
result.method='Independent planar static bending-energy minimization; frictionless nodal contact, midpoint quadrature.';
if planeActive, result.contactGapMm=-c; else, result.contactGapMm=[]; end
% Expose gradient checks at a deterministic non-equilibrium point.
if isfield(model,'checkDerivatives') && model.checkDerivatives
    probe=theta+0.01*sin((1:n)'); [~,g]=energy(probe); [~,~,a]=constraints(probe);
    fd=zeros(n,1); cd=zeros(size(a)); step=1e-6;
    for j=1:n
        plus=probe; minus=probe; plus(j)=plus(j)+step; minus(j)=minus(j)-step;
        fd(j)=(energy(plus)-energy(minus))/(2*step);
        cp=constraints(plus); cm=constraints(minus); cd(j,:)=(cp-cm)'/(2*step);
    end
    result.objectiveGradientRelativeError=norm(g-fd)/max(1,norm(fd));
    result.constraintGradientRelativeError=norm(a-cd,'fro')/max(1,norm(cd,'fro'));
end

    function [p,jx,jz]=geometry(theta)
        midpoint=C*theta; midpoint(1)=midpoint(1)+0.5*theta0;
        p=base+[cumsum(h.*sin(midpoint))';cumsum(h.*cos(midpoint))'];
        if nargout>1
            jx=prefix*diag(h.*cos(midpoint))*C;
            jz=prefix*diag(-h.*sin(midpoint))*C;
        end
    end
    function [value,gradient]=energy(theta)
        delta=B*theta; delta(1)=delta(1)-theta0; delta=delta-h.*k0;
        [p,jx,jz]=geometry(theta);
        value=0.5*model.EINmm2*sum(delta.^2./h)-force'*p(:,end);
        gradient=model.EINmm2*B'*(delta./h)-jx(end,:)'*force(1)-jz(end,:)'*force(2);
    end
    function [c,ceq,dc,dceq]=constraints(theta)
        ceq=[]; dceq=[];
        if ~planeActive, c=zeros(0,1); dc=zeros(n,0); return; end
        [p,jx,jz]=geometry(theta);
        c=-(normal'*(p-plane))'; dc=-(normal(1)*jx+normal(2)*jz)';
    end
    function H=hessian(theta,lambda)
        midpoint=C*theta; midpoint(1)=midpoint(1)+0.5*theta0;
        diagonal=h.*(force(1)*sin(midpoint)+force(2)*cos(midpoint));
        if planeActive
            downstream=flipud(cumsum(flipud(lambda(:))));
            diagonal=diagonal+h.*(normal(1)*sin(midpoint)+normal(2)*cos(midpoint)).*downstream;
        end
        H=model.EINmm2*B'*diag(1./h)*B+C'*diag(diagonal)*C;
        H=0.5*(H+H');
    end
end
