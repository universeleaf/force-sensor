function estimate = estimate_planar_multi_contact(input)
% Sparse-curvature inverse for a known ordered frictionless contact set.
% INPUT contains geometry, stiffness, base pose, and measurements only.
% Unknowns: base moment, each contact's positive normal force and arc length,
% and an independent two-component tip force. This is a planar constrained
% least-squares prototype, not the full 3-D frictional MPCC/MAP estimator.
model=input.model;
assert(~isfield(model,'tipForceXZ'),'rod:TruthInMultiContactInput', ...
    'Tip force is unknown; do not pass it to the inverse model.');
K=numel(model.contactPlaneIndex); L=model.sMm(end); scale=model.EINmm2/L;
fbgs=input.fbgArcLengthMm(:)'; measured=input.curvaturePerMm(:)';
assert(numel(fbgs)==numel(measured) && all(isfinite([fbgs measured])) && ...
    all(diff(fbgs)>0) && fbgs(1)>=0 && fbgs(end)<=L);
normals=model.planeNormalXZ; normals=normals./vecnorm(normals);
model.planeNormalXZ=normals;
intrinsic=interp1(model.sMm(1:end-1),model.intrinsicCurvaturePerMm, ...
    fbgs,'previous','extrap');
elastic=measured-intrinsic;
s=model.sMm; e=interp1(fbgs,elastic,s,'pchip','extrap');
theta=model.baseAngleRad; p=zeros(2,numel(s)); p(:,1)=model.baseXZ;
for j=1:numel(s)-1
    ds=s(j+1)-s(j); turn=ds*(model.intrinsicCurvaturePerMm(j)+(e(j)+e(j+1))/2);
    mid=theta+turn/2;
    factor=1; if abs(turn)>1e-8, factor=sin(turn/2)/(turn/2); end
    p(:,j+1)=p(:,j)+ds*factor*[sin(mid);cos(mid)]; theta=theta+turn;
end
% Only contact count/order is supplied. Arc-length seeds and branch bounds
% come from distance minima on the shape reconstructed from observations.
partition=linspace(0,L,K+1); contactS=zeros(1,K);
for j=1:K
    plane=model.contactPlaneIndex(j);
    candidates=find(s>partition(j)&s<partition(j+1));
    g=normals(:,plane)'*(p(:,candidates)-model.planePointXZ(:,plane));
    [~,at]=min(g); contactS(j)=s(candidates(at));
end
branchEdges=[0 (contactS(1:end-1)+contactS(2:end))/2 L];
ps=interp1(s,p',fbgs)'; pc=interp1(s,p',contactS)';
G=zeros(numel(fbgs),K+2);
for j=1:K
    r=pc(:,j)-ps; n=normals(:,model.contactPlaneIndex(j));
    G(:,j)=((r(2,:)*n(1)-r(1,:)*n(2)).*(fbgs<contactS(j)))';
end
r=p(:,end)-ps; G(:,K+1)=r(2,:)'; G(:,K+2)=-r(1,:)';
linearOptions=optimoptions('lsqlin','Display','off');
forces=lsqlin(G,model.EINmm2*elastic',[],[],[],[],[zeros(K,1);-inf;-inf],[],[],linearOptions);
m0=sum((pc(2,:)-model.baseXZ(2)).*normals(1,model.contactPlaneIndex).*forces(1:K)' ...
    -(pc(1,:)-model.baseXZ(1)).*normals(2,model.contactPlaneIndex).*forces(1:K)') ...
    +(p(2,end)-model.baseXZ(2))*forces(K+1)-(p(1,end)-model.baseXZ(1))*forces(K+2);
initial=[m0/scale;forces(1:K);contactS(:)/L;forces(K+1:K+2)];
lb=[-inf;zeros(K,1);branchEdges(1:end-1)'/L+1e-6;-inf;-inf];
ub=[inf;inf(K,1);branchEdges(2:end)'/L-1e-6;inf;inf];
sigma=max(input.curvatureStdPerMm,1e-7);
options=optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-12, ...
    'OptimalityTolerance',1e-8,'StepTolerance',1e-12,'MaxIterations',100, ...
    'MaxFunctionEvaluations',3000,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-6);
timer=tic;
[x,value,residual,flag,solverOutput,~,jacobian]=lsqnonlin(@cost,initial,lb,ub,options);
contactS=x(K+2:2*K+1)'*L;
query=unique([s fbgs contactS linspace(0,L,2001)]);
state=integrate_planar_multi_contact(model,x(1)*scale,contactS,x(2:K+1),x(end-1:end),query);
audit=audit_planar_multi_contact(model,state);
estimate=struct('state',state,'audit',audit,'exitflag',flag,'objective',value, ...
    'iterations',solverOutput.iterations,'seconds',toc(timer), ...
    'maxScaledResidual',max(abs(residual)), ...
    'jacobianRank',rank(full(jacobian)),'unknownCount',numel(x), ...
    'requiresReview',~audit.passed || flag<=0 || rank(full(jacobian))<numel(x), ...
    'seedContactArcLengthMm',initial(K+2:2*K+1)'*L, ...
    'searchBoundsMm',[branchEdges(1:end-1);branchEdges(2:end)], ...
    'scope','Planar frictionless known-contact-order sparse-curvature inverse; conditional least-squares solution.');
    function r=cost(v)
        q=integrate_planar_multi_contact(model,v(1)*scale,v(K+2:2*K+1)*L, ...
            v(2:K+1),v(end-1:end),fbgs);
        n=normals(:,model.contactPlaneIndex); points=model.planePointXZ(:,model.contactPlaneIndex);
        r=[(q.curvaturePerMm-measured)'/sigma; q.tipMomentResidualNmm/1e-3; ...
            sum(n.*(q.contactPointsXZ-points),1)'/1e-3; ...
            sum(n.*q.contactTangentXZ,1)'/1e-4];
    end
end
