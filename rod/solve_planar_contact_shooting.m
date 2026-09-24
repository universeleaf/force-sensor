function result = solve_planar_contact_shooting(model,relativeTolerance)
% Continuous planar, inextensible rod with at most ONE smooth body contact.
% Piecewise constant intrinsic curvature, known end force, free end moment.
% Integrates force/moment balance independently of the LCP/inverse/energy mesh.
% Unknowns are base moment, nonnegative normal reaction and contact arclength.
% Optional signed tangential/normal force ratio prescribes a sliding branch.
% Multiple contacts, endpoint contact and unknown stick/slip are outside it.
if nargin<2,relativeTolerance=1e-10;end
assert(isnumeric(relativeTolerance)&&isreal(relativeTolerance)&&isscalar(relativeTolerance)&& ...
    isfinite(relativeTolerance)&&relativeTolerance>=1e-13&&relativeTolerance<=1e-6);
s=model.sMm(:)'; k=model.intrinsicCurvaturePerMm(:)';
assert(numel(s)>=2&&s(1)==0&&all(isfinite([s k]))&&isreal([s k])&& ...
    all(diff(s)>0)&&numel(k)==numel(s)-1,'Invalid arclength/intrinsic curvature.');
L=s(end);
EI=model.EINmm2; base=model.baseXZ(:); tip=model.tipForceXZ(:); angle=model.baseAngleRad;
assert(isscalar(EI)&&isfinite(EI)&&EI>0&&numel(base)==2&&numel(tip)==2&& ...
    isscalar(angle)&&isreal([base;tip;angle])&&all(isfinite([base;tip;angle])));
hasPlane=isfield(model,'planePointXZ')&&~isempty(model.planePointXZ);
normal=[0;0];plane=[0;0];
if hasPlane
    normal=model.planeNormalXZ(:);plane=model.planePointXZ(:);
    assert(numel(normal)==2&&numel(plane)==2&&isreal([normal;plane])&& ...
        all(isfinite([normal;plane]))&&norm(normal)>0);
    normal=normal/norm(normal);
    assert(normal'*(base-plane)>=0,'Clamped base is inside the obstacle.');
end
forceDirection=normal; ratio=0;
if isfield(model,'tangentialNormalRatio')
    ratio=model.tangentialNormalRatio;
    assert(hasPlane&&isscalar(ratio)&&isreal(ratio)&&isfinite(ratio));
    forceDirection=normal+ratio*[-normal(2);normal(1)];
end
changes=[1 find(abs(diff(k))>1e-14)+1];edges=[s(changes) L];profile=k(changes);
momentScale=EI/L;
options=optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-14, ...
    'OptimalityTolerance',1e-11,'StepTolerance',1e-13,'MaxIterations',150, ...
    'MaxFunctionEvaluations',1500,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-5);
odeOptions=odeset('RelTol',relativeTolerance,'AbsTol',relativeTolerance/10);
timer=tic;
[freeMoment,~,~,flag,output]=lsqnonlin(@freeResidual,0,[],[],options);
solution=shoot(freeMoment*momentScale,0,L);
probe=unique([linspace(0,L,1601) edges]); values=evaluate(solution,probe);
lambda=0;contact=L;mode='no-contact';
if hasPlane
    gaps=normal'*(values(1:2,:)-plane); [minimum,j]=min(gaps);
    if minimum < -1e-7
        guess=[freeMoment;1;min(1-1e-4,max(1e-4,probe(j)/L))];
        [unknown,~,~,flag,output]=lsqnonlin(@contactResidual,guess, ...
            [-inf;0;1e-6],[inf;inf;1-1e-6],options);
        lambda=unknown(2);contact=unknown(3)*L;
        solution=shoot(unknown(1)*momentScale,lambda,contact);mode='smooth-body-contact';
    end
end
nodes=evaluate(solution,s); tipMoment=nodes(4,end);
contactValue=evaluate(solution,contact);
gap=normal'*(contactValue(1:2)-plane);
tangent=[sin(contactValue(3));cos(contactValue(3))];
probe=unique([probe contact]); values=evaluate(solution,probe);
if hasPlane,penetration=max([0,-normal'*(values(1:2,:)-plane)]);else,penetration=0;end
boundaryResidual=max(abs([tipMoment;lambda*L*(normal'*tangent)]));
assert(flag>0&&boundaryResidual<1e-5&&penetration<1e-6&&abs(lambda*gap)<1e-5, ...
    'rod:ContinuousContactNotAdmissible', ...
    'Continuous single-contact solution failed equilibrium or unilateral checks; model may require multiple/endpoint contacts.');
intrinsic=zeros(size(s));
for j=1:numel(s),intrinsic(j)=profile(find(edges(1:end-1)<=s(j),1,'last'));end
result=struct('sMm',s,'thetaRad',nodes(3,:),'pXZ',nodes(1:2,:), ...
    'segmentCurvaturePerMm',diff(nodes(3,:))'./diff(s)', ...
    'curvaturePerMm',intrinsic+nodes(4,:)/EI,'momentNmm',nodes(4,:), ...
    'contactResultantXZ',forceDirection*lambda,'normalReactionN',lambda, ...
    'tangentialNormalRatio',ratio,'contactArcLengthMm',NaN, ...
    'tipForceXZ',tip,'contactPointXZ',contactValue(1:2), ...
    'exitflag',flag,'iterations',output.iterations,'stationarityInfNmm',boundaryResidual, ...
    'maxPenetrationMm',penetration,'complementarityNmm',abs(lambda*gap), ...
    'seconds',toc(timer),'mode',mode,'odeRelativeTolerance',relativeTolerance, ...
    'contactForceXZ',zeros(2,numel(s)-1), ...
    'method','Continuous planar shooting with ODE force/moment balance and one smooth frictionless body contact; conditional local root, not global stability or multi-contact proof.');
if lambda>1e-8
    result.contactArcLengthMm=contact;
    [~,j]=min(abs(s(2:end)-contact)); result.contactForceXZ(:,j)=forceDirection*lambda;
end
if ratio~=0
    result.method='Independent continuous planar shooting on a prescribed sliding branch; force balance, smooth contact tangency and nonpenetration; not a stick/slip transition solver.';
end
    function residual=freeResidual(value)
        curve=shoot(value*momentScale,0,L);last=evaluate(curve,L);
        residual=last(4)/momentScale;
    end
    function residual=contactResidual(value)
        at=value(3)*L; curve=shoot(value(1)*momentScale,value(2),at);
        last=evaluate(curve,L);touch=evaluate(curve,at);
        residual=[last(4)/momentScale;normal'*(touch(1:2)-plane)/L; ...
            normal'*[sin(touch(3));cos(touch(3))]];
    end
    function curve=shoot(moment,reaction,at)
        boundaries=unique([edges at]);state=[base;angle;moment];pieces=cell(1,numel(boundaries)-1);
        for segment=1:numel(pieces)
            mid=mean(boundaries(segment:segment+1));
            intrinsicValue=profile(find(edges(1:end-1)<=mid,1,'last'));
            force=tip+forceDirection*reaction*(mid<at);
            pieces{segment}=ode45(@balance,boundaries(segment:segment+1),state,odeOptions);
            state=pieces{segment}.y(:,end);
        end
        curve=struct('boundaries',boundaries,'pieces',{pieces});
        function rate=balance(~,state)
            theta=state(3);
            rate=[sin(theta);cos(theta);intrinsicValue+state(4)/EI; ...
                sin(theta)*force(2)-cos(theta)*force(1)];
        end
    end
    function values=evaluate(curve,queries)
        values=zeros(4,numel(queries));
        for segment=1:numel(curve.pieces)
            use=queries>=curve.boundaries(segment)&queries<=curve.boundaries(segment+1);
            if any(use),values(:,use)=deval(curve.pieces{segment},queries(use));end
        end
    end
end
