function state = integrate_planar_multi_contact(model, baseMomentNmm, contactS, normalForceN, tipForceXZ, queryS)
% Exact planar restriction of unshearable nonlinear Cosserat balance.
% Each contact changes the downstream load. Intrinsic curvature and loads
% are constant per integration segment; no small-angle beam approximation.
if nargin<6, queryS=model.sMm; end
s=model.sMm(:)'; k=model.intrinsicCurvaturePerMm(:)';
assert(numel(k)==numel(s)-1 && all(diff(s)>0));
contactS=contactS(:)'; normalForceN=normalForceN(:)';
assert(all(diff(contactS)>0) && numel(contactS)==numel(normalForceN));
normal=model.planeNormalXZ(:,model.contactPlaneIndex);
normal=normal./vecnorm(normal); forces=normal.*normalForceN;
changes=[1 find(abs(diff(k))>1e-14)+1];
edges=[s(changes) s(end)]; profile=k(changes);
breaks=unique([edges contactS]);
pieces=cell(1,numel(breaks)-1);
y=[model.baseXZ(:);model.baseAngleRad;baseMomentNmm];
relTol=1e-10;
if isfield(model,'relativeTolerance'), relTol=model.relativeTolerance; end
odeOptions=odeset('RelTol',relTol,'AbsTol',relTol/10);
for j=1:numel(pieces)
    mid=mean(breaks(j:j+1));
    intrinsic=profile(find(edges(1:end-1)<=mid,1,'last'));
    load=tipForceXZ(:)+sum(forces(:,contactS>mid),2);
    pieces{j}=ode45(@(t,y)[sin(y(3));cos(y(3));intrinsic+y(4)/model.EINmm2; ...
        sin(y(3))*load(2)-cos(y(3))*load(1)],breaks(j:j+1),y,odeOptions);
    y=pieces{j}.y(:,end);
end
queryS=queryS(:)';
values=evaluate(queryS); touch=evaluate(contactS);
intrinsic=zeros(size(queryS));
for j=1:numel(queryS)
    intrinsic(j)=profile(find(edges(1:end-1)<=queryS(j),1,'last'));
end
state=struct('sMm',queryS,'pXZ',values(1:2,:),'thetaRad',values(3,:), ...
    'curvaturePerMm',intrinsic+values(4,:)/model.EINmm2, ...
    'momentNmm',values(4,:),'tipMomentResidualNmm',y(4), ...
    'contactPointsXZ',touch(1:2,:),'contactTangentXZ',[sin(touch(3,:));cos(touch(3,:))], ...
    'contactForceXZ',forces,'normalForceN',normalForceN,'contactS',contactS, ...
    'tipForceXZ',tipForceXZ(:),'baseMomentNmm',baseMomentNmm);
    function values=evaluate(queries)
        values=zeros(4,numel(queries));
        for a=1:numel(pieces)
            take=queries>=breaks(a)&queries<=breaks(a+1);
            if any(take), values(:,take)=deval(pieces{a},queries(take)); end
        end
    end
end
