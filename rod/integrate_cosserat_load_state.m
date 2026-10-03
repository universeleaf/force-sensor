function shape = integrate_cosserat_load_state(tube, baseMoment, contactS, contactForce, tipForce, options)
%INTEGRATE_COSSERAT_LOAD_STATE Lifted nonlinear equilibrium integration.
% The caller imposes terminalMomentNmm=0. No nested shooting/root solve and
% no frozen-shape force arms. Loads jump at continuously valued contacts.
if nargin<6, options=struct; end
if ~isfield(options,'relativeTolerance'), options.relativeTolerance=2e-8; end
if ~isfield(options,'collisionStepMm'), options.collisionStepMm=1; end
if ~isfield(options,'maxRhsEvaluations'), options.maxRhsEvaluations=100000; end
s=tube.s(:)'; K=reshape(getTubeK(tube),3,[]); u0=tube.uhat;
assert(numel(s)>=2 && all(diff(s)>0) && isequal(size(K),size(u0)) && ...
    all(K>0,'all') && all(isfinite([s(:);K(:);u0(:)])), 'rod:InvalidTube','Invalid calibrated rod.');
contactS=contactS(:)'; contactForce=reshape(contactForce,3,[]);
assert(size(contactForce,2)==numel(contactS) && ...
    all(contactS>s(1)) && all(contactS<s(end)) && ...
    all(isfinite([baseMoment(:);contactS(:);contactForce(:);tipForce(:)])), ...
    'rod:InvalidLoadState','Loads and interior contact positions must be finite.');
% Intermediate optimizer/derivative trials may violate contact ordering.
% Segment edges are sorted independently; load columns and queried material
% points keep their original identities. Ordering is a solver constraint.
assert(numel(baseMoment)==3 && numel(tipForce)==3,'rod:InvalidLoadState','Moment/tip load must be 3-vectors.');
% Piecewise-constant calibrated fields, held on [s_j,s_{j+1}). Only real
% profile changes require a new ODE segment; the sensor grid does not.
change=find(any(abs(diff(u0,1,2))>1e-14,1) | any(abs(diff(K,1,2))>1e-10,1))+1;
edges=unique([s(1) s(change) contactS s(end)]);
pieces=cell(1,numel(edges)-1); state=[tube.T_base(1:3,4);reshape(tube.T_base(1:3,1:3),9,1);baseMoment(:)];
rhsCount=0; intrinsic=zeros(3,1); stiffness=ones(3,1); load=zeros(3,1);
odeOptions=odeset('RelTol',options.relativeTolerance,'AbsTol',options.relativeTolerance/100);
for j=1:numel(pieces)
    midpoint=mean(edges(j:j+1)); node=find(s<=midpoint,1,'last');
    intrinsic=u0(:,node); stiffness=K(:,node);
    load=tipForce(:)+sum(contactForce(:,contactS>midpoint),2);
    pieces{j}=ode45(@balance,edges(j:j+1),state,odeOptions); state=pieces{j}.y(:,end);
end
% Append continuous contacts with stable row identities. Keep duplicates:
% unique/sorting would merge or permute constraint rows as contacts migrate.
collisionS=[linspace(s(1),s(end),ceil((s(end)-s(1))/options.collisionStepMm)+1),contactS];
y=evaluate(s); yc=evaluate(contactS); ys=evaluate(collisionS);
R=reshape(y(4:12,:),3,3,[]); u=u0;
for j=1:numel(s), u(:,j)=u0(:,j)+(R(:,:,j)'*y(13:15,j))./K(:,j); end
shape=struct('p',y(1:3,:),'u',u,'R',R,'sMm',s,'momentNmm',y(13:15,:), ...
    'contactPoints',yc(1:3,:),'contactTangents',yc(10:12,:), ...
    'contactS',contactS,'collisionS',collisionS,'collisionP',ys(1:3,:), ...
    'baseMomentNmm',baseMoment(:),'terminalMomentNmm',state(13:15), ...
    'tipMomentResidualNmm',norm(state(13:15),inf),'rhsEvaluations',rhsCount, ...
    'pieces',{pieces},'segmentEdges',edges,'mechanics','lifted-nonlinear-cosserat');
    function dy=balance(~,q)
        rhsCount=rhsCount+1;
        if rhsCount>options.maxRhsEvaluations
            error('rod:CosseratEquilibrium','Lifted integration RHS budget exceeded.');
        end
        rot=reshape(q(4:12),3,3); tangent=rot(:,3);
        v=intrinsic+(rot'*q(13:15))./stiffness;
        hat=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
        dR=rot*hat; dy=[tangent;dR(:);-cross(tangent,load)];
    end
    function q=evaluate(arcs)
        q=zeros(15,numel(arcs));
        for z=1:numel(pieces)
            take=arcs>=edges(z)&arcs<=edges(z+1);
            if any(take), q(:,take)=deval(pieces{z},arcs(take)); end
        end
    end
end
