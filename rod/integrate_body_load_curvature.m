function shape = integrate_body_load_curvature(tube, contactS, localForce, localTipForce, options)
%INTEGRATE_BODY_LOAD_CURVATURE Exact body-frame Cosserat load map.
% h=R'*m, n=R'*N: h'=-u x h-e3 x n, n'=-u x n-q_local.
% Backward integration imposes h(L)=0 without a shooting loop. It retains
% calibrated intrinsic curvature, nonuniform stiffness and unobserved twist.
% Point forces jump n; Gaussian forces are continuous local transverse loads.
if nargin<5,options=struct;end
if ~isfield(options,'gaussianSigmaMm'),options.gaussianSigmaMm=[];end
if ~isfield(options,'queryArcsMm'),options.queryArcsMm=tube.s;end
if ~isfield(options,'computeShape'),options.computeShape=true;end
if ~isfield(options,'relativeTolerance'),options.relativeTolerance=2e-8;end
assert(isnumeric(options.relativeTolerance)&&isreal(options.relativeTolerance)&& ...
    isscalar(options.relativeTolerance)&&isfinite(options.relativeTolerance)&&options.relativeTolerance>0&& ...
    (islogical(options.computeShape)||isnumeric(options.computeShape))&& ...
    isreal(options.computeShape)&&isscalar(options.computeShape)&&ismember(options.computeShape,[0 1]), ...
    'rod:InvalidBodyOptions','Provide a positive tolerance and scalar logical computeShape.');
s=tube.s(:)'; K=reshape(getTubeK(tube),3,[]); u0=tube.uhat;
cs=contactS(:)'; f=reshape(localForce,3,[]); sigma=options.gaussianSigmaMm(:)';
assert(all(diff(s)>0)&&isequal(size(K),size(u0))&&all(K>0,'all')&& ...
    all(isfinite([s(:);K(:);u0(:);f(:);localTipForce(:);cs(:)]))&& ...
    numel(localTipForce)==3&&size(f,2)==numel(cs)&&all(diff(cs)>0)&& ...
    all(cs>s(1))&&all(cs<s(end)), 'rod:InvalidBodyLoad','Invalid calibrated rod or local loads.');
assert(isempty(sigma)||(numel(sigma)==numel(cs)&&all(isfinite(sigma))&&all(sigma>0)), ...
    'rod:InvalidBodyLoad','Gaussian widths must be positive, one per component.');
query=options.queryArcsMm(:)';
assert(all(isfinite(query))&&all(query>=s(1))&&all(query<=s(end)), ...
    'rod:InvalidBodyQuery','Curvature queries must be inside the calibrated rod.');
changes=find(any(abs(diff(u0,1,2))>1e-14,1)|any(abs(diff(K,1,2))>1e-10,1))+1;
edges=[s(1),s(changes),s(end)];
if isempty(sigma),edges=[edges,cs];
else
    % Segment near narrow Gaussian peaks so the adaptive ODE cannot step
    % over a concentrated force without sampling its support.
    support=cs(:)+sigma(:)*[-6 -3 0 3 6];
    edges=[edges,support(support>s(1)&support<s(end))'];
end
edges=unique(edges); pieces=cell(1,numel(edges)-1);
state=[zeros(3,1);localTipForce(:)]; rhsCount=0; intrinsic=zeros(3,1); stiffness=ones(3,1);
if ~isempty(sigma)
    area=0.5*(erf((s(end)-cs)./(sqrt(2)*sigma))-erf((s(1)-cs)./(sqrt(2)*sigma)));
    assert(all(isfinite(area))&&all(area>0),'rod:InvalidBodyLoad','Gaussian normalization must be finite and positive.');
else,area=[];end
settings=odeset('RelTol',options.relativeTolerance,'AbsTol',options.relativeTolerance/100);
for j=numel(pieces):-1:1
    midpoint=mean(edges(j:j+1)); node=find(s<=midpoint,1,'last');
    intrinsic=u0(:,node); stiffness=K(:,node);
    pieces{j}=ode45(@balance,edges([j+1 j]),state,settings); state=pieces{j}.y(:,end);
    if isempty(sigma)
        % A force at the lower edge belongs to the proximal interval only.
        take=find(cs==edges(j)); state(4:6)=state(4:6)+sum(f(:,take),2);
    end
end
q=localState(query); predicted=zeros(3,numel(query));
for j=1:numel(query)
    node=find(s<=query(j),1,'last'); predicted(:,j)=u0(:,node)+q(1:3,j)./K(:,node);
end
shape=struct('predictedCurvature',predicted,'queryArcsMm',query, ...
    'terminalMomentNmm',zeros(3,1),'rhsEvaluations',rhsCount,'localTipForce',localTipForce(:), ...
    'contactS',cs,'localContactForce',f,'gaussianSigmaMm',sigma,'p',[],'R',[],'u',[]);
if ~options.computeShape,return;end
posePieces=cell(size(pieces)); pose=[tube.T_base(1:3,4);reshape(tube.T_base(1:3,1:3),9,1)];
for j=1:numel(pieces)
    midpoint=mean(edges(j:j+1)); node=find(s<=midpoint,1,'last');
    intrinsic=u0(:,node); stiffness=K(:,node);
    posePieces{j}=ode45(@kinematics,edges(j:j+1),pose,settings); pose=posePieces{j}.y(:,end);
end
ys=poseAt(s); yc=poseAt(cs); shape.p=ys(1:3,:); shape.R=reshape(ys(4:12,:),3,3,[]);
qs=localState(s); shape.u=u0+qs(1:3,:)./K; shape.contactForce=zeros(3,numel(cs));
for j=1:numel(cs)
    if isempty(sigma)
        shape.contactForce(:,j)=reshape(yc(4:12,j),3,3)*f(:,j);
    else
        % Integrate R(s)q_j(s), rather than report its body-frame amplitude
        % as a world resultant when the robot turns across a broad Gaussian.
        pts=unique([s(1),s(end),linspace(max(s(1),cs(j)-8*sigma(j)),min(s(end),cs(j)+8*sigma(j)),161)]);
        yp=poseAt(pts); rp=reshape(yp(4:12,:),3,3,[]);
        density=exp(-0.5*((pts-cs(j))/sigma(j)).^2)/(sqrt(2*pi)*sigma(j)*area(j));
        integrand=zeros(3,numel(pts));
        for z=1:numel(pts),integrand(:,z)=rp(:,:,z)*f(:,j)*density(z);end
        shape.contactForce(:,j)=trapz(pts,integrand,2);
    end
end
shape.tipForce=shape.R(:,:,end)*localTipForce(:);
shape.baseMomentNmm=shape.R(:,:,1)*state(1:3);
    function dy=balance(arc,v)
        rhsCount=rhsCount+1;
        if rhsCount>200000,error('rod:BodyLoadBudget','Body-load ODE evaluation budget exceeded.');end
        u=intrinsic+v(1:3)./stiffness; load=zeros(3,1);
        if ~isempty(sigma)
            density=exp(-0.5*((arc-cs)./sigma).^2)./(sqrt(2*pi)*sigma.*area);
            load=f*density(:);
        end
        dy=[-cross(u,v(1:3))-cross([0;0;1],v(4:6));-cross(u,v(4:6))-load];
    end
    function values=localState(arcs)
        values=zeros(6,numel(arcs));
        for z=1:numel(pieces)
            take=arcs>=edges(z)&arcs<=edges(z+1);
            if any(take),values(:,take)=deval(pieces{z},arcs(take));end
        end
    end
    function dy=kinematics(arc,v)
        % Use this exact calibration segment, including its one-sided edge.
        qv=deval(pieces{j},arc); u=intrinsic+qv(1:3)./stiffness;
        R=reshape(v(4:12),3,3); H=[0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0];
        dR=R*H;dy=[R(:,3);dR(:)];
    end
    function values=poseAt(arcs)
        values=zeros(12,numel(arcs));
        for z=1:numel(posePieces)
            take=arcs>=edges(z)&arcs<=edges(z+1);
            if any(take),values(:,take)=deval(posePieces{z},arcs(take));end
        end
    end
end
