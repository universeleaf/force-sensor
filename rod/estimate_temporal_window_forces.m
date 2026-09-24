function result = estimate_temporal_window_forces(sensorInput, windowLength, options)
%ESTIMATE_TEMPORAL_WINDOW_FORCES Joint short-window Cosserat MAP smoother.
% The independent formulation-aligned estimate initializes a joint window.
% Every state in the window is evaluated with the nonlinear Cosserat balance
% map; the objective adds current FBG/environment likelihoods, state priors,
% temporal process factors, and predecessor contact kinematics. This is the
% complete short-window research path, while estimate_sensor_forces remains
% the stable per-frame public API.
if nargin<2||isempty(windowLength),windowLength=3;end
if nargin<3,options=struct;end
if ~isfield(options,'maxIterations'),options.maxIterations=12;end
if ~isfield(options,'maxFunctionEvaluations'),options.maxFunctionEvaluations=160;end
if ~isfield(options,'showProgress'),options.showProgress=false;end
if isfield(options,'frameIndices') && ~isempty(options.frameIndices)
    sensorInput.packet=subset_sensor_packet(sensorInput.packet,options.frameIndices);
end
base=estimate_sensor_forces(sensorInput); o=base.ours; m=base.measurements;
nt=size(o.state,2); W=min([windowLength,nt]); assert(W>=1,'Window must contain at least one frame.');
cfg=sensorInput.config; cfg.forceSensor.normalReference=m.planeNormalMeasured(:,1);
if ~isfield(cfg.forceSensor,'curvatureObservedAxes'),cfg.forceSensor.curvatureObservedAxes=[1 2];end
if ~isfield(cfg.forceSensor,'mechanicsRelativeTolerance'),cfg.forceSensor.mechanicsRelativeTolerance=2e-7;end
if ~isfield(cfg.forceSensor,'mechanicsMomentToleranceNmm'),cfg.forceSensor.mechanicsMomentToleranceNmm=2e-4;end
if ~isfield(cfg.forceSensor,'planeCollisionStepMm'),cfg.forceSensor.planeCollisionStepMm=0.5;end
if ~isfield(cfg.forceSensor,'mechanicsMaxRhsEvaluations'),cfg.forceSensor.mechanicsMaxRhsEvaluations=50000;end
if ~isfield(cfg.forceSensor,'slipToleranceMm'),cfg.forceSensor.slipToleranceMm=0.005;end
nx=size(o.state,1); xIndependent=o.state(:,1:W); y0=xIndependent(:);
stdState=zeros(nx,1); processStd=zeros(nx,1); mf=cfg.forceSensor.numFrictionDirs;
stdState(:)=stateStd(cfg.forceSensor,mf); processStd(:)=processStdVector(cfg.forceSensor,mf);
posteriorStd=zeros(nx,W);
for k=1:W, posteriorStd(:,k)=sqrt(max(diag(o.posteriorCovariance(:,:,k)),1e-8)); end
lb=repmat(-inf(nx,1),W,1);ub=repmat(inf(nx,1),W,1);
for k=1:W
    rows=(k-1)*nx+(1:nx); lb(rows(4:5))=-pi/2+1e-4;ub(rows(4:5))=pi/2-1e-4;
    lb(rows(6))=sensorInput.tube.s(1);ub(rows(6))=sensorInput.tube.s(end);
    lb(rows(7))=0;lb(rows(8:7+mf))=0;lb(rows(8+mf))=0;
    ub(rows(7))=forceUpper(cfg.forceSensor,'normalForceN',200);ub(rows(8:7+mf))=forceUpper(cfg.forceSensor,'betaN',200);
    ub(rows(8+mf))=forceUpper(cfg.forceSensor,'lambda',200);
    lim=forceUpper(cfg.forceSensor,'tipForceN',25);lb(rows(9+mf:11+mf))=-lim;ub(rows(9+mf:11+mf))=lim;
end
timer=tic; solver=struct('exitflag',0,'iterations',0,'message','not run'); y=y0;
if exist('fmincon','file')==2
    solverOptions=optimoptions('fmincon','Display','off','Algorithm','sqp', ...
        'MaxIterations',options.maxIterations,'MaxFunctionEvaluations',options.maxFunctionEvaluations, ...
        'OptimalityTolerance',1e-4,'StepTolerance',1e-6);
    try
        [y,~,solver.exitflag,solver.output]=fmincon(@jointObjective,y0,[],[],[],[],lb,ub,[],solverOptions);
        solver.iterations=solver.output.iterations;solver.message=solver.output.message;
    catch err
        solver.errorIdentifier=err.identifier;solver.message=err.message;
    end
end
elapsed=toc(timer); [states,details]=decodeWindow(y); contact=zeros(3,W);tip=contact;total=contact;arc=nan(1,W);frameSeconds=zeros(1,W);
for k=1:W
    contact(:,k)=details{k}.contactForce;tip(:,k)=details{k}.tipForce;total(:,k)=contact(:,k)+tip(:,k);arc(k)=details{k}.s1;
end
result=struct('method','short-window-cosserat-map','windowLength',W,'state',states, ...
    'contactForceResultant',contact,'tipForce',tip,'totalForceResultant',total, ...
    'contactArcLength',arc,'details',{details},'independent',base, ...
    'solver',solver,'optimizationSeconds',elapsed,'objective',jointObjective(y), ...
    'scope','Joint nonlinear Cosserat MAP window with current measurement factors, process factors, contact kinematics and complementarity penalties.');
result.quality=base.quality;result.quality.temporalWindowOptimized=true;result.quality.temporalWindowLength=W;
result.quality.temporalWindowUsesPreviousCosseratBalance=true;
if solver.exitflag<=0,result.quality.requiresReview(:)=true;end
if options.showProgress,fprintf('Temporal window: W=%d, exit %d, %.3f s\n',W,solver.exitflag,elapsed);end

    function value=jointObjective(yv)
        try
            [~,d]=decodeWindow(yv); value=0;
            for kk=1:W
                xk=yv((kk-1)*nx+(1:nx)); dk=d{kk};
                measU=m.uSparse(cfg.forceSensor.curvatureObservedAxes,:,kk); predU=dk.shape.u(cfg.forceSensor.curvatureObservedAxes,m.fbgIdx);
                e=(predU-measU); value=value+0.5*sum((e(:)/cfg.forceSensor.measurementStd.curvature).^2);
                en=[xk(1:3)-m.planePointMeasured(:,kk);dk.n-m.planeNormalMeasured(:,min(kk,size(m.planeNormalMeasured,2)))];
                envStd=[cfg.forceSensor.measurementStd.planePointMm(:);cfg.forceSensor.measurementStd.normalVector(:)];
                value=value+0.5*sum((en(:)./envStd).^2);
                value=value+0.5*sum(((xk-xIndependent(:,kk))./posteriorStd(:,kk)).^2);
                gap=max(-dk.gap,0); cone=max(-dk.coneSlack,0); comp=dk.gap*dk.fn;
                value=value+1e4*(gap^2+cone^2+comp^2)+1e-3*dk.shape.tipMomentResidualNmm^2;
                if kk>1
                    xprev=yv((kk-2)*nx+(1:nx));
                    value=value+0.5*sum(((xk-xprev)./processStd).^2);
                    vobs=observedTangentialStep(m.previousShape{kk},m.p(:,:,kk),dk.s1,dk.n,sensorInput.tube);
                    value=value+0.5*sum(((dk.vTangential-vobs)/max(cfg.forceSensor.slipToleranceMm,1e-3)).^2);
                end
            end
            if ~isfinite(value),value=1e30;end
        catch
            value=1e30;
        end
    end

    function [states,details]=decodeWindow(yv)
        states=reshape(yv,nx,W);details=cell(1,W);prev=[];
        for kk=1:W
            xk=states(:,kk);details{kk}=decodeState(xk,kk,prev);prev=details{kk}.shape;
        end
    end

    function d=decodeState(xk,kk,prev)
        n=etaToNormalPublic(xk(4:5),cfg.forceSensor.normalReference);D=frictionDirectionsPublic(n,mf);
        fn=max(0,xk(7));beta=max(0,xk(8:7+mf));lambda=max(0,xk(8+mf));fe=xk(9+mf:11+mf);
        fc=n*fn+D*beta;opts=struct('relativeTolerance',cfg.forceSensor.mechanicsRelativeTolerance, ...
            'momentToleranceNmm',cfg.forceSensor.mechanicsMomentToleranceNmm, ...
            'collisionStepMm',cfg.forceSensor.planeCollisionStepMm,'maxRhsEvaluations',cfg.forceSensor.mechanicsMaxRhsEvaluations);
        tube=sensorInput.tube;tube.T_base=m.baseTraj(:,:,kk);shape=solve_cosserat_force_map(tube,xk(6),fc,fe,opts);
        gap=n'*(shape.pc-xk(1:3));v=zeros(3,1);if ~isempty(prev),v=(eye(3)-n*n')*(shape.pc-prev.pc);end
        w=D'*v+lambda*ones(mf,1);cone=cfgFrameMu(kk)*fn-sum(beta);
        d=struct('s1',xk(6),'n',n,'fn',fn,'beta',beta,'lambda',lambda,'contactForce',fc,'tipForce',fe, ...
            'shape',shape,'pc',shape.pc,'gap',gap,'vTangential',v,'frictionW',w,'coneSlack',cone);
    end

    function mu=cfgFrameMu(kk),mu=m.frictionMu(min(kk,numel(m.frictionMu)));end
end

function stdv=stateStd(fs,m),stdv=[fs.priorStd.planePointMm(:);fs.priorStd.normalParam(:);fs.priorStd.sMm;fs.priorStd.normalForceN;fs.priorStd.betaN*ones(m,1);fs.priorStd.lambda;fs.priorStd.tipForceN*ones(3,1)];end
function stdv=processStdVector(fs,m),stdv=[fs.processStd.planePointMm(:);fs.processStd.normalParam(:);fs.processStd.sMm;fs.processStd.normalForceN;fs.processStd.betaN*ones(m,1);fs.processStd.lambda;fs.processStd.tipForceN*ones(3,1)];end
function lim=forceUpper(fs,name,default),lim=default;if isfield(fs,'useForceBounds')&&fs.useForceBounds&&isfield(fs,'forceBounds')&&isfield(fs.forceBounds,name),lim=fs.forceBounds.(name);end;end
function n=etaToNormalPublic(eta,n0),n0=n0(:)/norm(n0);B=stableBasis(n0);theta=norm(eta);if theta<1e-10,n=n0;else,n=cos(theta)*n0+sin(theta)*B*(eta(:)/theta);n=n/norm(n);end;end
function D=frictionDirectionsPublic(n,m),B=stableBasis(n);if m==1,D=contact_tangent(n);elseif m==2,D=[contact_tangent(n),-contact_tangent(n)];else,a=linspace(0,2*pi,m+1);a=a(1:end-1);D=B*[cos(a);sin(a)];end;end
function B=stableBasis(n),t=contact_tangent(n);t=t/norm(t);t2=cross(n,t);t2=t2/norm(t2);B=[t,t2];end
function v=observedTangentialStep(prev,currentP,s1,n,tube)
if isempty(prev)||~isfield(prev,'p'),v=zeros(3,1);return;end
prevP=interp1(tube.s,prev.p',min(max(s1,tube.s(1)),tube.s(end)),'linear')';
currentP=interp1(tube.s,currentP',min(max(s1,tube.s(1)),tube.s(end)),'linear')';
v=(eye(3)-n*n')*(currentP-prevP);
end
