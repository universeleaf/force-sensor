function result = estimate_formulation_window(input, options)
%ESTIMATE_FORMULATION_WINDOW Joint 3-D multi-contact Cosserat MPCC/MAP.
% Formulation (3)-(23), generalized to a measured half-space environment and
% several candidate contacts. Every frame has its own equilibrium state.
% No truth forces, contact locations, dense truth shapes or mode labels enter.
if nargin<2, options=struct; end
validate_lcp_dependency(fileparts(fileparts(mfilename('fullpath'))));
options=defaults(options);
obs=formulation_window_observations(input); tube=input.tube; fs=input.config.forceSensor;
[candidates,reconstructed,candidateAudit]=formulation_contact_candidates(tube,obs,options);
if isfield(options,'contactPlaneIndex')
    error('rod:ContactLabelsNotAllowed','Contact count/order labels are not accepted. Configure candidate search instead.');
end
m=options.numFrictionDirs; K=numel(candidates.planeIndex); T=obs.frameCount;
spec=formulation_window_spec(obs.planeCount,candidates.planeIndex,m); nx=spec.stateLength;
[arcLower,arcUpper,physicalA,physicalB]=formulation_contact_arc_domain( ...
    tube.s,candidates.seedArcMm,spec,T,options.minArcSeparationMm,options.contactArcMode);
stiffness=reshape(getTubeK(tube),3,[]); momentScale=mean(stiffness(1:2,:),'all')/(tube.s(end)-tube.s(1));
[x0,lb,ub,scale,priorStd,processStd]=initialState();
assert(all(isfinite(priorStd))&&all(priorStd>0)&&all(isfinite(processStd))&&all(processStd>0), ...
    'rod:InvalidWindowPrior','Prior and process standard deviations must be finite and positive.');
warmBoundCorrection=0;
if isfield(options,'initialState')
    z=options.initialState;
    boundRoundoff=1e-9*max(1,abs(z));
    assert(isequal(size(z),size(x0))&&isreal(z)&&all(isfinite(z),'all')&& ...
        all(z>=lb-boundRoundoff,'all')&&all(z<=ub+boundRoundoff,'all'),'rod:InvalidWindowInitialState', ...
        'Warm state must match inferred state coordinates/times and obey bounds.');
    x0=max(lb,min(ub,z)); warmBoundCorrection=max(abs(x0-z),[],'all');
    assert(all(physicalA*x0(:)<=physicalB+1e-9),'rod:InvalidWindowInitialState', ...
        'Warm contacts must preserve their arc order and minimum separation.');
    % Initialization only; never converted into another posterior factor.
end
rows=spec.physicalState; processW=diag(1./processStd(rows));
if isfield(options,'processCovariance')
    Cq=options.processCovariance;
    assert(isequal(size(Cq),[numel(rows) numel(rows)])&&isreal(Cq)&&all(isfinite(Cq),'all')&& ...
        norm(Cq-Cq','fro')<1e-10*max(norm(Cq,'fro'),eps), ...
        'rod:InvalidWindowProcessCovariance','Provide a finite symmetric physical-state process covariance.');
    [Lq,qflag]=chol(Cq,'lower');
    assert(qflag==0,'rod:InvalidWindowProcessCovariance','Process covariance must be positive definite.');
    processW=Lq\eye(numel(rows));
end
free=find(ub(:)>lb(:)); fixed=x0(:); sc=scale(free); origin=x0(free);
lo=(lb(free)-origin)./sc; hi=(ub(free)-origin)./sc; y=zeros(size(free));
linearA=full(physicalA(:,free)*spdiags(sc,0,numel(sc),numel(sc)));linearB=physicalB-physicalA*fixed;
cachedY=[]; cachedFrames={}; rejected=0; lastRejection='';
derivativeY=[];derivativeCache=[];derivativeEvaluations=0;derivativeCacheHits=0;
% Geometry-only changes do not change the rod IVP. Finite differences also
% change only one time state: retain exact mechanical states per time rather
% than integrating the entire window again for each perturbed coordinate.
mechanicalCache=repmat({{}},1,T); mechanicalEvaluations=0; mechanicalCacheHits=0;
seedWhitening=obs.curvatureWhitening;
timer=tic; stages=cell(1,numel(options.relaxations));
[initialA,~,initialC,initialE,boundedA,boundedB,constantZeroPairs]=physicalConstraints(decode(y));
% fn, beta and lambda already have exact nonnegative box bounds. Repeating
% their rows as nonlinear inequalities creates dependent active constraints.
nonnegativeA=~boundedA;nonnegativeB=~boundedB;
constraintCount=numel(initialC)+3*numel(initialA);
equalityCount=numel(initialE);
seedResidualCount=numel(seedResidual(y));
seedOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',options.seedIterations, ...
    'MaxFunctionEvaluations',options.maxFunctionEvaluations,'FunctionTolerance',1e-9, ...
    'StepTolerance',1e-9,'OptimalityTolerance',1e-7,'FiniteDifferenceStepSize',1e-5);
seedTrace=cell(1,numel(options.seedCurvatureFloorsPerMm));
for seedStage=1:numel(options.seedCurvatureFloorsPerMm)
    floorStd=options.seedCurvatureFloorsPerMm(seedStage);
    for kk=1:T
        invW=obs.curvatureWhitening(:,:,kk)\eye(size(obs.curvatureWhitening,1));
        Cseed=invW*invW'+floorStd^2*eye(size(invW));
        Lseed=chol(Cseed,'lower'); seedWhitening(:,:,kk)=Lseed\eye(size(invW));
    end
    % The initializer uses trust-region least squares. Linear ordering stays
    % HARD in the final MAP; here it has an explicit feasibility penalty and
    % projection, avoiding a rank-deficient interior-point seed KKT system.
    [y,seedNorm,~,seedFlag,seedOut]=lsqnonlin(@safeSeed,y,lo,hi,seedOptions);
    y=projectArcInitializer(y);
    seedTrace{seedStage}=struct('curvatureFloorPerMm',floorStd,'exitflag',seedFlag, ...
        'iterations',seedOut.iterations,'residualSquaredNorm',seedNorm);
    if options.showProgress
        fprintf('  initialization floor=%g/mm: exit=%d, iterations=%d, residual=%.4g\n', ...
            floorStd,seedFlag,seedOut.iterations,seedNorm);
    end
end
% For a frictionless frame beta=cone=0, lambda is a free dual slack.
% Give its initializer a feasible interior value instead of leaving w a few
% roundoff units negative. Lambda remains an optimized MAP variable.
seedFrames=decode(y); seedX=expand(y);
for kk=1:T
    for pp=1:obs.planeCount
        active=find(candidates.planeIndex==pp & seedFrames{kk}.normalForce>options.activeForceThresholdN);
        if ~isempty(active)
            ix=spec.plane(pp);
            seedX(ix.point,kk)=seedX(ix.point,kk)+seedFrames{kk}.normal(:,pp)*mean(seedFrames{kk}.gap(active));
        end
    end
    for jj=1:K
        ix=spec.contact(jj);
        if options.useFrictionHistory&&obs.mu(ix.plane,kk)==0&&obs.predecessorIndex(kk)>0
            slip=seedFrames{kk}.directions{ix.plane}'*seedFrames{kk}.tangentialStepMm(:,jj);
            seedX(ix.lambda,kk)=min(options.maxSlipMm,max([0;-slip])+1e-4);
        end
    end
end
y=(seedX(free)-origin)./sc;
solverOptions=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',options.maxIterations,'MaxFunctionEvaluations',options.maxFunctionEvaluations, ...
    'ConstraintTolerance',options.constraintTolerance,'OptimalityTolerance',1e-5,'StepTolerance',1e-8, ...
    'FiniteDifferenceStepSize',1e-5,'FiniteDifferenceType','central','SpecifyObjectiveGradient',true, ...
    'SpecifyConstraintGradient',options.useSharedDerivatives);
if options.showProgress, solverOptions.OutputFcn=@progress; end
best=struct('available',false,'y',y,'objective',inf,'exitflag',0,'stage',0);
usedInteriorRetry=false;polishTrace={};branchA=[];branchB=[];branchScaleA=[];branchScaleB=[];
for stage=1:numel(options.relaxations)
    tau=options.relaxations(stage); stageTimer=tic;
    [y,value,flag,out]=fmincon(@objective,y,linearA,linearB,[],[],lo,hi,@constraints,solverOptions);
    sqpExit=flag; retry=[];
    if flag<=0&&options.retryInteriorPoint&&~usedInteriorRetry&&~best.available
        % Planar/zero-friction branches can have linearly dependent active
        % rows. Try another constrained optimizer on the SAME MAP and MPCC;
        % record both exits, rather than accepting a failed SQP as success.
        retryOptions=solverOptions; retryOptions.Algorithm='interior-point';
        usedInteriorRetry=true;
        retryOptions.MaxIterations=options.retryIterations;
        [trial,vTrial,fTrial,oTrial]=fmincon(@objective,y,linearA,linearB,[],[],lo,hi,@constraints,retryOptions);
        [tc,te]=constraints(trial); retryViolation=max([0;tc;abs(te);linearA*trial-linearB]);
        retry=struct('algorithm','interior-point','exitflag',fTrial,'iterations',oTrial.iterations, ...
            'objective',vTrial,'constraintViolation',retryViolation,'message',oTrial.message);
        if fTrial>0 || (flag<=0&&retryViolation<out.constrviolation)
            y=trial; value=vTrial; flag=fTrial; out=oTrial;
        end
    end
    [c,e]=constraints(y);
    stages{stage}=struct('relaxation',tau,'exitflag',flag,'iterations',out.iterations, ...
        'functionCount',out.funcCount,'objective',value,'constraintViolation',max([0;c;abs(e);linearA*y-linearB]), ...
        'seconds',toc(stageTimer),'message',out.message,'firstOrderOptimality',out.firstorderopt);
    stages{stage}.sqpExitflag=sqpExit; stages{stage}.retry=retry;
    [aa,bb,cc,ee]=physicalConstraints(decode(y));
    unrelaxed=max([0;-aa;-bb;abs(aa.*bb);cc;abs(ee);linearA*y-linearB]);
    stages{stage}.unrelaxedViolation=unrelaxed;
    stages{stage}.accepted=flag>0&&unrelaxed<=options.complementarityTolerance;
    if options.polishActiveBranch&&~stages{stage}.accepted&& ...
            (flag>0||stage==1||stage==numel(options.relaxations))
        % The original MPCC is a union of active branches. On a branch,
        % a*b=0 is represented by a=0 or b=0, avoiding its vanishing gradient.
        % A selected branch is never accepted without checking ALL original
        % inequalities/complementarity and a positive optimizer exit.
        [polished,pvalue,pflag,pout]=polishBranch(y);
        [pa,pb,pc,pe]=physicalConstraints(decode(polished));
        pviolation=max([0;-pa;-pb;abs(pa.*pb);pc;abs(pe);linearA*polished-linearB]);
        polishTrace{end+1}=struct('afterStage',stage,'exitflag',pflag,'iterations',pout.iterations, ...
            'objective',pvalue,'unrelaxedViolation',pviolation,'sideA',branchA,'sideB',branchB, ...
            'constraintTolerance',pout.branchConstraintTolerance,'stepTolerance',pout.branchStepTolerance, ...
            'constraintViolation',pout.constrviolation,'firstOrderOptimality',pout.firstorderopt, ...
            'message',pout.message,'restoration',pout.restoration, ...
            'accepted',pflag>0&&pviolation<=options.complementarityTolerance); %#ok<AGROW>
        if polishTrace{end}.accepted
            y=polished;value=pvalue;flag=pflag;unrelaxed=pviolation;
        end
    end
    if flag>0&&unrelaxed<=options.complementarityTolerance&&value<best.objective
        best=struct('available',true,'y',y,'objective',value,'exitflag',flag,'stage',stage);
    elseif best.available
        % A later continuation step can fail because of a degenerate active
        % branch. Preserve the earlier solution of the ORIGINAL constraints;
        % never replace it with a worse/infeasible trial or rewrite its exit.
        y=best.y;
    end
    if options.showProgress
        fprintf('Window K=%d T=%d tau=%g: exit=%d, objective=%.4g, violation=%.3g, %.1fs\n', ...
            K,T,tau,flag,value,stages{stage}.constraintViolation,stages{stage}.seconds);
    end
    if flag>0&&unrelaxed<=options.exactContinuationTolerance
        % Already feasible for the original (unrelaxed) MPCC, to a tighter
        % threshold than the audit. Do not spend further stages approaching
        % constraints which this solution already satisfies.
        stages=stages(1:stage); break;
    end
end
selectedStage=stage;
if best.available, y=best.y; flag=best.exitflag; selectedStage=best.stage; end
frames=decode(y); X=expand(y); quality=audit(frames,flag);
contact=zeros(3,K,T); tip=zeros(3,T); arcs=zeros(K,T); active=false(K,T);
for k=1:T
    contact(:,:,k)=frames{k}.contactForce; tip(:,k)=frames{k}.tipForce;
    arcs(:,k)=frames{k}.contactS(:); active(:,k)=frames{k}.normalForce(:)>options.activeForceThresholdN;
end
indices=obs.outputIndices;
result=struct('method','joint-3d-multi-contact-cosserat-mpcc-map', ...
    'state',X(:,indices),'stateSpec',spec,'contactPlaneIndex',candidates.planeIndex, ...
    'contactForce',contact(:,:,indices),'contactForceResultant',reshape(sum(contact(:,:,indices),2),3,[]), ...
    'tipForce',tip(:,indices),'totalForceResultant',reshape(sum(contact(:,:,indices),2),3,[])+tip(:,indices), ...
    'contactArcLength',arcs(:,indices),'activeContacts',active(:,indices), ...
    'frames',{frames(indices)},'timeSeconds',obs.timeSeconds(indices), ...
    'windowState',X,'windowFrames',{frames},'windowTimeSeconds',obs.timeSeconds, ...
    'candidateAudit',candidateAudit,'quality',subsetQuality(quality,indices), ...
    'objective',objective(y),'optimizationSeconds',toc(timer), ...
    'solver',struct('seedExitflag',seedFlag,'seedIterations',seedOut.iterations,'seedTrace',{seedTrace}, ...
        'stages',{stages},'exitflag',flag,'selectedStage',selectedStage, ...
        'activeBranchPolish',{polishTrace}, ...
        'warmStateBoundCorrection',warmBoundCorrection, ...
        'mechanicalEvaluations',mechanicalEvaluations,'mechanicalCacheHits',mechanicalCacheHits, ...
        'derivativeBundleEvaluations',derivativeEvaluations,'derivativeBundleCacheHits',derivativeCacheHits, ...
        'rejectedTrials',rejected,'lastRejection',lastRejection), ...
    'options',options,'observations',obs, ...
    'scope','Full nonlinear equilibrium at all window times; full polyhedral friction MPCC after the first time; bounded shape-driven candidate set; half-space environment.');
result.uncertainty=struct('computed',false,'coverageValidated',false,'scope', ...
    'Local linear covariance on an inferred active branch; mode/candidate mixtures and model calibration are not marginalized.');
if options.computeCovariance
    result.uncertainty=localCovariance(y,frames);
    result.quality.forceSeparationAssessed=true;
    unresolved=reshape(result.uncertainty.forceUnresolvedByData,3*(K+1),[]);
    result.quality.hasForceSeparationWarning=any(unresolved,1);
    result.quality.requiresReview=result.quality.requiresReview|result.quality.hasForceSeparationWarning;
else
    result.quality.forceSeparationAssessed=false;
end
result.observationFit=audit_formulation_window_fit(result);
result.quality.hasObservationFitWarning=result.observationFit.requiresReview;
result.quality.requiresReview=result.quality.requiresReview|result.quality.hasObservationFitWarning;
% Include covariance work as well as optimization in the reported work count.
result.solver.mechanicalEvaluations=mechanicalEvaluations;result.solver.mechanicalCacheHits=mechanicalCacheHits;
result.solver.derivativeBundleEvaluations=derivativeEvaluations;result.solver.derivativeBundleCacheHits=derivativeCacheHits;
result.endToEndSeconds=toc(timer);
result.factorConfiguration=struct('contactGeometry',options.useContactGeometry, ...
    'frictionHistory',options.useFrictionHistory,'temporalPrior',options.useTemporalPrior, ...
    'environmentLikelihood',options.useEnvironmentLikelihood,'contactArcMode',options.contactArcMode);

    function [X,lower,upper,S,Pstd,Qstd]=initialState()
        X=zeros(nx,T); lower=-inf(nx,T); upper=inf(nx,T); S=ones(nx,T);
        Pstd=ones(nx,1)*1e6; Qstd=Pstd;
        for jj=1:obs.planeCount
            ix=spec.plane(jj); X(ix.point,:)=reshape(obs.point(:,jj,:),3,T);
            S(ix.point,:)=1; S(ix.eta,:)=0.02;
            lower(ix.eta,:)=-options.normalChartRadiusRad; upper(ix.eta,:)=options.normalChartRadiusRad;
            % Initial camera observations are initializers, not a second
            % geometry likelihood disguised as a prior.
            Pstd(ix.point)=1e6; Pstd(ix.eta)=1e6;
            Qstd(ix.point)=fs.processStd.planePointMm; Qstd(ix.eta)=fs.processStd.normalParam;
        end
        for jj=1:K
            ix=spec.contact(jj);X(ix.s,:)=max(arcLower(jj),min(arcUpper(jj),candidates.seedArcByFrameMm(jj,:)));S(ix.s,:)=10;
            lower(ix.s,:)=arcLower(jj);upper(ix.s,:)=arcUpper(jj);
            lower([ix.fn ix.beta ix.lambda],:)=0;
            upper(ix.fn,:)=options.maxForceN; upper(ix.beta,:)=options.maxForceN; upper(ix.lambda,:)=options.maxSlipMm;
            Pstd(ix.s)=tube.s(end)-tube.s(1); Pstd(ix.fn)=fs.priorStd.normalForceN;
            Pstd(ix.beta)=fs.priorStd.betaN; Pstd(ix.lambda)=fs.priorStd.lambda;
            Qstd(ix.s)=fs.processStd.sMm; Qstd(ix.fn)=fs.processStd.normalForceN;
            Qstd(ix.beta)=fs.processStd.betaN; Qstd(ix.lambda)=fs.processStd.lambda;
            for kk=1:T
                if obs.mu(ix.plane,kk)==0
                    upper(ix.beta,kk)=0;
                end
                if obs.predecessorIndex(kk)==0||~options.useFrictionHistory, upper(ix.lambda,kk)=0; end
            end
        end
        lower(spec.tip,:)=-options.maxTipForceN; upper(spec.tip,:)=options.maxTipForceN;
        Pstd(spec.tip)=fs.priorStd.tipForceN; Qstd(spec.tip)=fs.processStd.tipForceN;
        S(spec.baseMoment,:)=momentScale;
        % Project tracked guesses onto the ordered polytope without swapping
        % plane identities. This changes initialization, not observations.
        for kk=1:T
            for jj=2:K
                X(spec.contact(jj).s,kk)=max(X(spec.contact(jj).s,kk),X(spec.contact(jj-1).s,kk)+options.minArcSeparationMm);
            end
            for jj=K-1:-1:1
                X(spec.contact(jj).s,kk)=min(X(spec.contact(jj).s,kk),X(spec.contact(jj+1).s,kk)-options.minArcSeparationMm);
            end
        end
        % Observation-only linear moment fitting is an initializer, not a
        % likelihood or prior. The final solve re-integrates current geometry.
        for kk=1:T
            sh=reconstructed{kk}; na=numel(obs.axes); nf=K*(1+m)+3;
            A=zeros(na*numel(tube.s),nf); target=zeros(na*numel(tube.s),1);
            Blo=-inf(nf,1); Bhi=inf(nf,1); Bcone=zeros(K,nf); seedD=cell(1,K);
            for jj=1:K
                plane=spec.contact(jj).plane; columns=(jj-1)*(m+1)+(1:m+1);
                [n,D]=formulation_contact_frame(obs.normal(:,plane,1),[0;0],m); seedD{jj}=[n D];
                Blo(columns)=0; Bhi(columns)=options.maxForceN;
                if obs.mu(plane,kk)==0, Bhi(columns(2:end))=0; end
                Bcone(jj,columns)=[-obs.mu(plane,kk),ones(1,m)];
            end
            Blo(end-2:end)=-options.maxTipForceN; Bhi(end-2:end)=options.maxTipForceN;
            for zz=1:numel(tube.s)
                rows=(zz-1)*na+(1:na); point=sh.p(:,zz); projection=sh.R(:,obs.axes,zz)';
                target(rows)=stiffness(obs.axes,zz).*(sh.u(obs.axes,zz)-tube.uhat(obs.axes,zz));
                for jj=1:K
                    if tube.s(zz)<X(spec.contact(jj).s,kk)
                        pc=interp1(tube.s,sh.p',X(spec.contact(jj).s,kk))';
                        columns=(jj-1)*(m+1)+(1:m+1);
                        A(rows,columns)=projection*hat(pc-point)*seedD{jj};
                    end
                end
                arm=sh.p(:,end)-point; A(rows,end-2:end)=projection*hat(arm);
            end
            fitOpts=optimoptions('lsqlin','Display','off');
            f=lsqlin([A;0.01*eye(nf)],[target;zeros(nf,1)],Bcone,zeros(K,1),[],[],Blo,Bhi,[],fitOpts);
            moment=zeros(3,1);
            for jj=1:K
                columns=(jj-1)*(m+1)+(1:m+1); ix=spec.contact(jj);
                X(ix.fn,kk)=f(columns(1)); X(ix.beta,kk)=f(columns(2:end));
                pc=interp1(tube.s,sh.p',X(ix.s,kk))';
                moment=moment+cross(pc-sh.p(:,1),seedD{jj}*f(columns));
            end
            X(spec.tip,kk)=f(end-2:end);
            X(spec.baseMoment,kk)=moment+cross(sh.p(:,end)-sh.p(:,1),f(end-2:end));
        end
    end
    function X=expand(q)
        v=fixed; v(free)=origin+sc.*q; X=reshape(v,nx,T);
    end
    function q=projectArcInitializer(q)
        xx=expand(q);
        for kk=1:T
            for jj=1:K
                ix=spec.contact(jj);xx(ix.s,kk)=max(arcLower(jj),min(arcUpper(jj),xx(ix.s,kk)));
            end
            for jj=2:K
                xx(spec.contact(jj).s,kk)=max(xx(spec.contact(jj).s,kk),xx(spec.contact(jj-1).s,kk)+options.minArcSeparationMm);
            end
            for jj=K-1:-1:1
                xx(spec.contact(jj).s,kk)=min(xx(spec.contact(jj).s,kk),xx(spec.contact(jj+1).s,kk)-options.minArcSeparationMm);
            end
        end
        q=(xx(free)-origin)./sc;
    end
    function frames=decode(q)
        assert(isequal(size(q),size(free)),'rod:InvalidOptimizerState','Internal optimizer state has the wrong dimension.');
        assert(isreal(q)&&all(isfinite(q)),'rod:InvalidOptimizerTrial','Optimizer trial is nonfinite or complex.');
        if isequal(q,cachedY), frames=cachedFrames; return; end
        X=expand(q); frames=cell(1,T);
        for kk=1:T
            x=X(:,kk); normals=zeros(3,obs.planeCount); points=normals; directions=cell(1,obs.planeCount);
            for jj=1:obs.planeCount
                ix=spec.plane(jj); points(:,jj)=x(ix.point);
                [normals(:,jj),directions{jj}]=formulation_contact_frame(obs.normal(:,jj,1),x(ix.eta),m);
            end
            s=zeros(1,K); force=zeros(3,K); fn=zeros(1,K); beta=zeros(m,K); lambda=zeros(1,K);
            for jj=1:K
                ix=spec.contact(jj); s(jj)=x(ix.s); fn(jj)=x(ix.fn); beta(:,jj)=x(ix.beta); lambda(jj)=x(ix.lambda);
                force(:,jj)=normals(:,ix.plane)*fn(jj)+directions{ix.plane}*beta(:,jj);
            end
            rod=tube; rod.T_base=obs.basePose(:,:,kk);
            signature=[x(spec.baseMoment);s(:);force(:);x(spec.tip)];
            assert(all(s>tube.s(1))&&all(s<tube.s(end)), ...
                'rod:InvalidOptimizerTrial','Optimizer trial contact lies outside the rod interior.');
            hit=0; entries=mechanicalCache{kk};
            if options.cacheMechanics
                for zz=1:numel(entries)
                    if isequal(signature,entries{zz}.signature), hit=zz; break; end
                end
            end
            if hit>0
                sh=entries{hit}.shape; predicted=entries{hit}.predicted;
                mechanicalCacheHits=mechanicalCacheHits+1;
                entries=[entries(hit),entries(1:hit-1),entries(hit+1:end)];
            else
                sh=integrate_cosserat_load_state(rod,x(spec.baseMoment),s,force,x(spec.tip),options);
                mechanicalEvaluations=mechanicalEvaluations+1;
                [~,sensorR]=cosserat_state_at_arc(sh,obs.arcs);
                predicted=zeros(numel(obs.axes),numel(obs.arcs));
                for zz=1:numel(obs.arcs)
                    node=find(tube.s<=obs.arcs(zz),1,'last');
                    segment=find(sh.segmentEdges<=obs.arcs(zz),1,'last'); segment=min(segment,numel(sh.pieces));
                    state=deval(sh.pieces{segment},obs.arcs(zz));
                    curv=tube.uhat(:,node)+(sensorR(:,:,zz)'*state(13:15))./stiffness(:,node);
                    predicted(:,zz)=curv(obs.axes);
                end
                if options.cacheMechanics
                    entries=[{struct('signature',signature,'shape',sh,'predicted',predicted)},entries];
                    entries=entries(1:min(8,numel(entries)));
                end
            end
            mechanicalCache{kk}=entries;
            v=zeros(3,K); gap=zeros(1,K); tangent=gap; cone=gap; w=zeros(m,K);
            previousIndex=obs.predecessorIndex(kk);
            if previousIndex>0, previousP=cosserat_state_at_arc(frames{previousIndex}.shape,s);
            else, previousP=sh.contactPoints; end
            for jj=1:K
                plane=spec.contact(jj).plane; n=normals(:,plane);
                gap(jj)=n'*(sh.contactPoints(:,jj)-points(:,plane));
                tangent(jj)=n'*sh.contactTangents(:,jj);
                v(:,jj)=(eye(3)-n*n')*(sh.contactPoints(:,jj)-previousP(:,jj));
                w(:,jj)=directions{plane}'*v(:,jj)+lambda(jj);
                cone(jj)=obs.mu(plane,kk)*fn(jj)-sum(beta(:,jj));
            end
            rodGap=zeros(obs.planeCount,numel(sh.collisionS));
            for jj=1:obs.planeCount, rodGap(jj,:)=normals(:,jj)'*(sh.collisionP-points(:,jj)); end
            frames{kk}=struct('shape',sh,'point',points,'normal',normals,'directions',{directions}, ...
                'contactS',s,'contactForce',force,'tipForce',x(spec.tip),'normalForce',fn, ...
                'beta',beta,'lambda',lambda,'gap',gap,'tangency',tangent,'tangentialStepMm',v, ...
                'frictionW',w,'coneSlack',cone,'rodGap',rodGap,'predictedCurvature',predicted);
        end
        cachedY=q; cachedFrames=frames;
    end
    function [r,measurementRows]=residual(q,curvatureW)
        if nargin<2, curvatureW=obs.curvatureWhitening; end
        frames=decode(q); X=expand(q); r=[];measurementRows=[];
        for kk=1:T
            e=frames{kk}.predictedCurvature-obs.u(:,:,kk);
            start=numel(r);r=[r;curvatureW(:,:,kk)*e(:)]; %#ok<AGROW>
            if options.useEnvironmentLikelihood&&obs.environmentLikelihood(kk)
                for jj=1:obs.planeCount
                    e=[frames{kk}.point(:,jj)-obs.point(:,jj,kk);frames{kk}.normal(:,jj)-obs.normal(:,jj,kk)];
                    r=[r;obs.environmentWhitening(:,:,jj,kk)*e]; %#ok<AGROW>
                end
            end
            measurementRows=[measurementRows,start+1:numel(r)]; %#ok<AGROW>
            if kk>1&&options.useTemporalPrior
                dt=(obs.timeSeconds(kk)-obs.timeSeconds(kk-1))/obs.referencePeriodSeconds;
                rows=spec.physicalState;
                r=[r;processW*(X(rows,kk)-X(rows,kk-1))/sqrt(dt)]; %#ok<AGROW>
            end
        end
        % One weak initial physical prior, never a posterior from these data.
        rows=spec.physicalState; prior=zeros(nx,1);
        for jj=1:K, prior(spec.contact(jj).s)=tube.s(1)+(tube.s(end)-tube.s(1))*jj/(K+1); end
        if isfield(options,'initialPrior')
            ip=options.initialPrior; assert(numel(ip.mean)==numel(rows)&& ...
                isequal(size(ip.covariance),[numel(rows) numel(rows)]), ...
                'rod:InvalidInitialPrior','Prior covers physical coordinates, excluding lifted base moments.');
            [Lprior,pflag]=chol(ip.covariance,'lower');
            assert(pflag==0&&all(isfinite(ip.mean))&&norm(ip.covariance-ip.covariance','fro')<1e-8, ...
                'rod:InvalidInitialPrior','Provide a finite mean and symmetric positive definite prior.');
            r=[r;Lprior\(X(rows,1)-ip.mean(:))];
        else, r=[r;(X(rows,1)-prior(rows))./priorStd(rows)]; end
    end
    function r=seedResidual(q)
        frames=decode(q); r=residual(q,seedWhitening);
        for kk=1:T
            d=frames{kk};r=[r;d.shape.terminalMomentNmm/options.seedMomentStdNmm;min(d.coneSlack(:),0)/0.01; ...
                min(diff(d.contactS(:))-options.minArcSeparationMm,0)/options.seedGapStdMm]; %#ok<AGROW>
            if options.useContactGeometry
                r=[r;d.gap(:)/options.seedGapStdMm;d.tangency(:)/options.seedTangencyStd; ...
                    min(d.rodGap(:),0)/options.seedGapStdMm]; %#ok<AGROW>
            end
            if options.useFrictionHistory,r=[r;min(d.frictionW(:),0)/options.seedGapStdMm];end %#ok<AGROW>
        end
    end
    function [v,g]=objective(q)
        try
            r=residual(q); v=0.5*(r'*r);
            if nargout>1
                if options.useSharedDerivatives,d=derivatives(q);J=d.Jr;
                else,J=numericalJacobian(@residual,q);end
                g=J'*r;
            end
        catch err
            rejectTrial(err); v=1e30; if nargout>1, g=zeros(size(q)); end
        end
    end
    function [a,b,c,e,boundedA,boundedB,constantZeroPairs]=physicalConstraints(frames)
        a=[]; b=[]; c=[]; e=[];boundedA=false(0,1);boundedB=false(0,1);constantZeroPairs=false(0,1);
        for kk=1:T
            d=frames{kk};e=[e;d.shape.terminalMomentNmm/momentScale]; %#ok<AGROW>
            if options.useContactGeometry
                a=[a;d.gap(:)/options.lengthScaleMm];b=[b;d.normalForce(:)/options.forceScaleN]; %#ok<AGROW>
                boundedA=[boundedA;false(K,1)];boundedB=[boundedB;true(K,1)]; %#ok<AGROW>
                constantZeroPairs=[constantZeroPairs;false(K,1)]; %#ok<AGROW>
                mask=true(size(d.rodGap));gridCount=size(mask,2)-K;
                for jj=1:K,mask(candidates.planeIndex(jj),gridCount+jj)=false;end
                % Assigned-plane candidate gaps already occur in a>=0.
                % Retain every OTHER plane at those continuous positions.
                c=[c;-d.rodGap(mask)/options.lengthScaleMm];e=[e;d.tangency(:).*d.normalForce(:)/options.forceScaleN]; %#ok<AGROW>
            end
            if options.useFrictionHistory
                a=[a;d.frictionW(:)/options.lengthScaleMm;d.lambda(:)/options.lengthScaleMm]; %#ok<AGROW>
                b=[b;d.beta(:)/options.forceScaleN;d.coneSlack(:)/options.forceScaleN]; %#ok<AGROW>
                noHistory=obs.predecessorIndex(kk)==0;zeroMu=obs.mu(candidates.planeIndex,kk)==0;zeroMu=zeroMu(:);
                % With no predecessor, w=lambda=0 identically. At mu=0,
                % beta=cone=0 identically. Their products impose no new row.
                boundedA=[boundedA;repmat(noHistory,m*K,1);true(K,1)]; %#ok<AGROW>
                boundedB=[boundedB;true(m*K,1);zeroMu]; %#ok<AGROW>
                constantZeroPairs=[constantZeroPairs;repelem(zeroMu,m)|noHistory;zeroMu|noHistory]; %#ok<AGROW>
            else,c=[c;-d.coneSlack(:)/options.forceScaleN];end %#ok<AGROW>
        end
    end
    function [c,e,gc,ge]=constraints(q)
        try
            frames=decode(q); [a,b,c,e]=physicalConstraints(frames);
            c=[c;-a;-b;a.*b-tau];
            if nargout>2
                d=derivatives(q);gc=[d.Jc;-d.Ja;-d.Jb;b.*d.Ja+a.*d.Jb]';ge=d.Je';
            end
        catch err
            rejectTrial(err); c=ones(constraintCount,1)*1e6; e=ones(equalityCount,1)*1e6;
            if nargout>2,gc=zeros(numel(q),constraintCount);ge=zeros(numel(q),equalityCount);end
        end
    end
    function d=derivatives(q)
        if isequal(q,derivativeY),d=derivativeCache;derivativeCacheHits=derivativeCacheHits+1;return;end
        [r,measurementRows]=residual(q);[a,b,c,e]=physicalConstraints(decode(q));
        [~,J,n]=formulation_derivative_bundle(@packed,q);derivativeEvaluations=derivativeEvaluations+n;
        ends=cumsum([numel(r),numel(a),numel(b),numel(c),numel(e)]);starts=[0 ends(1:end-1)];
        d=struct('Jr',J(starts(1)+1:ends(1),:),'Ja',J(starts(2)+1:ends(2),:), ...
            'Jb',J(starts(3)+1:ends(3),:),'Jc',J(starts(4)+1:ends(4),:), ...
            'Je',J(starts(5)+1:ends(5),:),'measurementRows',measurementRows);
        derivativeY=q;derivativeCache=d;
    end
    function v=packed(q)
        r=residual(q);[a,b,c,e]=physicalConstraints(decode(q));v=[r;a;b;c;e];
    end
    function [q,value,flag,out]=polishBranch(q)
        % Restore an active surface initializer before asking the optimizer
        % to polish. With ~100 N reactions, even a 0.3 micrometre gap violates
        % the original product tolerance. This projection changes only the
        % initializer; the original camera/process objective is optimized.
        frames=decode(q);xx=expand(q);
        for kk=1:T
            dframe=frames{kk};
            if options.useContactGeometry
                for pp=1:obs.planeCount
                    contacts=find(candidates.planeIndex==pp & dframe.normalForce>options.activeForceThresholdN);
                    if ~isempty(contacts)
                        ix=spec.plane(pp);xx(ix.point,kk)=xx(ix.point,kk)+dframe.normal(:,pp)*mean(dframe.gap(contacts));
                    end
                end
            end
            if options.useFrictionHistory&&obs.predecessorIndex(kk)>0
                for jj=1:K
                    ix=spec.contact(jj);
                    if obs.mu(ix.plane,kk)==0
                        slip=dframe.directions{ix.plane}'*dframe.tangentialStepMm(:,jj);
                        xx(ix.lambda,kk)=min(options.maxSlipMm,max([0;-slip])+1e-6);
                    end
                end
            end
        end
        q=(xx(free)-origin)./sc;
        [a,b,~,~]=physicalConstraints(decode(q));d=derivatives(q);
        % A slightly negative slack at a relaxed trial must not activate the
        % opposite zero-force branch (e.g. beta=0, w<0). Favor its exact zero
        % load side so lambda can restore w>=0 without imposing w=0.
        chooseA=b>1e-8 & b>=max(a,0);branchA=find(chooseA);branchB=find(~chooseA);
        branchA=branchA(~constantZeroPairs(branchA));branchB=branchB(~constantZeroPairs(branchB));
        % Fixed-coordinate zero rows need no new equality. Original a>=0,
        % b>=0 and post-solve complementarity remain checked in all cases.
        branchA=branchA(vecnorm(d.Ja(branchA,:),2,2)>1e-10);
        branchB=branchB(vecnorm(d.Jb(branchB,:),2,2)>1e-10);
        branchScaleA=max(1,abs(b(branchA)));branchScaleB=max(1,abs(a(branchB)));
        restorationOrigin=q;
        restorationPhysicsCount=equalityCount+numel(branchA)+numel(branchB)+2*numel(initialA)+ ...
            numel(initialC)+numel(linearB);
        restorationCount=restorationPhysicsCount+numel(q)+numel(residual(q));
        restoreOptions=optimoptions('lsqnonlin','Display','off','MaxIterations',options.restorationIterations, ...
            'MaxFunctionEvaluations',options.maxFunctionEvaluations,'FunctionTolerance',1e-14, ...
            'OptimalityTolerance',1e-10,'StepTolerance',1e-12,'FiniteDifferenceType','central', ...
            'FiniteDifferenceStepSize',1e-5);
        restoreBefore=restorationResidual(q);before=norm(restoreBefore(1:restorationPhysicsCount),inf);
        restoreStart=q;
        [restoreTrial,restoreNorm,~,restoreFlag,restoreOut]=lsqnonlin(@restorationResidual,q,lo,hi,restoreOptions);
        restoreAfter=restorationResidual(restoreTrial);
        [restoreAccepted,initialRestoreMerit,candidateRestoreMerit]= ...
            formulation_restoration_merit(restoreBefore,restoreAfter);
        % Re-evaluate the actual fixed merit, rather than trusting an exit
        % code or the solver's reported norm. A returned restoration can be
        % worse than its start at a nonsmooth mechanical branch. Preserve
        % the original initializer and continue the original MAP in that case.
        if restoreAccepted,q=restoreTrial;else,q=restoreStart;end
        restoration=struct('exitflag',restoreFlag,'iterations',restoreOut.iterations, ...
            'residualSquaredNorm',restoreNorm,'initialMaxViolation',before, ...
            'restoredMaxViolation',norm(restoreAfter(1:restorationPhysicsCount),inf), ...
            'accepted',restoreAccepted,'initialResidualSquaredNorm',initialRestoreMerit, ...
            'candidateResidualSquaredNorm',candidateRestoreMerit, ...
            'mapWeight',options.restorationMapWeight,'mapResidualSquaredNorm',norm(residual(q))^2, ...
            'scope','Initializer feasibility only; the original MAP is optimized and audited next.');
        opts=solverOptions;opts.OutputFcn=[];opts.MaxIterations=options.polishIterations;
        % Use the same physical accuracy target as exact continuation. An
        % extra factor of ten made the default constraint tolerance 1e-9,
        % stricter than the 1e-8 step threshold: even a restored 3.8e-9
        % physical violation was rejected as infeasible on a clean packet.
        % The original unrelaxed audit is still mandatory after optimization.
        opts.ConstraintTolerance=min(options.constraintTolerance,options.exactContinuationTolerance);
        [q,value,flag,out]=fmincon(@objective,q,linearA,linearB,[],[],lo,hi,@branchConstraints,opts);
        if flag<=0
            opts.Algorithm='interior-point';
            [trial,vTrial,fTrial,oTrial]=fmincon(@objective,q,linearA,linearB,[],[],lo,hi,@branchConstraints,opts);
            if fTrial>0,q=trial;value=vTrial;flag=fTrial;out=oTrial;end
        end
        out.restoration=restoration;
        out.branchConstraintTolerance=opts.ConstraintTolerance;
        out.branchStepTolerance=opts.StepTolerance;
        function r=restorationResidual(v)
            try
                [aa,bb,cc,ee]=physicalConstraints(decode(v));
                % A feasibility-only restoration can leave the observed
                % shape manifold and return a mechanically feasible but
                % unusable initializer. Retain a weak original-MAP merit
                % term here; the subsequent constrained solve uses the
                % original objective, without this restoration weighting.
                r=[ee;branchScaleA.*aa(branchA);branchScaleB.*bb(branchB); ...
                    min(aa,0);min(bb,0);max(cc,0);max(linearA*v-linearB,0); ...
                    1e-8*(v-restorationOrigin);options.restorationMapWeight*residual(v)];
            catch err,rejectTrial(err);r=ones(restorationCount,1)*1e10;end
        end
    end
    function [c,e,gc,ge]=branchConstraints(q)
        keepA=nonnegativeA;keepB=nonnegativeB;keepA(branchA)=false;keepB(branchB)=false;
        % A selected zero equality also enforces that side's nonnegativity.
        % Keep all other inequalities; original a/b are audited afterward.
        try
            [a,b,c,e]=physicalConstraints(decode(q));c=[c;-a(keepA);-b(keepB)];
            e=[e;branchScaleA.*a(branchA);branchScaleB.*b(branchB)];
            if nargout>2
                d=derivatives(q);gc=[d.Jc;-d.Ja(keepA,:);-d.Jb(keepB,:)]';
                ge=[d.Je;branchScaleA.*d.Ja(branchA,:);branchScaleB.*d.Jb(branchB,:)]';
            end
        catch err
            rejectTrial(err);nc=numel(initialC)+sum(keepA)+sum(keepB);ne=equalityCount+numel(branchA)+numel(branchB);
            c=ones(nc,1)*1e6;e=ones(ne,1)*1e6;
            if nargout>2,gc=zeros(numel(q),nc);ge=zeros(numel(q),ne);end
        end
    end
    function r=safeSeed(q)
        try, r=seedResidual(q);
        catch err, rejectTrial(err); r=ones(seedResidualCount,1)*1e10; end
    end
    function rejectTrial(err)
        if ~ismember(err.identifier,{'rod:CosseratEquilibrium','rod:InvalidOptimizerTrial'}), rethrow(err); end
        rejected=rejected+1; lastRejection=err.message;
    end
    function stop=progress(~,v,state)
        stop=false;
        if strcmp(state,'iter')&&mod(v.iteration,10)==0
            fprintf('  joint MPCC tau=%g iteration=%d cost=%.4g %.1fs\n',tau,v.iteration,v.fval,toc(stageTimer));
        end
    end
    function q=audit(frames,exitflag)
        q=struct('finite',true(1,T),'equilibriumResidualNmm',zeros(1,T),'minimumRodGapMm',inf(1,T), ...
            'complementarityResidual',zeros(1,T),'tangencyResidual',zeros(1,T), ...
            'minimumArcSeparationMm',inf(1,T),'configuredConstraintViolation',zeros(1,T), ...
            'curvatureResidualRms',zeros(1,T),'frictionHistoryAvailable',obs.predecessorIndex>0, ...
            'requiresReview',false(1,T),'forceAccuracyCertified',false,'uncertaintyCoverageValidated',false);
        for kk=1:T
            d=frames{kk}; q.finite(kk)=all(isfinite([d.shape.p(:);d.contactForce(:);d.tipForce(:)]));
            q.equilibriumResidualNmm(kk)=d.shape.tipMomentResidualNmm;
            q.minimumRodGapMm(kk)=min([inf;d.rodGap(:)]);
            q.complementarityResidual(kk)=max([0;abs(d.gap(:).*d.normalForce(:)); ...
                abs(d.frictionW(:).*d.beta(:));abs(d.coneSlack(:).*d.lambda(:)); ...
                -d.gap(:);-d.frictionW(:);-d.coneSlack(:)]);
            q.tangencyResidual(kk)=max([0;abs(d.tangency(:).*d.normalForce(:))]);
            q.minimumArcSeparationMm(kk)=min([inf;diff(d.contactS(:))]);
            violation=max([0;options.minArcSeparationMm-diff(d.contactS(:));-d.coneSlack(:)]);
            if options.useContactGeometry
                violation=max([violation;-d.rodGap(:);-d.gap(:);abs(d.gap(:).*d.normalForce(:));abs(d.tangency(:).*d.normalForce(:))]);
            end
            if options.useFrictionHistory
                violation=max([violation;-d.frictionW(:);abs(d.frictionW(:).*d.beta(:));abs(d.coneSlack(:).*d.lambda(:))]);
            end
            q.configuredConstraintViolation(kk)=violation;
            e=obs.curvatureWhitening(:,:,kk)*(d.predictedCurvature(:)-reshape(obs.u(:,:,kk),[],1));
            q.curvatureResidualRms(kk)=sqrt(mean(e.^2));
            missingHistory=options.useFrictionHistory&&obs.predecessorIndex(kk)==0&&any(obs.mu(candidates.planeIndex,kk)'>0 & d.normalForce>options.activeForceThresholdN);
            q.requiresReview(kk)=~q.finite(kk)||exitflag<=0||q.equilibriumResidualNmm(kk)>options.equilibriumToleranceNmm|| ...
                q.configuredConstraintViolation(kk)>options.complementarityTolerance|| ...
                q.curvatureResidualRms(kk)>options.maxWhitenedCurvatureRms||missingHistory|| ...
                candidateAudit.truncated||candidateAudit.ambiguousCornerCandidates||candidateAudit.ambiguousTrackAssignment;
        end
    end
    function u=localCovariance(q,frames)
        d=derivatives(q);J=d.Jr;[a,b,c,~]=physicalConstraints(frames);
        Jdata=J(d.measurementRows,:);
        active=find(c>-1e-5); sideA=find(a<1e-5 & b>1e-5); sideB=find(b<1e-5 & a>1e-5);
        lowerActive=find(q-lo<1e-5); upperActive=find(hi-q<1e-5);
        activeLinear=find(linearA*q-linearB>-1e-5);I=eye(numel(q));
        G=full([d.Je;d.Jc(active,:);d.Ja(sideA,:);d.Jb(sideB,:); ...
            linearA(activeLinear,:);I(lowerActive,:);-I(upperActive,:)]);
        N=null(G,1e-7); H=J*N; [~,Sinfo,Vinfo]=svd(H,'econ'); singular=diag(Sinfo);
        tolInfo=max(1e-7,max(size(H))*eps(max([singular;1]))); keep=singular>tolInfo; rankH=sum(keep);
        Vresolved=Vinfo(:,keep);
        C=N*(Vresolved*diag(1./singular(keep).^2)*Vresolved')*N';
        forceJ=numericalJacobian(@forceVector,q);
        forceCov=forceJ*C*forceJ'; nullInformation=N*null(H,tolInfo);
        unresolved=sqrt(sum((forceJ*nullInformation).^2,2))>1e-7;
        dataInformation=Jdata*N; dataSingular=svd(dataInformation,'econ');
        tolData=max(1e-7,max(size(dataInformation))*eps(max([dataSingular;1])));
        dataNull=N*null(dataInformation,tolData);
        unresolvedByData=sqrt(sum((forceJ*dataNull).^2,2))>1e-7;
        stdv=sqrt(max(diag(forceCov),0)); stdv(unresolved)=inf;
        u=struct('computed',true,'coverageValidated',false,'forceCovariance',forceCov, ...
            'forceStd',stdv,'forceUnresolved',unresolved,'tangentDimension',size(N,2), ...
            'forceUnresolvedByData',unresolvedByData,'measurementInformationRank',sum(dataSingular>tolData), ...
            'informationRankTolerance',tolInfo,'measurementRankTolerance',tolData, ...
            'informationRank',rankH,'singularValues',singular,'weaklyActivePairs',sum(a<1e-5&b<1e-5), ...
            'scope','Local constrained linearization including nuisance planes/positions/moments and process prior; no mode/candidate mixture or model-calibration uncertainty.');
        function f=forceVector(v)
            xx=expand(v);f=[];
            for z=obs.outputIndices
                force=zeros(3,K);
                for jj=1:K
                    ix=spec.contact(jj);pp=spec.plane(ix.plane);
                    [normal,D]=formulation_contact_frame(obs.normal(:,ix.plane,1),xx(pp.eta,z),m);
                    force(:,jj)=normal*xx(ix.fn,z)+D*xx(ix.beta,z);
                end
                f=[f;force(:);xx(spec.tip,z)]; %#ok<AGROW>
            end
        end
    end
end
function opts=defaults(opts)
assert(isstruct(opts)&&isscalar(opts),'rod:InvalidWindowOptions','Options must be a scalar struct.');
d=struct('numFrictionDirs',4,'maxContacts',4,'candidateMaxGapMm',5,'candidateMergeMm',10, ...
    'candidateTrackMaxFraction',0.25,'contactArcMode','ordered', ...
    'minArcSeparationMm',0.1,'normalChartRadiusRad',0.5,'maxForceN',200,'maxTipForceN',25,'maxSlipMm',200, ...
    'relativeTolerance',2e-8,'collisionStepMm',1,'maxRhsEvaluations',100000, ...
    'seedIterations',50,'seedCurvatureFloorsPerMm',[1e-4 1e-5 0], ...
    'maxIterations',100,'maxFunctionEvaluations',20000,'retryInteriorPoint',true,'retryIterations',40, ...
    'seedMomentStdNmm',1e-3,'seedGapStdMm',1e-3,'seedTangencyStd',1e-4, ...
    'relaxations',[1e-2 1e-4 1e-6 1e-8],'lengthScaleMm',1,'forceScaleN',1, ...
    'constraintTolerance',1e-5,'exactContinuationTolerance',1e-8, ...
    'activeForceThresholdN',1e-3,'equilibriumToleranceNmm',2e-4,'geometryToleranceMm',2e-4, ...
    'complementarityTolerance',1e-5,'maxWhitenedCurvatureRms',4,'showProgress',true,'computeCovariance',false, ...
    'cacheMechanics',true,'polishActiveBranch',true,'polishIterations',60,'restorationIterations',30, ...
    'restorationMapWeight',1e-4, ...
    'useSharedDerivatives',true,'useContactGeometry',true, ...
    'useFrictionHistory',true,'useTemporalPrior',true,'useEnvironmentLikelihood',true);
names=fieldnames(d); for k=1:numel(names), if ~isfield(opts,names{k}), opts.(names{k})=d.(names{k}); end; end
switches={'showProgress','computeCovariance','retryInteriorPoint','cacheMechanics', ...
    'polishActiveBranch','useSharedDerivatives','useContactGeometry','useFrictionHistory','useTemporalPrior','useEnvironmentLikelihood'};
for k=1:numel(switches)
    v=opts.(switches{k});
    assert((islogical(v)||isnumeric(v))&&isreal(v)&&isscalar(v)&&ismember(v,[0 1]), ...
        'rod:InvalidWindowOptions','%s must be a scalar logical value.',switches{k});
end
positive={'candidateMaxGapMm','candidateMergeMm','minArcSeparationMm','normalChartRadiusRad','maxForceN', ...
    'maxTipForceN','maxSlipMm','relativeTolerance','collisionStepMm','maxRhsEvaluations','seedIterations', ...
    'maxIterations','maxFunctionEvaluations','retryIterations','seedMomentStdNmm','seedGapStdMm','seedTangencyStd', ...
    'lengthScaleMm','forceScaleN','equilibriumToleranceNmm','geometryToleranceMm','complementarityTolerance','maxWhitenedCurvatureRms'};
positive=[positive,{'constraintTolerance','exactContinuationTolerance','candidateTrackMaxFraction','polishIterations','restorationIterations','activeForceThresholdN','restorationMapWeight'}];
for k=1:numel(positive)
    v=opts.(positive{k}); assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'rod:InvalidWindowOptions','%s must be a finite positive scalar.',positive{k});
end
integers={'maxRhsEvaluations','seedIterations','maxIterations','maxFunctionEvaluations','retryIterations','polishIterations','restorationIterations'};
for k=1:numel(integers)
    v=opts.(integers{k});assert(v==fix(v),'rod:InvalidWindowOptions','%s must be an integer.',integers{k});
end
assert(ischar(opts.contactArcMode)&&ismember(opts.contactArcMode,{'ordered','partitioned'}), ...
    'rod:InvalidWindowOptions','contactArcMode must be ordered or partitioned.');
assert(isnumeric(opts.maxContacts)&&isreal(opts.maxContacts)&&isscalar(opts.maxContacts)&& ...
    isfinite(opts.maxContacts)&&opts.maxContacts>=0&&opts.maxContacts==round(opts.maxContacts)&& ...
    isnumeric(opts.numFrictionDirs)&&isreal(opts.numFrictionDirs)&&isscalar(opts.numFrictionDirs)&& ...
    isfinite(opts.numFrictionDirs)&&opts.numFrictionDirs>=2&&opts.numFrictionDirs==round(opts.numFrictionDirs)&& ...
    isnumeric(opts.relaxations)&&isreal(opts.relaxations)&&isvector(opts.relaxations)&&~isempty(opts.relaxations)&& ...
    all(isfinite(opts.relaxations))&&all(opts.relaxations>0)&&all(diff(opts.relaxations)<0), ...
    'rod:InvalidWindowOptions','Invalid candidate count/friction directions/continuation.');
assert(isnumeric(opts.seedCurvatureFloorsPerMm)&&isreal(opts.seedCurvatureFloorsPerMm)&& ...
    isvector(opts.seedCurvatureFloorsPerMm)&&~isempty(opts.seedCurvatureFloorsPerMm)&& ...
    all(isfinite(opts.seedCurvatureFloorsPerMm))&&all(opts.seedCurvatureFloorsPerMm>=0), ...
    'rod:InvalidWindowOptions','Initialization covariance floors must be finite nonnegative values.');
end
function H=hat(v), H=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0]; end
function out=subsetQuality(q,idx)
out=q; names=fieldnames(q);
for k=1:numel(names)
    v=q.(names{k}); if numel(v)>1, out.(names{k})=v(idx); end
end
end
function J=numericalJacobian(fun,x)
f=fun(x); J=zeros(numel(f),numel(x));
for j=1:numel(x)
    h=1e-5*max(1,abs(x(j))); a=x; b=x; a(j)=a(j)+h; b(j)=b(j)-h;
    J(:,j)=(fun(a)-fun(b))/(2*h);
end
end
