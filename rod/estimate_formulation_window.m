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
stiffness=reshape(getTubeK(tube),3,[]); momentScale=mean(stiffness(1:2,:),'all')/(tube.s(end)-tube.s(1));
[x0,lb,ub,scale,priorStd,processStd]=initialState();
assert(all(isfinite(priorStd))&&all(priorStd>0)&&all(isfinite(processStd))&&all(processStd>0), ...
    'rod:InvalidWindowPrior','Prior and process standard deviations must be finite and positive.');
if isfield(options,'initialState')
    z=options.initialState;
    assert(isequal(size(z),size(x0))&&isreal(z)&&all(isfinite(z),'all')&& ...
        all(z>=lb,'all')&&all(z<=ub,'all'),'rod:InvalidWindowInitialState', ...
        'Warm state must match inferred state coordinates/times and obey bounds.');
    x0=z; % initialization only; never converted into another posterior factor
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
cachedY=[]; cachedFrames={}; rejected=0; lastRejection='';
seedWhitening=obs.curvatureWhitening;
timer=tic; stages=cell(1,numel(options.relaxations));
[initialA,~,initialC,initialE]=physicalConstraints(decode(y));
constraintCount=numel(initialC)+3*numel(initialA); equalityCount=numel(initialE);
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
    [y,seedNorm,~,seedFlag,seedOut]=lsqnonlin(@safeSeed,y,lo,hi,seedOptions);
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
        if obs.mu(ix.plane,kk)==0&&obs.predecessorIndex(kk)>0
            slip=seedFrames{kk}.directions{ix.plane}'*seedFrames{kk}.tangentialStepMm(:,jj);
            seedX(ix.lambda,kk)=min(options.maxSlipMm,max([0;-slip])+1e-4);
        end
    end
end
y=(seedX(free)-origin)./sc;
solverOptions=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',options.maxIterations,'MaxFunctionEvaluations',options.maxFunctionEvaluations, ...
    'ConstraintTolerance',options.constraintTolerance,'OptimalityTolerance',1e-5,'StepTolerance',1e-8, ...
    'FiniteDifferenceStepSize',1e-5,'FiniteDifferenceType','central','SpecifyObjectiveGradient',true);
if options.showProgress, solverOptions.OutputFcn=@progress; end
best=struct('available',false,'y',y,'objective',inf,'exitflag',0,'stage',0);
usedInteriorRetry=false;
for stage=1:numel(options.relaxations)
    tau=options.relaxations(stage); stageTimer=tic;
    [y,value,flag,out]=fmincon(@objective,y,[],[],[],[],lo,hi,@constraints,solverOptions);
    sqpExit=flag; retry=[];
    if flag<=0&&options.retryInteriorPoint&&~usedInteriorRetry&&~best.available
        % Planar/zero-friction branches can have linearly dependent active
        % rows. Try another constrained optimizer on the SAME MAP and MPCC;
        % record both exits, rather than accepting a failed SQP as success.
        retryOptions=solverOptions; retryOptions.Algorithm='interior-point';
        usedInteriorRetry=true;
        retryOptions.MaxIterations=options.retryIterations;
        [trial,vTrial,fTrial,oTrial]=fmincon(@objective,y,[],[],[],[],lo,hi,@constraints,retryOptions);
        [tc,te]=constraints(trial); retryViolation=max([0;tc;abs(te)]);
        retry=struct('algorithm','interior-point','exitflag',fTrial,'iterations',oTrial.iterations, ...
            'objective',vTrial,'constraintViolation',retryViolation,'message',oTrial.message);
        if fTrial>0 || (flag<=0&&retryViolation<out.constrviolation)
            y=trial; value=vTrial; flag=fTrial; out=oTrial;
        end
    end
    [c,e]=constraints(y);
    stages{stage}=struct('relaxation',tau,'exitflag',flag,'iterations',out.iterations, ...
        'functionCount',out.funcCount,'objective',value,'constraintViolation',max([0;c;abs(e)]), ...
        'seconds',toc(stageTimer),'message',out.message,'firstOrderOptimality',out.firstorderopt);
    stages{stage}.sqpExitflag=sqpExit; stages{stage}.retry=retry;
    [aa,bb,cc,ee]=physicalConstraints(decode(y));
    unrelaxed=max([0;-aa;-bb;abs(aa.*bb);cc;abs(ee)]);
    stages{stage}.unrelaxedViolation=unrelaxed;
    stages{stage}.accepted=flag>0&&unrelaxed<=options.complementarityTolerance;
    if stages{stage}.accepted&&value<best.objective
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
            ix=spec.contact(jj); X(ix.s,:)=candidates.seedArcMm(jj); S(ix.s,:)=10;
            partitions=[tube.s(1),mean([candidates.seedArcMm(1:end-1);candidates.seedArcMm(2:end)],1),tube.s(end)];
            lower(ix.s,:)=partitions(jj)+options.minArcSeparationMm/2;
            upper(ix.s,:)=partitions(jj+1)-options.minArcSeparationMm/2;
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
                if obs.predecessorIndex(kk)==0, upper(ix.lambda,kk)=0; end
            end
        end
        lower(spec.tip,:)=-options.maxTipForceN; upper(spec.tip,:)=options.maxTipForceN;
        Pstd(spec.tip)=fs.priorStd.tipForceN; Qstd(spec.tip)=fs.processStd.tipForceN;
        S(spec.baseMoment,:)=momentScale;
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
                    if tube.s(zz)<candidates.seedArcMm(jj)
                        pc=interp1(tube.s,sh.p',candidates.seedArcMm(jj))';
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
                pc=interp1(tube.s,sh.p',candidates.seedArcMm(jj))';
                moment=moment+cross(pc-sh.p(:,1),seedD{jj}*f(columns));
            end
            X(spec.tip,kk)=f(end-2:end);
            X(spec.baseMoment,kk)=moment+cross(sh.p(:,end)-sh.p(:,1),f(end-2:end));
        end
    end
    function X=expand(q)
        v=fixed; v(free)=origin+sc.*q; X=reshape(v,nx,T);
    end
    function frames=decode(q)
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
            sh=integrate_cosserat_load_state(rod,x(spec.baseMoment),s,force,x(spec.tip),options);
            [sensorP,sensorR]=cosserat_state_at_arc(sh,obs.arcs); %#ok<ASGLU>
            predicted=zeros(numel(obs.axes),numel(obs.arcs));
            for zz=1:numel(obs.arcs)
                node=find(tube.s<=obs.arcs(zz),1,'last');
                segment=find(sh.segmentEdges<=obs.arcs(zz),1,'last'); segment=min(segment,numel(sh.pieces));
                state=deval(sh.pieces{segment},obs.arcs(zz));
                curv=tube.uhat(:,node)+(sensorR(:,:,zz)'*state(13:15))./stiffness(:,node);
                predicted(:,zz)=curv(obs.axes);
            end
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
    function r=residual(q,curvatureW)
        if nargin<2, curvatureW=obs.curvatureWhitening; end
        frames=decode(q); X=expand(q); r=[];
        for kk=1:T
            e=frames{kk}.predictedCurvature-obs.u(:,:,kk);
            r=[r;curvatureW(:,:,kk)*e(:)]; %#ok<AGROW>
            if obs.environmentLikelihood(kk)
                for jj=1:obs.planeCount
                    e=[frames{kk}.point(:,jj)-obs.point(:,jj,kk);frames{kk}.normal(:,jj)-obs.normal(:,jj,kk)];
                    r=[r;obs.environmentWhitening(:,:,jj,kk)*e]; %#ok<AGROW>
                end
            end
            if kk>1
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
            d=frames{kk}; r=[r;d.shape.terminalMomentNmm/options.seedMomentStdNmm; ...
                d.gap(:)/options.seedGapStdMm;d.tangency(:)/options.seedTangencyStd; ...
                min(d.rodGap(:),0)/options.seedGapStdMm;min(d.coneSlack(:),0)/0.01; ...
                min(d.frictionW(:),0)/options.seedGapStdMm]; %#ok<AGROW>
        end
    end
    function [v,g]=objective(q)
        try
            r=residual(q); v=0.5*(r'*r);
            if nargout>1, J=numericalJacobian(@residual,q); g=J'*r; end
        catch err
            rejectTrial(err); v=1e30; if nargout>1, g=zeros(size(q)); end
        end
    end
    function [a,b,c,e]=physicalConstraints(frames)
        a=[]; b=[]; c=[]; e=[];
        for kk=1:T
            d=frames{kk}; a=[a;d.gap(:)/options.lengthScaleMm;d.frictionW(:)/options.lengthScaleMm;d.lambda(:)/options.lengthScaleMm]; %#ok<AGROW>
            b=[b;d.normalForce(:)/options.forceScaleN;d.beta(:)/options.forceScaleN;d.coneSlack(:)/options.forceScaleN]; %#ok<AGROW>
            c=[c;-d.rodGap(:)/options.lengthScaleMm; ...
                (options.minArcSeparationMm-diff(d.contactS(:)))/options.lengthScaleMm]; %#ok<AGROW>
            e=[e;d.shape.terminalMomentNmm/momentScale;d.tangency(:).*d.normalForce(:)/options.forceScaleN]; %#ok<AGROW>
        end
    end
    function [c,e]=constraints(q)
        try
            frames=decode(q); [a,b,c,e]=physicalConstraints(frames);
            c=[c;-a;-b;a.*b-tau];
        catch err
            rejectTrial(err); c=ones(constraintCount,1)*1e6; e=ones(equalityCount,1)*1e6;
        end
    end
    function r=safeSeed(q)
        try, r=seedResidual(q);
        catch err, rejectTrial(err); r=ones(seedResidualCount,1)*1e10; end
    end
    function rejectTrial(err)
        if ~strcmp(err.identifier,'rod:CosseratEquilibrium'), rethrow(err); end
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
            e=obs.curvatureWhitening(:,:,kk)*(d.predictedCurvature(:)-reshape(obs.u(:,:,kk),[],1));
            q.curvatureResidualRms(kk)=sqrt(mean(e.^2));
            missingHistory=obs.predecessorIndex(kk)==0&&any(obs.mu(candidates.planeIndex,kk)'>0 & d.normalForce>options.activeForceThresholdN);
            q.requiresReview(kk)=~q.finite(kk)||exitflag<=0||q.equilibriumResidualNmm(kk)>options.equilibriumToleranceNmm|| ...
                q.minimumRodGapMm(kk)<-options.geometryToleranceMm|| ...
                q.complementarityResidual(kk)>options.complementarityTolerance|| ...
                q.tangencyResidual(kk)>options.complementarityTolerance|| ...
                q.curvatureResidualRms(kk)>options.maxWhitenedCurvatureRms||missingHistory|| ...
                candidateAudit.truncated||candidateAudit.ambiguousCornerCandidates;
        end
    end
    function u=localCovariance(q,frames)
        J=numericalJacobian(@residual,q); [a,b,c,e]=physicalConstraints(frames);
        Jdata=numericalJacobian(@measurementResidual,q);
        active=find(c>-1e-5); sideA=find(a<1e-5 & b>1e-5); sideB=find(b<1e-5 & a>1e-5);
        lowerActive=find(q-lo<1e-5); upperActive=find(hi-q<1e-5);
        G=numericalJacobian(@activeResidual,q);
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
        function r=activeResidual(v)
            [aa,bb,cc,ee]=physicalConstraints(decode(v));
            r=[ee;cc(active);aa(sideA);bb(sideB);v(lowerActive)-lo(lowerActive);hi(upperActive)-v(upperActive)];
        end
        function f=forceVector(v)
            d=decode(v); f=[];
            for z=obs.outputIndices, f=[f;d{z}.contactForce(:);d{z}.tipForce]; end %#ok<AGROW>
        end
        function r=measurementResidual(v)
            dd=decode(v); r=[];
            for z=1:T
                delta=dd{z}.predictedCurvature-obs.u(:,:,z);
                r=[r;obs.curvatureWhitening(:,:,z)*delta(:)]; %#ok<AGROW>
                if obs.environmentLikelihood(z)
                    for pp=1:obs.planeCount
                        delta=[dd{z}.point(:,pp)-obs.point(:,pp,z);dd{z}.normal(:,pp)-obs.normal(:,pp,z)];
                        r=[r;obs.environmentWhitening(:,:,pp,z)*delta]; %#ok<AGROW>
                    end
                end
            end
        end
    end
end
function opts=defaults(opts)
d=struct('numFrictionDirs',4,'maxContacts',4,'candidateMaxGapMm',5,'candidateMergeMm',10, ...
    'minArcSeparationMm',0.1,'normalChartRadiusRad',0.5,'maxForceN',200,'maxTipForceN',25,'maxSlipMm',200, ...
    'relativeTolerance',2e-8,'collisionStepMm',1,'maxRhsEvaluations',100000, ...
    'seedIterations',50,'seedCurvatureFloorsPerMm',[1e-4 1e-5 0], ...
    'maxIterations',100,'maxFunctionEvaluations',20000,'retryInteriorPoint',true,'retryIterations',40, ...
    'seedMomentStdNmm',1e-3,'seedGapStdMm',1e-3,'seedTangencyStd',1e-4, ...
    'relaxations',[1e-2 1e-4 1e-6 1e-8],'lengthScaleMm',1,'forceScaleN',1, ...
    'constraintTolerance',1e-5,'exactContinuationTolerance',1e-8, ...
    'activeForceThresholdN',1e-3,'equilibriumToleranceNmm',2e-4,'geometryToleranceMm',2e-4, ...
    'complementarityTolerance',1e-5,'maxWhitenedCurvatureRms',4,'showProgress',true,'computeCovariance',false);
names=fieldnames(d); for k=1:numel(names), if ~isfield(opts,names{k}), opts.(names{k})=d.(names{k}); end; end
positive={'candidateMaxGapMm','candidateMergeMm','minArcSeparationMm','normalChartRadiusRad','maxForceN', ...
    'maxTipForceN','maxSlipMm','relativeTolerance','collisionStepMm','maxRhsEvaluations','seedIterations', ...
    'maxIterations','maxFunctionEvaluations','retryIterations','seedMomentStdNmm','seedGapStdMm','seedTangencyStd', ...
    'lengthScaleMm','forceScaleN','equilibriumToleranceNmm','geometryToleranceMm','complementarityTolerance','maxWhitenedCurvatureRms'};
positive=[positive,{'constraintTolerance','exactContinuationTolerance'}];
for k=1:numel(positive)
    v=opts.(positive{k}); assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
        'rod:InvalidWindowOptions','%s must be a finite positive scalar.',positive{k});
end
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
