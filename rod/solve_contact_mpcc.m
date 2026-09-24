function [x,info] = solve_contact_mpcc(x0,objective,decode,bounds,scale,cfg)
%SOLVE_CONTACT_MPCC Scaled Scholtes continuation for the COMPLETE MPCC.
% No prescribed stick/slip mode and no noise-triggered constraint deletion.
% a>=0,b>=0,a*b<=tau approaches complementarity as tau decreases.
% Final unrelaxed residuals, all stage exits and the actual tolerance are
% returned. A finite relaxation is a numerical tolerance, not exact equality.
m=cfg.numFrictionDirs; origin=x0(:); scale=scale(:);
fixed=[];
if cfg.currentMu==0, fixed=8:7+m; origin(fixed)=0; end
free=setdiff(1:numel(x0),fixed); sc=scale(free);
lo=(bounds{1}(free)-origin(free))./sc; hi=(bounds{2}(free)-origin(free))./sc;
stages=cfg.mpccRelaxations; y=zeros(numel(free),1); traces=cell(1,numel(stages));
opts=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',cfg.linearizedSolveMaxIter,'MaxFunctionEvaluations',12000, ...
    'FiniteDifferenceStepSize',2e-5,'FiniteDifferenceType','central','OptimalityTolerance',1e-5, ...
    'ConstraintTolerance',1e-8,'StepTolerance',1e-8);
if isfield(cfg,'nonlinearFiniteDifferenceStep')
    opts.FiniteDifferenceStepSize=cfg.nonlinearFiniteDifferenceStep;
end
if cfg.showProgress,opts.OutputFcn=@progress;end
timer=tic;
rejected=0;
lastRejection='';
[initialGeometryC,initialGeometryEq]=plane_contact_constraints(decode(origin),cfg);
for stage=1:numel(stages)
    tau=stages(stage);
    stageTimer=tic;stageRejected=rejected;
    [y,value,flag,out]=fmincon(@safeObjective,y,[],[],[],[],lo,hi,@constraints,opts);
    [c,ceq]=constraints(y);
    traces{stage}=struct('relaxation',tau,'exitflag',flag,'iterations',out.iterations, ...
        'objective',value,'constraintViolation',max([0;c;abs(ceq)]), ...
        'functionCount',out.funcCount,'seconds',toc(stageTimer), ...
        'rejectedMechanicsTrials',rejected-stageRejected);
    if cfg.showProgress
        fprintf('      full MPCC tau %.1g: objective %.5g, exit %d, violation %.3g\n',tau,value,flag,max([0;c;abs(ceq)]));
    end
end
x=expand(y); d=decode(x);
violation=fullViolation(d);
% Polish the active set inferred by the FULL solve, including sliding faces
% with two active generators. This is not a mode chosen from noisy data.
mode=struct('name','no-contact','activeDirection',[]);
if d.fn>1e-4*cfg.mpccForceScaleN
    if cfg.currentMu==0,mode.name='frictionless-contact';
    elseif norm(d.vTangential)<1e-4*cfg.mpccLengthScaleMm,mode.name='sticking-contact';
    else
        mode.name='sliding-contact';
        active=find(d.beta>1e-5*cfg.mpccForceScaleN & d.frictionW<1e-4*cfg.mpccLengthScaleMm);
        if isempty(active),[~,active]=min(d.frictionW);end
        mode.activeDirection=active(:)';
    end
end
[polished,polishInfo]=solve_contact_mode_map(x,objective,decode,bounds,scale,cfg,mode);
polishViolation=fullViolation(decode(polished));
polishAccepted=polishInfo.exitflag>0 && polishViolation<=max(1e-7,violation) && ...
    (objective(polished)<=value+1e-5*max(1,abs(value)) || violation>1e-7);
selectedOptimality=out.firstorderopt;selectedIterations=out.iterations;
finalStateSource='homotopy';
if polishAccepted
    x=polished; value=objective(x); violation=polishViolation; flag=polishInfo.exitflag;
    selectedOptimality=polishInfo.firstOrderOptimality;selectedIterations=polishInfo.iterations;
    finalStateSource='polish';
end
info=struct('coordinates','full-mpcc-scaled','exitflag',flag,'iterations',selectedIterations, ...
    'constraintViolation',violation,'firstOrderOptimality',selectedOptimality, ...
    'objective',value,'seconds',toc(timer),'independentVariables',numel(free), ...
    'homotopyTrace',{traces},'finalRelaxation',stages(end), ...
    'acceptedAlpha',1,'acceptedCost',value);
info.polish=polishInfo; info.polishAccepted=polishAccepted;
info.finalStateSource=finalStateSource;
info.totalIterations=sum(cellfun(@(t)t.iterations,traces))+polishInfo.iterations;
info.rejectedMechanicsTrials=rejected;
info.lastMechanicsRejection=lastRejection;

    function v=expand(q)
        v=origin; v(free)=origin(free)+sc.*q;
    end
    function [a,b]=pairs(d)
        % Normalize with explicit physical units, independent of state scales.
        a=[d.gap/cfg.mpccLengthScaleMm;d.frictionW(:)/cfg.mpccLengthScaleMm; ...
            d.lambda/cfg.mpccLengthScaleMm];
        b=[d.fn/cfg.mpccForceScaleN;d.beta(:)/cfg.mpccForceScaleN; ...
            d.frictionConeSlack/cfg.mpccForceScaleN];
    end
    function [c,ceq]=constraints(q)
        try
            decoded=decode(expand(q)); [a,b]=pairs(decoded);
            [gc,ge]=plane_contact_constraints(decoded,cfg);
            % Interior tangency is itself a force-gated product. Relax it
            % with the other complementarity products; imposing it exactly
            % from the first stage can trap the fn=0 branch. Polishing and
            % final acceptance still check its unrelaxed equality.
            c=[-a;-b;a.*b-tau;gc;ge-tau;-ge-tau]; ceq=[];
        catch err
            if ~strcmp(err.identifier,'rod:CosseratEquilibrium'),rethrow(err);end
            rejected=rejected+1;
            lastRejection=err.message;
            c=ones(3*(m+2)+numel(initialGeometryC)+2*numel(initialGeometryEq),1)*1e6;
            ceq=[];
        end
    end
    function violation=fullViolation(decoded)
        [a,b]=pairs(decoded); [gc,ge]=plane_contact_constraints(decoded,cfg);
        violation=max([0;-a;-b;abs(a.*b);gc;abs(ge)]);
    end
    function value=safeObjective(q)
        try
            value=objective(expand(q));
        catch err
            if ~strcmp(err.identifier,'rod:CosseratEquilibrium'),rethrow(err);end
            rejected=rejected+1;lastRejection=err.message; value=Inf;
        end
    end
    function stop=progress(~,v,state)
        stop=false;interval=10;
        if isfield(cfg,'fminconProgressInterval'),interval=cfg.fminconProgressInterval;end
        if strcmp(state,'iter')&&mod(v.iteration,interval)==0
            fprintf('        MPCC tau %.1g iteration %d: cost %.5g, %.1f s, rejected mechanics %d\n', ...
                tau,v.iteration,v.fval,toc(stageTimer),rejected-stageRejected);
        end
    end
end
