function [x, info] = solve_contact_mode_map(x0, objective, decode, bounds, scale, cfg, mode)
%SOLVE_CONTACT_MODE_MAP Eliminate variables fixed by the contact mode.
% Uses the same MAP objective and physical constraints as the full state.
% Inactive friction coordinates are identically zero, rather than redundant
% equality constraints on variables already sitting at a nonsmooth bound.
if isfield(cfg,'useMultiStart') && cfg.useMultiStart
    error('rod:UnsupportedReducedMultiStart', ...
        'useMultiStart requires subproblemCoordinates=''full''; the reduced solver supports one start.');
end
m=cfg.numFrictionDirs; betaIdx=8:7+m; tipIdx=9+m:11+m;
switch mode.name
    case 'no-contact'
        free=[1:6,tipIdx];
    case {'frictionless-contact','sliding-contact'}
        free=[1:7,tipIdx];
        if strcmp(mode.name,'sliding-contact')&&numel(mode.activeDirection)>1
            free=[free,betaIdx(mode.activeDirection(1:end-1))];
        end
    case {'sticking-contact','unresolved-contact'}
        free=[1:7,betaIdx,tipIdx];
    otherwise
        error('Mode reduction requires an explicit active set.');
end
% With zero cone slack (separation/frictionless contact), lambda need only
% bound tangential work. It is NOT forced to its minimum: the MAP prior can
% prefer a larger feasible value. Preserve this degree of freedom in MPCC
% polishing; the historical reduced solver retains its original convention.
freeLambda=isfield(cfg,'complementaritySolver')&&strcmp(cfg.complementaritySolver,'scholtes')&& ...
    ismember(mode.name,{'no-contact','frictionless-contact'});
if freeLambda,free=[free,8+m];end
% Appended nuisance coordinates (e.g. predecessor FBG values) remain free
% during polishing. Freezing them here would change the joint MAP problem.
free=[free,12+m:numel(x0)];
% Shift coordinates near x0 and scale only independent variables.
origin=x0(free); sc=scale(free);
lb=(bounds{1}(free)-origin)./sc; ub=(bounds{2}(free)-origin)./sc;
dependent=setdiff(1:numel(x0),free);
upperDependent=dependent(isfinite(bounds{2}(dependent)));
lowerDependent=dependent(isfinite(bounds{1}(dependent)) & bounds{1}(dependent)~=0);
opts=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',cfg.linearizedSolveMaxIter,'MaxFunctionEvaluations',4000, ...
    'OptimalityTolerance',1e-5,'ConstraintTolerance',1e-7,'StepTolerance',1e-8);
if isfield(cfg,'mechanicsModel')&&strcmp(cfg.mechanicsModel,'cosserat-shooting')
    opts.FiniteDifferenceStepSize=2e-5;
    opts.FiniteDifferenceType='central';
    if isfield(cfg,'nonlinearFiniteDifferenceStep')
        opts.FiniteDifferenceStepSize=cfg.nonlinearFiniteDifferenceStep;
    end
end
timer=tic;
rejected=0;
[initialC,initialEq]=computeConstraints(zeros(numel(free),1));
[y,value,exitflag,out]=fmincon(@safeObjective,zeros(numel(free),1), ...
    [],[],[],[],lb,ub,@constraints,opts);
x=expand(y);
[c,ceq]=constraints(y);
info=struct('coordinates','mode-reduced','exitflag',exitflag, ...
    'iterations',out.iterations,'functionCount',out.funcCount, ...
    'constraintViolation',max([0;c(:);abs(ceq(:))]), ...
    'firstOrderOptimality',out.firstorderopt,'objective',value, ...
    'seconds',toc(timer),'independentVariables',numel(free));
info.rejectedMechanicsTrials=rejected;

    function full=expand(v)
        full=x0; full(free)=origin+sc.*v;
        if ~freeLambda,full(8+m)=0;end
        switch mode.name
            case 'no-contact'
                full(7)=0; full(betaIdx)=0;
            case 'frictionless-contact'
                full(betaIdx)=0;
            case 'sliding-contact'
                active=betaIdx(mode.activeDirection);
                full(setdiff(betaIdx,active))=0;
                full(active(end))=cfg.currentMu*full(7)-sum(full(active(1:end-1)));
        end
        if ~freeLambda&&~ismember(mode.name,{'sticking-contact','unresolved-contact'})
            d=decode(full);
            full(8+m)=max(0,-min(d.D'*d.vTangential));
        end
    end

    function [c,ceq]=constraints(v)
        try
            [c,ceq]=computeConstraints(v);
        catch err
            if ~strcmp(err.identifier,'rod:CosseratEquilibrium'),rethrow(err);end
            rejected=rejected+1; c=ones(size(initialC))*1e6; ceq=ones(size(initialEq))*1e6;
        end
    end
    function value=safeObjective(v)
        try
            value=objective(expand(v));
        catch err
            if ~strcmp(err.identifier,'rod:CosseratEquilibrium'),rethrow(err);end
            rejected=rejected+1; value=Inf;
        end
    end
    function [c,ceq]=computeConstraints(v)
        full=expand(v); d=decode(full);
        switch mode.name
            case 'unresolved-contact'
                c=-d.frictionConeSlack; ceq=d.gap;
            case 'no-contact'
                c=-d.gap; ceq=[];
            case 'frictionless-contact'
                c=[]; ceq=d.gap;
            case 'sliding-contact'
                % The active direction must minimize tangential work.
                c=-d.frictionW(:);
                if numel(mode.activeDirection)>1,c=[c;-full(betaIdx(mode.activeDirection(end)))];end
                ceq=[d.gap;d.frictionW(mode.activeDirection)];
            case 'sticking-contact'
                t=contact_tangent(d.n); B=[t,cross(d.n,t)];
                c=-d.frictionConeSlack;
                ceq=[d.gap;B'*d.vTangential];
        end
        if freeLambda,c=[c(:);-d.frictionW(:)];end
        [geometryC,geometryEq]=plane_contact_constraints(d,cfg);
        % fn is identically zero in this mode, so tangency is identically
        % satisfied; a zero-gradient equality would make SQP rank deficient.
        if strcmp(mode.name,'no-contact'),geometryEq=[];end
        c=[c(:);geometryC]; ceq=[ceq(:);geometryEq];
        % Eliminating a coordinate must not discard its configured bounds.
        % Zero lower bounds already hold by construction for beta/lambda/fn.
        c=[c(:);full(upperDependent)-bounds{2}(upperDependent); ...
            bounds{1}(lowerDependent)-full(lowerDependent)];
    end
end
