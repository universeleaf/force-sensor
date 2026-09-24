function [seed,info]=refine_cosserat_shape_seed(tube,uSparse,fbgIdx,initial,cfg)
% Refine [s; Fc(3); Fe(3)] against CURRENT sparse bending observations.
% This is an initialization proposal only: no environment/contact mode is
% imposed here. The complete MAP still scores it and enforces all physics.
% No true shape, contact location or force is accepted by this interface.
initial=initial(:);seed=initial;timer=tic;rejected=0;
axes=cfg.curvatureObservedAxes;target=uSparse(axes,:);
forceStd=[repmat(cfg.priorStd.normalForceN,3,1);repmat(cfg.priorStd.tipForceN,3,1)];
sc=[max(1,(tube.s(end)-tube.s(1))/20);ones(6,1)];
opts=struct('relativeTolerance',cfg.mechanicsRelativeTolerance, ...
    'momentToleranceNmm',cfg.mechanicsMomentToleranceNmm);
if isfield(cfg,'mechanicsMaxRhsEvaluations'),opts.maxRhsEvaluations=cfg.mechanicsMaxRhsEvaluations;end
lo=[(tube.s(1)-initial(1))/sc(1);-inf(6,1)];
hi=[(tube.s(end)-initial(1))/sc(1);inf(6,1)];
settings=optimoptions('lsqnonlin','Display','off','MaxIterations',40,'MaxFunctionEvaluations',600, ...
    'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-5, ...
    'FunctionTolerance',1e-10,'StepTolerance',1e-9,'OptimalityTolerance',1e-7);
info=struct('accepted',false,'exitflag',NaN,'initialSquaredResidual',Inf, ...
    'finalSquaredResidual',Inf,'rejectedMechanicsTrials',0,'seconds',0);
r0=residual(zeros(7,1));info.initialSquaredResidual=sum(r0.^2);
if info.initialSquaredResidual<1e11
    [y,value,~,flag,out]=lsqnonlin(@residual,zeros(7,1),lo,hi,settings);
    info.exitflag=flag;info.finalSquaredResidual=value;info.iterations=out.iterations;
    if isfinite(value)&&value<info.initialSquaredResidual
        seed=initial+sc.*y;info.accepted=true;
    end
end
info.rejectedMechanicsTrials=rejected;info.seconds=toc(timer);
    function r=residual(y)
        v=initial+sc.*y;
        try
            shape=solve_cosserat_force_map(tube,v(1),v(2:4),v(5:7),opts);
            errorU=shape.u(axes,fbgIdx)-target;
            r=[errorU(:)/cfg.measurementStd.curvature;v(2:7)./forceStd];
        catch err
            if ~strcmp(err.identifier,'rod:CosseratEquilibrium'),rethrow(err);end
            rejected=rejected+1;r=ones(numel(target)+6,1)*1e6;
        end
    end
end
