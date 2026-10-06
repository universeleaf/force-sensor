function analyze_bottleneck_audit(folder)
% Independent residual, sensitivity, and geometry-perturbation measurements.
[root,paths]=setup_tsfs;
if nargin<1, folder=fullfile(paths.results,'baseline'); end
report=struct;
if isfile(fullfile(folder,'sliding_results.mat'))
    d=load(fullfile(folder,'sliding_results.mat')); tr=load(fullfile(folder,'sliding_input.mat'),'truth');
    o=d.output.ours; m=d.output.measurements; tube=d.sensorInput.tube; nt=numel(o.frameSeconds);
    rows=cell(1,nt);
    for k=1:nt
        tube.T_base=m.baseTraj(:,:,k);
        opts=struct('relativeTolerance',2e-8,'momentToleranceNmm',2e-5,'collisionStepMm',0.1);
        sh=solve_cosserat_force_map(tube,o.contactArcLength(k),o.contactForceResultant(:,k),o.tipForce(:,k),opts);
        n=o.planeNormal(:,k); p0=o.state(1:3,k); fn=n'*o.contactForceResultant(:,k);
        ft=o.contactForceResultant(:,k)-n*fn;
        prev=sample_integrated_shape(tube,m.previousShape{k},o.contactArcLength(k));
        slip=(eye(3)-n*n')*(sh.pc-prev);
        forceG=o.forceSensitivity{k}.forceToBendingMatrix;
        [~,sv,v]=svd(forceG,'econ'); singular=diag(sv); weak=v(:,end);
        rows{k}=struct('frame',k,'seconds',o.frameSeconds(k), ...
            'contactErrorN',norm(o.contactForceResultant(:,k)-tr.truth.contactForce(:,k)), ...
            'tipErrorN',norm(o.tipForce(:,k)-tr.truth.tipForce(:,k)), ...
            'arcErrorMm',o.contactArcLength(k)-tr.truth.contactS(k), ...
            'shapeTruthRmseMm',sqrt(mean(sum((sh.p-tr.truth.p(:,:,k)).^2,1))), ...
            'denseMinGapMm',min([n'*(sh.collisionP-p0),n'*(sh.pc-p0)]), ...
            'tangency',n'*sh.contactTangent,'terminalMomentNmm',sh.tipMomentResidualNmm, ...
            'slipMm',slip,'frictionN',ft,'frictionWorkNmm',ft'*slip, ...
            'circularConeViolationN',max(0,norm(ft)-m.frictionMu(k)*fn), ...
            'circularDissipationResidualNmm',ft'*slip+m.frictionMu(k)*fn*norm(slip), ...
            'curvatureRmsePerMm',sqrt(mean((sh.u(1:2,m.fbgIdx)-m.uSparse(1:2,:,k)).^2,'all')), ...
            'forceSingularValues',singular,'forceCondition',singular(1)/singular(end), ...
            'weakForceDirection',weak,'weakTotalNorm',norm(weak(1:3)+weak(4:6)), ...
            'contactToTipMm',tube.s(end)-o.contactArcLength(k), ...
            'sensorsAfterContact',sum(tube.s(m.fbgIdx)>o.contactArcLength(k)), ...
            'conditionalForceStdN',o.forceSensitivity{k}.conditionalNoiseStdN);
    end
    report.sliding=rows;
    % Derivative consistency at perturbations comparable to the scaled SQP
    % steps. These are diagnostic evaluations, not altered optimizations.
    k=1; tube.T_base=m.baseTraj(:,:,k); f=[o.contactForceResultant(:,k);o.tipForce(:,k)];
    sizes=[1e-8,1e-7,1e-6,1e-4,1e-3]; matrices=cell(size(sizes));
    opts=struct('relativeTolerance',d.output.config.forceSensor.mechanicsRelativeTolerance, ...
        'momentToleranceNmm',d.output.config.forceSensor.mechanicsMomentToleranceNmm);
    for hidx=1:numel(sizes)
        h=sizes(hidx); G=zeros(2*numel(m.fbgIdx),6);
        for j=1:6
            a=f;b=f;a(j)=a(j)+h;b(j)=b(j)-h;
            qa=solve_cosserat_force_map(tube,o.contactArcLength(k),a(1:3),a(4:6),opts);
            qb=solve_cosserat_force_map(tube,o.contactArcLength(k),b(1:3),b(4:6),opts);
            du=(qa.u(1:2,m.fbgIdx)-qb.u(1:2,m.fbgIdx))/(2*h);G(:,j)=du(:);
        end
        matrices{hidx}=G;
    end
    ref=matrices{end-1}; differences=cellfun(@(G)norm(G-ref,'fro')/norm(ref,'fro'),matrices);
    report.slidingDerivativeSteps=struct('forceStepN',sizes,'relativeDifferenceFrom1e4',differences);
end

d=load(fullfile(folder,'two_contact_input.mat')); es=load(fullfile(folder,'planar_results.mat'));
% Nonlinear force sensitivity: differentiate the IVP then eliminate its base
% moment with the terminal equilibrium equality (implicit function theorem).
k=6; input=d.sensorInput.frames{k}; e=es.estimates{k}; model=input.model; q=e.state;
K=numel(q.contactS); scale=model.EINmm2/model.sMm(end);
x=[q.baseMomentNmm/scale;q.normalForceN(:);q.contactS(:);q.tipForceXZ(:)];
[Jcurv,Jmom,Jgeom]=planarJac(model,input.fbgArcLengthMm,x,K,scale);
forceCols=[2:K+1,2*K+2:2*K+3]; arcCols=K+2:2*K+1;
G=Jcurv(:,forceCols)-Jcurv(:,1)*(Jmom(:,1)\Jmom(:,forceCols));
[~,S,V]=svd(G,'econ'); values=diag(S); w=V(:,end);
stdForce=input.curvatureStdPerMm*sqrt(diag(pinv(G'*G)));
geomFree=[forceCols,arcCols];
Geq=Jcurv(:,geomFree)-Jcurv(:,1)*(Jmom(:,1)\Jmom(:,geomFree));
JG=Jgeom(:,geomFree)-Jgeom(:,1)*(Jmom(:,1)\Jmom(:,geomFree));
% Dimensionless columns: 1 N force and 1 mm arc perturbations. Curvature
% data only, followed by curvature plus the existing geometric weights.
report.planarSensitivity=struct('frame',k,'forceSingularValues',values, ...
    'forceCondition',values(1)/values(end),'weakForceDirection',w, ...
    'forceStdAtDeclaredNoiseN',stdForce,'forceStdAt2p5e5NoiseN',stdForce*2.5e-5/input.curvatureStdPerMm, ...
    'forceAndArcDataSingularValues',svd(Geq/input.curvatureStdPerMm), ...
    'forceAndArcWithGeometrySingularValues',svd([Geq/input.curvatureStdPerMm;diag([1e3*ones(1,K),1e4*ones(1,K)])*JG]), ...
    'scope','Local equilibrated derivative, fixed normals and planes, no force prior; singular values depend on stated units.');

% Direct same-packet environmental misspecification and latent-offset trials.
rows={};
for declaredSigma=[input.curvatureStdPerMm,2.5e-5]
for offset=[0,0.1,1]
    for sigma=[0,1]
        if offset==0 && sigma==1, continue; end
        trial=input; normal=trial.model.planeNormalXZ(:,1); normal=normal/norm(normal);
        trial.curvatureStdPerMm=declaredSigma;
        trial.model.planePointXZ(:,1)=trial.model.planePointXZ(:,1)+offset*normal;
        timer=tic; estimate=estimate_planar_multi_contact(trial,struct('planePointStdMm',sigma)); elapsed=toc(timer);
        delta=estimate.state.contactForceXZ-d.truth.contactForces([1 3],:,k);
        rows{end+1}=struct('offsetMm',offset,'latentOffsetStdMm',sigma,'declaredFbgStdPerMm',declaredSigma,'seconds',elapsed, ...
            'contactRmseN',sqrt(mean(sum(delta.^2,1))),'tipErrorN',norm(estimate.state.tipForceXZ-d.truth.tipForce([1 3],k)), ...
            'totalErrorN',norm(sum(estimate.state.contactForceXZ,2)+estimate.state.tipForceXZ-d.truth.totalForce([1 3],k)), ...
            'arcErrorMm',estimate.state.contactS-d.truth.contactS(:,k)','offsetEstimateMm',estimate.planePointOffsetMm, ...
            'maxScaledResidual',estimate.maxScaledResidual,'audit',estimate.audit,'exitflag',estimate.exitflag, ...
            'iterations',estimate.iterations,'requiresReview',estimate.requiresReview);
    end
end
end
report.geometryPerturbations=rows;
atomic_write_artifact(fullfile(folder,'independent_analysis.json'),'json',report);
save(fullfile(folder,'independent_analysis.mat'),'report','-v7.3');
fprintf('INDEPENDENT ANALYSIS COMPLETE\n');
end

function [Ju,Jm,Jg]=planarJac(model,fbgs,x,K,scale)
n=numel(x); Ju=zeros(numel(fbgs),n); Jm=zeros(1,n); Jg=zeros(2*K,n);
for j=1:n
    h=1e-5*max(1,abs(x(j))); a=x; b=x; a(j)=a(j)+h; b(j)=b(j)-h;
    qa=evalState(a); qb=evalState(b);
    Ju(:,j)=(qa.curvaturePerMm-qb.curvaturePerMm)'/(2*h);
    Jm(:,j)=(qa.tipMomentResidualNmm-qb.tipMomentResidualNmm)/(2*h);
    nc=model.planeNormalXZ(:,model.contactPlaneIndex); nc=nc./vecnorm(nc);
    Jg(:,j)=[sum(nc.*(qa.contactPointsXZ-qb.contactPointsXZ),1)'; ...
        sum(nc.*(qa.contactTangentXZ-qb.contactTangentXZ),1)']/(2*h);
end
    function q=evalState(v)
        q=integrate_planar_multi_contact(model,v(1)*scale,v(K+2:2*K+1),v(2:K+1),v(2*K+2:2*K+3),fbgs);
    end
end
