function results = run_independent_truth_benchmark(truthMethod)
%RUN_INDEPENDENT_TRUTH_BENCHMARK Estimate loads from a different forward model.
% Frictionless planar energy equilibrium, finite mesh contact multipliers.
% This tests model mismatch; it is not validation of frictional dynamics.
if nargin<1,truthMethod='energy';end
assert(any(strcmp(truthMethod,{'energy','continuous'})),'Use energy or continuous.');
continuous=strcmp(truthMethod,'continuous');
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','video','inverse_results.mat'),'results'); cfg=d.results.config;
cfg.outputDir=fullfile(root,'out','stage2','independent_video_tip_gated');
if continuous,cfg.outputDir=fullfile(root,'out','completion','continuous');end
if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
runRecord=begin_result_run(cfg.outputDir);
cfg.scenarioName=['independent_' truthMethod '_video_tip']; cfg.video.enabled=false;
cfg.tipLoadN=[1;0;-1]; cfg.sensing.historySource='sparse-paired';
cfg.sensing.curvatureInterpolation='intrinsic-delta';
cfg.sensing.samplePeriodSeconds=0.02; cfg.sensing.planeNormalNoiseStdDeg=0;
cfg.forceSensor.subproblemCoordinates='mode-reduced'; cfg.forceSensor.useShapeOnlySeed=true;
if continuous
    % The 25-step inherited budget exhausted four of eight SQP calls on the
    % continuous-model fixture. Keep the legacy energy protocol unchanged.
    cfg.forceSensor.linearizedSolveMaxIter=100;
end
tube=make_experiment_tube(cfg); ns=numel(tube.s);
push=[0 10 14 18 20]; nt=numel(push);
mesh=linspace(0,200,201); mids=0.5*(mesh(1:end-1)+mesh(2:end));
if continuous
    % Output sampling is distinct from ODE integration. Include the exact
    % intrinsic-curvature breakpoint; use constitutive curvature, not a
    % finite difference across that discontinuity.
    mesh=unique([tube.s 120]);mids=0.5*(mesh(1:end-1)+mesh(2:end));
end
kappa=zeros(size(mids)); kappa(mids>=120)=1/30;
model=struct('sMm',mesh,'intrinsicCurvaturePerMm',kappa,'EINmm2',tube.kb, ...
    'baseXZ',[0;0],'baseAngleRad',0,'tipForceXZ',cfg.tipLoadN([1 3]), ...
    'planePointXZ',cfg.planePointMm([1 3]),'planeNormalXZ',cfg.planeNormal([1 3]));
f=struct('s',tube.s,'betaMm',push,'actuationMm',push,'frictionMu',zeros(1,nt), ...
    'phase',{repmat({'push'},1,nt)},'internalSampleIndices',1+round(push/0.1), ...
    'internalFrameCount',0,'p',zeros(3,ns,nt),'u',zeros(3,ns,nt), ...
    'baseTraj',repmat(eye(4),1,1,nt),'previousState',{cell(1,nt)}, ...
    'contactForceResultant',zeros(3,nt),'tipLoad',repmat(cfg.tipLoadN,1,nt), ...
    'contactArcLength',nan(1,nt));
equilibria=cell(2,nt); contactNodeCount=zeros(1,nt);
start=[];
for k=1:nt
    for phase=1:2
        z=push(k)-0.1*(phase==1); model.baseXZ=[0;z];
        if continuous
            eq=solve_planar_contact_shooting(model);
        else
            eq=solve_planar_energy_rod(model,start); start=eq.thetaRad(2:end);
        end
        fprintf('Independent frame %d pair %d: exit %d, stationarity %.3g, contact %.3f N\n', ...
            k,phase,eq.exitflag,eq.stationarityInfNmm,norm(eq.contactResultantXZ));
        assert(eq.exitflag>0 && eq.stationarityInfNmm<1e-3 && eq.maxPenetrationMm<1e-6, ...
            'Independent equilibrium has not converged.');
        equilibria{phase,k}=eq;
        u=tube.uhat;
        if continuous
            delta=eq.momentNmm/model.EINmm2;
            u(2,:)=u(2,:)+interp1(mesh,delta,tube.s,'linear');
        else
            delta=eq.segmentCurvaturePerMm(:)'-kappa;
            u(2,:)=u(2,:)+interp1(mids,delta,tube.s,'linear','extrap');
        end
        % Free-tip moment condition: no applied tip couple.
        u(2,end)=tube.uhat(2,end);
        p=zeros(3,ns); p([1 3],:)=interp1(mesh,eq.pXZ',tube.s,'pchip')';
        base=eye(4); base(3,4)=z;
        if phase==1
            f.previousState{k}=struct('u',u,'p',p,'T_base',base);
        else
            f.u(:,:,k)=u; f.p(:,:,k)=p; f.baseTraj(:,:,k)=base;
            f.contactForceResultant([1 3],k)=eq.contactResultantXZ;
            weights=vecnorm(eq.contactForceXZ); contactNodeCount(k)=sum(weights>1e-5);
            if sum(weights)>1e-5
                if continuous
                    f.contactArcLength(k)=eq.contactArcLengthMm;
                else
                    f.contactArcLength(k)=sum(mesh(2:end).*weights)/sum(weights);
                end
            end
        end
    end
end
f.totalForceResultant=f.contactForceResultant+f.tipLoad;
f.groundTruthDescription='Independent nonlinear energy minimization; KKT contact forces, frictionless planar static equilibrium, 1 mm mesh.';
if continuous
    f.groundTruthDescription=['Independent continuous ODE force/moment balance; ', ...
        'one smooth frictionless body contact, free tip moment, exact contact arclength. ', ...
        'Local admissible equilibrium; no global stability or frictional validation.'];
end
packet=simulate_sensor_packet(tube,f,cfg);
cfg.forceSensor.fbgIdx=packet.fbgIdx; cfg.forceSensor.normalReference=packet.planeNormal(:,1);
sensorInput=struct('tube',tube,'packet',packet,'config',sensor_estimator_config(cfg));
atomic_write_artifact(fullfile(cfg.outputDir,'sensor_input.mat'),'mat',struct('sensorInput',sensorInput));
atomic_write_artifact(fullfile(cfg.outputDir,'independent_forward.mat'),'mat', ...
    struct('f',f,'equilibria',{equilibria},'contactNodeCount',contactNodeCount,'model',model));
estimate=estimate_sensor_forces(sensorInput);
results=struct('config',cfg,'forward',f,'measurements',estimate.measurements,'ours',estimate.ours, ...
    'runRecord',runRecord,'quality',estimate.quality, ...
    'truthConsistency',struct('maxInequalityViolation',nan(1,nt),'maxEqualityResidual',nan(1,nt)));
results.provenance=struct('independentTruth',true,'truthMethod',truthMethod,'contactNodeCount',contactNodeCount, ...
    'scope',f.groundTruthDescription);
results.metrics.ours.finalRelativeErrorPct=100*norm( ...
    results.ours.totalForceResultant(:,end)-f.totalForceResultant(:,end))/norm(f.totalForceResultant(:,end));
try
    results.validation=validate_rod_plane_displacement_results(results);
    filename='inverse_results.mat';
catch info
    results.validation=struct('passed',false,'error',info.message);
    filename='failed_results.mat';
end
results=save_result_checkpoint(results,cfg.outputDir,filename);
export_inverse_audit(results); audit_sensor_stage1(cfg.outputDir,filename);
end
