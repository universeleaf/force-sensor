function test_history_resolution()
%TEST_HISTORY_RESOLUTION Noise propagation and cone-only branch invariants.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
d=load(fullfile(root,'out','stage1','video_noise_gated_02','sensor_input.mat'),'sensorInput');
input=d.sensorInput; cfg=input.config; tube=input.tube;
automatic=cfg;
if isfield(automatic.forceSensor,'historyCurvatureStdPerMm')
    automatic.forceSensor=rmfield(automatic.forceSensor,'historyCurvatureStdPerMm');
end
automatic.sensing.curvatureNoiseStd=5e-5;
calibrated=sensor_estimator_config(automatic);
assert(calibrated.forceSensor.historyCurvatureStdPerMm==5e-5,'Noise calibration was dropped at the estimator boundary.');
automatic.forceSensor.historyCurvatureStdPerMm=0;
explicit=sensor_estimator_config(automatic);
assert(explicit.forceSensor.historyCurvatureStdPerMm==0,'An explicit deterministic-history ablation was overwritten.');
cfg.forceSensor.historyCurvatureStdPerMm=5e-5;
cfg.forceSensor.fbgIdx=input.packet.fbgIdx;
m=measurements_from_sensor_packet(tube,input.packet,cfg); k=4;
n=m.planeNormalMeasured(:,k); [~,j]=min(abs(n'*(m.p(:,:,k)-m.planePointMeasured(:,k))));
C=paired_contact_uncertainty(tube,m.u(:,:,k),m.baseTraj(:,:,k),m.previousShape{k},cfg,tube.s(j),n);
assert(norm(C-C','fro')<1e-12 && min(eig(C))>=-1e-12,'Slip covariance is not positive semidefinite.');
assert(norm(C*n)<1e-10,'Tangential covariance leaked into the normal direction.');
cfg.forceSensor.historyCurvatureStdPerMm=1e-4;
C2=paired_contact_uncertainty(tube,m.u(:,:,k),m.baseTraj(:,:,k),m.previousShape{k},cfg,tube.s(j),n);
assert(norm(C2-4*C,'fro')<1e-10,'Variance does not scale with squared sensor noise.');
cfg.forceSensor.historyCurvatureStdPerMm=0;
C0=paired_contact_uncertainty(tube,m.u(:,:,k),m.baseTraj(:,:,k),m.previousShape{k},cfg,tube.s(j),n);
assert(all(C0==0,'all'),'Zero noise changed the deterministic mode threshold.');

angles=(0:15)*2*pi/16; D=[cos(angles);sin(angles);zeros(1,16)];
ambiguous=friction_mode_resolution([0.675;0;0],D,diag([0.204^2,0.204^2,0]),0.005,3);
assert(ambiguous.observedSlipMm>ambiguous.slipResolutionMm && ~ambiguous.slidingResolved, ...
    'Magnitude significance was mistaken for certainty of a discrete friction ray.');
resolved=friction_mode_resolution([0.675;0;0],D,diag([0.001^2,0.001^2,0]),0.005,3);
assert(resolved.slidingResolved && resolved.activeDirection==9, ...
    'A well-resolved sliding direction was unnecessarily discarded.');

% An unresolved displacement must not force friction against the noisy
% direction, but its beta must still obey the polyhedral Coulomb cone.
count=2; nx=11+count; x=zeros(nx,1); x(7)=1;
lo=-inf(nx,1); hi=inf(nx,1); lo(7:8+count)=0;
lo(7)=1; hi(7)=1;
solver=struct('numFrictionDirs',count,'currentMu',0.5,'linearizedSolveMaxIter',50);
mode=struct('name','unresolved-contact','activeDirection',[]);
[answer,info]=solve_contact_mode_map(x,@(v)(v(8)-2)^2+v(9)^2,@decode, ...
    {lo,hi},ones(nx,1),solver,mode);
assert(info.constraintViolation<1e-7 && abs(answer(8)-0.5)<1e-5 && abs(answer(9))<1e-5, ...
    'Unresolved contact discarded the friction cone or imposed noisy slip kinematics.');
assert(answer(8+count)==0,'Unresolved mode inferred a fictitious slip multiplier.');
rejected=false;
try
    validate_rod_plane_displacement_results(struct('ours',struct('frictionKinematicsEnforced',false)));
catch problem
    rejected=strcmp(problem.identifier,'rod:UnresolvedFrictionKinematics');
end
assert(rejected,'The strict full-friction validator certified an unresolved mode.');
disp('History covariance scaling, tangency and unresolved-mode cone checks passed.');
    function d=decode(v)
        d=struct('n',[0;0;1],'D',[1 -1;0 0;0 0],'vTangential',[10;0;0], ...
            'gap',v(1),'frictionW',[10;-10]+v(8+count), ...
            'frictionConeSlack',0.5*v(7)-sum(v(8:9)));
    end
end
