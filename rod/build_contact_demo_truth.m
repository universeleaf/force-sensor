function [input,truth]=build_contact_demo_truth(scene)
% Independent planar ODE solves contact location and reaction from geometry.
% Full-rod nonpenetration, tangency, free-end moment and unilateral checks
% are enforced by solve_planar_contact_shooting before any inverse is run.
cfg=run_rod_plane_force_sensing_experiment('sensor-config');
cfg=formulation_solver_config(cfg);
cfg.forceSensor.historyCurvatureStdPerMm=scene.curvatureNoiseStd;
cfg=calibrate_fbg_likelihood(cfg,scene.curvatureNoiseStd,1e-6);
cfg.forceSensor.showProgress=true;
cfg.sensing.curvatureInterpolation='intrinsic-delta';
cfg.sensing.shapeSmoothing=0;
s=unique([linspace(0,scene.lengthMm,201),scene.straightMm]);
u0=zeros(3,numel(s)); u0(2,s>=scene.straightMm)=1/scene.radiusMm;
tube=CreatTube(scene.lengthMm,s,u0);
tube.kb=tube.kb*scene.stiffnessScale; tube.kt=tube.kt*scene.stiffnessScale;
angle=deg2rad(scene.worldRotationYDeg);
Q=[cos(angle) 0 sin(angle);0 1 0;-sin(angle) 0 cos(angle)];
tube.T_base(1:3,1:3)=Q;
mid=(s(1:end-1)+s(2:end))/2; intrinsic=double(mid>=scene.straightMm)/scene.radiusMm;
model=struct('sMm',s,'intrinsicCurvaturePerMm',intrinsic, ...
    'EINmm2',tube.kb,'baseXZ',[0;0],'baseAngleRad',0, ...
    'tipForceXZ',scene.tipForceXZ,'planePointXZ',scene.planePointXZ, ...
    'planeNormalXZ',scene.planeNormalXZ,'tangentialNormalRatio',0);
if scene.sliding,model.tangentialNormalRatio=scene.frictionMu;end
nt=numel(scene.pushMm); ns=numel(s);
p=zeros(3,ns,nt); u=p; bases=repmat(tube.T_base,1,1,nt);
previous=cell(1,nt); eqs=cell(2,nt);
fc=zeros(3,nt);fe=fc;pc=fc;contactS=nan(1,nt);
planePoint=Q*[scene.planePointXZ(1);0;scene.planePointXZ(2)];
normal=Q*[scene.planeNormalXZ(1);0;scene.planeNormalXZ(2)];
for k=1:nt
    for phase=1:2
        old=phase==1;
        if scene.sliding
            % Translating an equilibrium parallel to a fixed infinite plane
            % preserves equilibrium. These are prescribed sliding pairs,
            % not a solved dynamic stick-to-slip transition.
            model.baseXZ=[0;scene.pushMm(k)];
            offset=[-0.5*(k-1)+scene.slipMm*old;0;0];
        else
            model.baseXZ=[0;scene.pushMm(k)-0.1*old]; offset=zeros(3,1);
        end
        eq=solve_planar_contact_shooting(model);
        eqs{phase,k}=eq;
        worldP=Q*([eq.pXZ(1,:);zeros(1,ns);eq.pXZ(2,:)]+offset);
        base=tube.T_base; base(1:3,4)=Q*([model.baseXZ(1);0;model.baseXZ(2)]+offset);
        curve=u0; curve(2,:)=curve(2,:)+eq.momentNmm/tube.kb;
        if old
            previous{k}=struct('p',worldP,'u',curve,'T_base',base);
        else
            p(:,:,k)=worldP; u(:,:,k)=curve; bases(:,:,k)=base;
            fc(:,k)=Q*[eq.contactResultantXZ(1);0;eq.contactResultantXZ(2)];
            fe(:,k)=Q*[scene.tipForceXZ(1);0;scene.tipForceXZ(2)];
            pc(:,k)=Q*([eq.contactPointXZ(1);0;eq.contactPointXZ(2)]+offset);
            contactS(k)=eq.contactArcLengthMm;
        end
    end
end
forward=struct('betaMm',scene.pushMm,'internalSampleIndices',1+round(scene.pushMm/0.1), ...
    'u',u,'baseTraj',bases,'previousState',{previous}, ...
    'frictionMu',scene.frictionMu*ones(1,nt));
sim=struct('randomSeed',scene.seed,'planePointMm',planePoint,'planeNormal',normal);
sim.sensing=struct('numFbgPoints',24,'samplePeriodSeconds',0.02, ...
    'curvatureNoiseStd',scene.curvatureNoiseStd,'planeOffsetBiasMm',0, ...
    'planeOffsetNoiseStdMm',scene.planeNoiseStdMm,'planeNormalNoiseStdDeg',0);
packet=simulate_sensor_packet(tube,forward,sim);
% Coordinate-independent environment likelihood; nonzero floors express
% model uncertainty even for synthetic zero-noise observations.
sigP=max(0.03,scene.planeNoiseStdMm); sigN=0.001;
packet.planeCovariance=repmat(diag([sigP^2*ones(1,3),sigN^2*ones(1,3)]),1,1,nt);
input=struct('tube',tube,'packet',packet,'config',cfg);
truth=struct('p',p,'u',u,'basePose',bases,'contactForce',fc, ...
    'tipForce',fe,'totalForce',fc+fe,'contactPoint',pc,'contactS',contactS, ...
    'planePoint',planePoint,'planeNormal',normal,'sMm',s, ...
    'equilibria',{eqs},'method','Independent continuous planar shooting; current inverse is 3-D Cosserat shooting.', ...
    'scope','One smooth body contact, planar static equilibrium, local root. Prescribed sliding pairs, no stick/slip transition proof.');
end
