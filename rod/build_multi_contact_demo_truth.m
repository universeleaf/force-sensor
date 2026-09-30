function [truth, scene, sensorInput] = build_multi_contact_demo_truth(scene, seed)
%BUILD_MULTI_CONTACT_DEMO_TRUTH Solve a fixed planar K-contact environment.
% The unknown contact positions, normal reactions, and base moment come from
% solve_planar_multi_contact; they are not prescribed by the scene.
if nargin<1 || isempty(scene), scene=multi_contact_demo_scenes(); scene=scene(1); end
if nargin<2 || isempty(seed), seed=scene.seed; end
L=scene.lengthMm; s=linspace(0,L,121);
intrinsic=scene.intrinsicCurvature*(-1).^floor((s(1:end-1)+1e-8)/scene.segmentLengthMm);
run_rod_plane_force_sensing_experiment('sensor-config'); % registers CreatTube/getTubeK dependency
tube=make_experiment_tube(struct('exposedLengthMm',L));
EINmm2=mean(tube.kb);
K=numel(scene.contactPlaneIndex); nt=numel(scene.baseMotionMm); ns=numel(s);
p=zeros(3,ns,nt); u=zeros(3,ns,nt); R=zeros(3,3,ns,nt);
contactPoints=zeros(3,K,nt); contactForces=zeros(3,K,nt); tipForce=zeros(3,nt);
basePose=repmat(eye(4),1,1,nt); contactS=zeros(K,nt); normalForce=zeros(K,nt);
gap=zeros(K,nt); penetration=zeros(1,nt); momentResidual=zeros(1,nt); audit=cell(1,nt);
frames=cell(1,nt); fbgIndices=unique(round(linspace(2,ns-1,24)));
random=RandStream('mt19937ar','Seed',seed);
for k=1:nt
    model=struct('sMm',s,'intrinsicCurvaturePerMm',intrinsic,'EINmm2',EINmm2, ...
        'baseXZ',scene.baseOriginXZ+[scene.baseMotionMm(k);0], ...
        'baseAngleRad',scene.baseAngleRad,'planePointXZ',scene.planePointXZ, ...
        'planeNormalXZ',scene.planeNormalXZ,'contactPlaneIndex',scene.contactPlaneIndex, ...
        'tipForceXZ',scene.tipForceXZ,'relativeTolerance',1e-9);
    state=solve_planar_multi_contact(model);
    a=scene.baseAngleRad;
    basePose(:,:,k)=[cos(a) 0 sin(a) model.baseXZ(1);0 1 0 0;-sin(a) 0 cos(a) model.baseXZ(2);0 0 0 1];
    p(1,:,k)=interp1(state.sMm,state.pXZ(1,:),s,'linear');
    p(3,:,k)=interp1(state.sMm,state.pXZ(2,:),s,'linear');
    u(2,:,k)=interp1(state.sMm,state.curvaturePerMm,s,'linear');
    angles=interp1(state.sMm,state.thetaRad,s);
    for z=1:ns
        a=angles(z); R(:,:,z,k)=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
    end
    inverseModel=rmfield(model,'tipForceXZ');
    frames{k}=struct('model',inverseModel,'fbgArcLengthMm',s(fbgIndices), ...
        'curvaturePerMm',u(2,fbgIndices,k)+scene.curvatureNoiseStd*randn(random,1,numel(fbgIndices)), ...
        'curvatureStdPerMm',max(scene.curvatureNoiseStd,1e-7));
    contactPoints(1,:,k)=state.contactPointsXZ(1,:); contactPoints(3,:,k)=state.contactPointsXZ(2,:);
    contactForces(1,:,k)=state.contactForceXZ(1,:); contactForces(3,:,k)=state.contactForceXZ(2,:);
    tipForce(:,k)=[scene.tipForceXZ(1);0;scene.tipForceXZ(2)];
    contactS(:,k)=state.contactS(:); normalForce(:,k)=state.normalForceN(:);
    gap(:,k)=state.audit.contactGapMm(:); penetration(k)=state.audit.maxPenetrationMm;
    momentResidual(k)=state.audit.tipMomentResidualNmm; audit{k}=state.audit;
end
planePointXZ=scene.planePointXZ(:,scene.contactPlaneIndex);
planeNormalXZ=scene.planeNormalXZ(:,scene.contactPlaneIndex);
planePoint=[planePointXZ(1,:);zeros(1,K);planePointXZ(2,:)];
planeNormal=[planeNormalXZ(1,:);zeros(1,K);planeNormalXZ(2,:)];
truth=struct('sMm',s,'p',p,'u',u,'R',R,'basePose',basePose,'contactS',contactS, ...
    'contactPoints',contactPoints,'contactForces',contactForces, ...
    'contactForce',squeeze(sum(contactForces,2)),'tipForce',tipForce, ...
    'totalForce',squeeze(sum(contactForces,2))+tipForce,'planePoint',planePoint, ...
    'planeNormal',planeNormal,'contactGapMm',gap,'maxPenetrationMm',penetration, ...
    'tipMomentResidualNmm',momentResidual,'normalForceN',normalForce, ...
    'audit',{audit},'kind',scene.id, ...
    'scope','Fixed-environment planar multi-contact forward equilibrium; each state solved independently.');
scene.contactCount=K; scene.seed=seed; scene.sMm=s; scene.intrinsicCurvaturePerMm=intrinsic;
sensorInput=struct('frames',{frames},'scope','Sparse curvature, calibrated rod/base and fixed planes; known contact count/order; no force/contact-location labels.');
end
