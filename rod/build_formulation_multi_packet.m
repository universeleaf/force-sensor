function [input, truth] = build_formulation_multi_packet(scene, frameIndices, rotation)
%BUILD_FORMULATION_MULTI_PACKET Adapt independent planar truth to 3-D sensors.
% The estimate receives measured planes and sparse bending, never the planar
% solver's contact count/order, forces, contact points or locations.
if nargin<1||isempty(scene), scene=multi_contact_demo_scenes(); scene=scene(1); end
if nargin<2||isempty(frameIndices), frameIndices=[3 10]; end
if nargin<3, rotation=eye(3); end
[truth,~,sensors]=build_multi_contact_demo_truth(scene);
cfg=formulation_solver_config(run_rod_plane_force_sensing_experiment('sensor-config'));
frame=sensors.frames{1}; s=frame.model.sMm;
profile=[frame.model.intrinsicCurvaturePerMm,frame.model.intrinsicCurvaturePerMm(end)];
u0=[zeros(1,numel(s));profile;zeros(1,numel(s))];
tube=make_experiment_tube(struct('exposedLengthMm',s(end),'rod',struct('sMm',s,'intrinsicCurvaturePerMm',u0)));
assert(abs(mean(tube.kb)-frame.model.EINmm2)<1e-8,'rod:CalibrationMismatch','Truth and inverse stiffness differ.');
frames=frameIndices(:)'; T=numel(frames); P=size(frame.model.planePointXZ,2);
base=truth.basePose(:,:,frames); curv=zeros(2,numel(frame.fbgArcLengthMm),T);
planes=[frame.model.planePointXZ(1,:);zeros(1,P);frame.model.planePointXZ(2,:)];
noiseStream=RandStream('mt19937ar','Seed',scene.seed+10000);
normal=[frame.model.planeNormalXZ(1,:);zeros(1,P);frame.model.planeNormalXZ(2,:)]; normal=normal./vecnorm(normal);
for k=1:T
    base(1:3,1:3,k)=rotation*base(1:3,1:3,k); base(1:3,4,k)=rotation*base(1:3,4,k);
    curv(2,:,k)=sensors.frames{frames(k)}.curvaturePerMm;
    curv(1,:,k)=scene.curvatureNoiseStd*randn(noiseStream,1,size(curv,2));
end
packet=struct('schemaVersion',2,'sFbgMm',frame.fbgArcLengthMm,'observedCurvatureAxes',[1 2], ...
    'curvaturePerMm',curv,'curvatureStdPerMm',max(scene.curvatureNoiseStd,1e-7), ...
    'basePose',base,'timeSeconds',(frames-frames(1))*0.02,'processReferencePeriodSeconds',0.02, ...
    'planePointMm',repmat(rotation*planes,1,1,T),'planeNormal',repmat(rotation*normal,1,1,T), ...
    'planeCovariance',repmat(diag([0.05^2*ones(1,3),0.001^2*ones(1,3)]),1,1,P,T), ...
    'frictionMu',zeros(P,T));
input=struct('tube',tube,'packet',packet,'config',cfg);
truth.p=rotateArray(truth.p(:,:,frames),rotation);
truth.u=truth.u(:,:,frames); truth.basePose=base; truth.R=truth.R(:,:,:,frames);
for k=1:T, for j=1:numel(s), truth.R(:,:,j,k)=rotation*truth.R(:,:,j,k); end; end
truth.contactS=truth.contactS(:,frames);
truth.contactForces=rotateArray(truth.contactForces(:,:,frames),rotation);
truth.contactPoints=rotateArray(truth.contactPoints(:,:,frames),rotation);
truth.tipForce=rotation*truth.tipForce(:,frames); truth.totalForce=rotation*truth.totalForce(:,frames);
truth.planePoint=rotation*truth.planePoint; truth.planeNormal=rotation*truth.planeNormal;
truth.scope='Independent planar equilibrium embedded in a rotated 3-D world; not intrinsically out-of-plane loading.';
end
function b=rotateArray(a,Q), b=reshape(Q*reshape(a,3,[]),size(a)); end
