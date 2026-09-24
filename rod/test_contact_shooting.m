function report = test_contact_shooting()
root=fileparts(fileparts(mfilename('fullpath')));
free=struct('sMm',linspace(0,100,81),'intrinsicCurvaturePerMm',zeros(1,80), ...
    'EINmm2',200700,'baseXZ',[0;0],'baseAngleRad',0,'tipForceXZ',[0.01;0]);
r=solve_planar_contact_shooting(free);
analytical=free.tipForceXZ(1)*100^3/(3*free.EINmm2);
report.smallDeflectionRelativeError=abs(r.pXZ(1,end)-analytical)/analytical;
assert(report.smallDeflectionRelativeError<5e-6&&r.stationarityInfNmm<1e-5);
d=load(fullfile(root,'out','stage2','independent_video_tip_gated','independent_forward.mat'));
model=d.model;push=[14 18 20];cases=cell(1,3);
report.toleranceForceDifferenceN=zeros(1,3);
report.toleranceTipDifferenceMm=zeros(1,3);
report.globalMomentBalanceErrorNmm=zeros(1,3);
for j=1:3
    model.baseXZ=[0;push(j)];r=solve_planar_contact_shooting(model);
    assert(strcmp(r.mode,'smooth-body-contact')&&r.contactArcLengthMm>120&&r.contactArcLengthMm<200);
    assert(r.contactResultantXZ(2)<0&&r.maxPenetrationMm<1e-6);
    tighter=solve_planar_contact_shooting(model,1e-12);
    report.toleranceForceDifferenceN(j)=norm(tighter.contactResultantXZ-r.contactResultantXZ);
    report.toleranceTipDifferenceMm(j)=norm(tighter.pXZ(:,end)-r.pXZ(:,end));
    armTip=r.pXZ(:,end)-model.baseXZ;armContact=r.contactPointXZ-model.baseXZ;
    moment=armTip(2)*model.tipForceXZ(1)-armTip(1)*model.tipForceXZ(2)+ ...
        armContact(2)*r.contactResultantXZ(1)-armContact(1)*r.contactResultantXZ(2);
    report.globalMomentBalanceErrorNmm(j)=abs(r.momentNmm(1)-moment);
    assert(report.toleranceForceDifferenceN(j)<1e-6&&report.toleranceTipDifferenceMm(j)<1e-6&& ...
        report.globalMomentBalanceErrorNmm(j)<1e-5,'Continuous reference failed independent accuracy checks.');
    cases{j}=r;
    fprintf('Continuous push %.1f: force %.8g N, contact %.6f mm, boundary residual %.3g\n', ...
        push(j),norm(r.contactResultantXZ),r.contactArcLengthMm,r.stationarityInfNmm);
end
% Rigid rotation must rotate positions/forces without changing contact arc.
a=0.4;R=[cos(a) sin(a);-sin(a) cos(a)];model=d.model;model.baseXZ=[0;18];
original=cases{2};model.baseXZ=R*model.baseXZ;model.baseAngleRad=a;
model.tipForceXZ=R*model.tipForceXZ;model.planePointXZ=R*model.planePointXZ;
model.planeNormalXZ=R*model.planeNormalXZ;rotated=solve_planar_contact_shooting(model);
report.rotationPositionErrorMm=max(vecnorm(rotated.pXZ-R*original.pXZ));
report.rotationForceErrorN=norm(rotated.contactResultantXZ-R*original.contactResultantXZ);
assert(report.rotationPositionErrorMm<1e-6&&report.rotationForceErrorN<1e-6);
report.cases=cases;report.passed=true;
folder=fullfile(root,'out','completion');if ~isfolder(folder),mkdir(folder);end
atomic_write_artifact(fullfile(folder,'continuous_checks.json'),'json',report);
disp(report);
end
