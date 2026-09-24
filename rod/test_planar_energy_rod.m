function report = test_planar_energy_rod()
% Independent-model checks: analytical small deflection, derivatives, mesh.
root=fileparts(fileparts(mfilename('fullpath')));
model=struct('sMm',linspace(0,100,81),'intrinsicCurvaturePerMm',zeros(80,1), ...
    'EINmm2',200700,'baseXZ',[0;0],'baseAngleRad',0,'tipForceXZ',[0.01;0], ...
    'checkDerivatives',true);
free=solve_planar_energy_rod(model);
analytical=0.01*100^3/(3*model.EINmm2);
report.smallDeflectionRelativeError=abs(free.pXZ(1,end)-analytical)/analytical;
assert(free.exitflag>0 && report.smallDeflectionRelativeError<5e-4);
assert(free.objectiveGradientRelativeError<1e-6);
model.tipForceXZ=[1;0]; model.planePointXZ=[0.8;0]; model.planeNormalXZ=[-1;0];
coarse=solve_planar_energy_rod(model);
assert(coarse.constraintGradientRelativeError<1e-6);
model.sMm=linspace(0,100,161); model.intrinsicCurvaturePerMm=zeros(160,1);
fine=solve_planar_energy_rod(model);
report.contactReactionCoarseN=coarse.contactResultantXZ;
report.contactReactionFineN=fine.contactResultantXZ;
report.meshReactionDifferenceN=norm(coarse.contactResultantXZ-fine.contactResultantXZ);
report.maxPenetrationMm=max(coarse.maxPenetrationMm,fine.maxPenetrationMm);
report.maxStationarityInfNmm=max(coarse.stationarityInfNmm,fine.stationarityInfNmm);
assert(coarse.exitflag>0 && fine.exitflag>0 && report.maxPenetrationMm<1e-7);
assert(report.meshReactionDifferenceN<1e-3 && report.maxStationarityInfNmm<1e-4);
assert(coarse.contactResultantXZ(1)<0,'Obstacle force has the wrong sign.');
report.passed=true;
folder=fullfile(root,'out','stage2'); if ~isfolder(folder),mkdir(folder);end
fid=fopen(fullfile(folder,'independent_model_checks.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
disp(report);
end
