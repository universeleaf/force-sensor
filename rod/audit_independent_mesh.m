function report = audit_independent_mesh()
% One further mesh refinement on the actual bent-rod contact geometry.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'out','stage2','independent_video_tip');
d=load(fullfile(folder,'independent_forward.mat'));
model=d.model; fineS=linspace(0,200,401); mid=0.5*(fineS(1:end-1)+fineS(2:end));
model.sMm=fineS; model.intrinsicCurvaturePerMm=double(mid>=120)/30;
report=struct('coarseStepMm',1,'fineStepMm',0.5,'pushMm',[14 18 20], ...
    'contactDifferenceN',zeros(1,3),'tipPositionDifferenceMm',zeros(1,3), ...
    'fineStationarityInfNmm',zeros(1,3),'fineContactNodeCount',zeros(1,3));
for j=1:3
    coarse=d.equilibria{2,j+2}; model.baseXZ=[0;report.pushMm(j)];
    initial=interp1(coarse.sMm,coarse.thetaRad,fineS(2:end),'pchip');
    fine=solve_planar_energy_rod(model,initial);
    report.contactDifferenceN(j)=norm(fine.contactResultantXZ-coarse.contactResultantXZ);
    report.tipPositionDifferenceMm(j)=norm(fine.pXZ(:,end)-coarse.pXZ(:,end));
    report.fineStationarityInfNmm(j)=fine.stationarityInfNmm;
    report.fineContactNodeCount(j)=sum(vecnorm(fine.contactForceXZ)>1e-5);
    assert(fine.exitflag>0 && fine.stationarityInfNmm<1e-4 && fine.maxPenetrationMm<1e-6);
end
report.scope='Single refinement on three frictionless equilibria; local stationarity, not proof of global optimality or full mesh convergence.';
fid=fopen(fullfile(root,'out','stage2','independent_mesh.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
disp(report);
end
