function probe_incline
[root,paths]=setup_tsfs;seed=load(fullfile(root,'datasets','sliding_clean.mat'));tube=seed.sensorInput.tube;s=0:200;R=120/(pi/2);model=struct('sMm',s,'intrinsicCurvaturePerMm',double((s(1:end-1)+s(2:end))/2>=80)/R,'EINmm2',tube.kb,'baseXZ',[0;0],'baseAngleRad',0,'tipForceXZ',[1;0]);
free=solve_planar_contact_shooting(model);n=[1;-1]/sqrt(2);clearance=max(-n'*free.pXZ);start=ceil(clearance+5);fprintf('FREE: contact onset distance %.6f mm, start %.1f mm, natural radius %.6f mm\n',clearance,start,R);
model.planeNormalXZ=n;model.planePointXZ=[0;sqrt(2)*start];
for d=[start clearance-1 60 50 40]
 model.baseXZ=[0;sqrt(2)*(start-d)];model.tangentialNormalRatio=-0.3;q=solve_planar_contact_shooting(model);fprintf('PROBE d %.3f: %s s %.5f fn %.5f tip=[%.4f %.4f] penetration %.2g\n',d,q.mode,q.contactArcLengthMm,q.normalReactionN,q.pXZ(:,end),q.maxPenetrationMm);
end
save(fullfile(paths.results,'probe.mat'),'free','start','clearance','model');
end
