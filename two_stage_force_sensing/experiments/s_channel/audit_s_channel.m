function audit_s_channel
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'s_channel_investigation');if ~isfolder(folder),mkdir(folder);end
a=load(fullfile(root,'datasets','s_channel_two_contact.mat'));t=a.truth;frames=tsfs.read_input(a.sensorInput);o=tsfs.defaults;
geomRows=[];geometries=cell(size(frames));noise=[];
for k=1:numel(frames)
 fr=frames{k};g=tsfs.geometry(fr,o);geometries{k}=g;
 expected=interp1(t.sMm,t.u(2,:,k)',fr.arcs(:))';noise=[noise fr.u-expected];
 for i=1:numel(g.s)
  n=g.normal(:,g.plane(i));nt=t.planeNormal(:,i);ang=atan2(norm(cross(n,nt)),dot(n,nt));pc=t.contactPoints(:,i,k);
  rr=struct('frame',k,'contact',i,'trueArcMm',t.contactS(i,k),'estimatedArcMm',g.s(i),'arcErrorMm',g.s(i)-t.contactS(i,k), ...
   'normalAngleDeg',ang*180/pi,'offsetShiftMm',g.offset(g.plane(i)), ...
   'gapAtTrueContactMm',n'*(pc-g.point(:,g.plane(i)))-g.offset(g.plane(i)), ...
   'reconstructionShapeRmsMm',sqrt(mean(sum((g.kinematicShape.p-t.p(:,:,k)).^2,1))));geomRows=[geomRows;rr];
 end
end
writetable(struct2table(geomRows),fullfile(folder,'geometry_errors.csv'));
noiseInfo=struct('injectedCurvatureStd',a.scene.curvatureNoiseStd,'actualCurvatureErrorMean',mean(noise),'actualCurvatureErrorStd',std(noise),'actualCurvatureErrorMax',max(abs(noise)), ...
 'likelihoodCurvatureStd',sqrt(frames{1}.CU(1,1)),'environmentPointPriorStdMm',sqrt(frames{1}.CE(1,1,1)), ...
 'environmentNormalPriorStd',sqrt(frames{1}.CE(4,4,1)),'normalPriorApproxDegrees',sqrt(frames{1}.CE(4,4,1))*180/pi);
fid=fopen(fullfile(folder,'noise.json'),'w');fprintf(fid,'%s',jsonencode(noiseInfo,PrettyPrint=true));fclose(fid);
% First-frame 2^3 ablation: correct arc, normal, offset independently. Same
% measured packet and initial force prior throughout; no recurrent history.
rows=[];runs={};fr=frames{1};g=geometries{1};
for mask=0:7
 gg=g;if bitget(mask,1),gg.s=t.contactS(:,1)';end
 if bitget(mask,2),gg.normal=t.planeNormal;end
 if bitget(mask,3),gg.offset=zeros(size(g.offset));end
 for i=1:numel(gg.s),gg.B(:,:,i)=tsfs.basis(gg.normal(:,gg.plane(i)));end
 gg.predictedPoint=tsfs.shape_at(gg.kinematicShape,gg.s);
 sol=tsfs.sqp(fr,gg,[],o);d=tsfs.diagnose(fr,gg,sol,o);sh=sol.evaluation.shape;
 row=struct('mask',mask,'correctArc',logical(bitget(mask,1)),'correctNormal',logical(bitget(mask,2)),'correctOffset',logical(bitget(mask,3)), ...
  'seconds',sol.seconds,'iterations',sol.iterations,'converged',sol.converged,'contactErrorN',norm(sh.force-t.contactForces(:,:,1),'fro'), ...
  'tipErrorN',norm(sh.x(4:6)-t.tipForce(:,1)),'tipX',sh.x(4),'tipY',sh.x(5),'tipZ',sh.x(6), ...
  'fbgRms',d.fbgRms,'penetrationMm',d.maximumPenetrationMm,'projectionN',max(abs(d.projectionN),[],'all'),'flags',strjoin(d.flags,';'));
 rows=[rows;row];sol.evaluation.shape.pieces={};runs{end+1}=sol;disp(row);
end
writetable(struct2table(rows),fullfile(folder,'factorial_ablation.csv'));
save(fullfile(folder,'audit.mat'),'runs','geomRows','geometries','noiseInfo','-v7.3');
% Compare baseline: it learns arc lengths from exactly the same observations.
baseline=estimate_planar_multi_contact(a.sensorInput.frames{1});
baselineSummary=struct('iterations',baseline.iterations,'seconds',baseline.seconds,'s',baseline.state.contactS,'sErrorMm',baseline.state.contactS-t.contactS(:,1)', ...
 'tipErrorN',norm(baseline.state.tipForceXZ-t.tipForce([1 3],1)),'contactErrorN',norm(baseline.state.contactForceXZ-t.contactForces([1 3],:,1),'fro'), ...
 'rank',baseline.jacobianRank,'unknownCount',baseline.unknownCount,'maxScaledResidual',baseline.maxScaledResidual);
% Isolate interpolation from midpoint integration: exact grid curvature, and
% the same sparse interpolation integrated on a finer kinematic grid.
exact=fr;exact.arcs=fr.tube.s;exact.axes=1:3;exact.u=t.u(:,:,1);shExact=tsfs.reconstruct(exact,false);
fine=fr;fine.tube.s=unique([fr.tube.s 0:0.1:fr.tube.s(end)]);fine.tube.uhat=interp1(fr.tube.s,fr.tube.uhat',fine.tube.s,'previous')';shFine=tsfs.reconstruct(fine,false);pFine=interp1(shFine.s,shFine.p',fr.tube.s)';
reconstruction=struct('sparseRmsMm',sqrt(mean(sum((g.kinematicShape.p-t.p(:,:,1)).^2,1))), ...
 'exactNodalCurvatureRmsMm',sqrt(mean(sum((shExact.p-t.p(:,:,1)).^2,1))), ...
 'sparseFineIntegrationRmsMm',sqrt(mean(sum((pFine-t.p(:,:,1)).^2,1))));
% Force observation sensitivity after eliminating base moment with m(L)=0.
gt=g;gt.s=t.contactS(:,1)';gt.normal=t.planeNormal;gt.offset=zeros(size(g.offset));for i=1:2,gt.B(:,:,i)=tsfs.basis(gt.normal(:,i));end
[~,scale,~]=tsfs.initialize(fr,gt,[],o);x=zeros(12,1);x(4:6)=t.tipForce(:,1);x(7)=t.normalForceN(1,1);x(10)=t.normalForceN(2,1);
x(1:3)=cross(t.p(:,end,1)-fr.base(1:3,4),x(4:6));for i=1:2,x(1:3)=x(1:3)+cross(t.contactPoints(:,i,1)-fr.base(1:3,4),t.contactForces(:,i,1));end
sh=equilibrium(fr,gt,x,scale,o);J=sh.Jprediction./scale';E=sh.JtipMoment./scale';F=J(:,4:end)-J(:,1:3)*(E(:,1:3)\E(:,4:end));cols=[1 3 4 7];JP=F(:,cols);sv=svd(JP);sv3=svd(F(:,[1 2 3 4 7]));sensitivity=zeros(4,2);unexplained=zeros(1,2);
for i=1:2
 gp=gt;gm=gt;h=1e-3;gp.s(i)=gp.s(i)+h;gm.s(i)=gm.s(i)-h;
 ep=equilibrium(fr,gp,x,scale,o);em=equilibrium(fr,gm,x,scale,o);d=(ep.prediction(:)-em.prediction(:))/(2*h);sensitivity(:,i)=-JP\d;unexplained(i)=norm(d+JP*sensitivity(:,i));
end
conditioning=struct('forceOrder',{{'tipX','tipZ','normal1','normal2'}},'planarForceSingularValuesPerN',sv,'planarForceCondition',sv(1)/sv(end), ...
 'threeDForceSingularValuesPerN',sv3,'unobservedTipYJacobianNorm',norm(F(:,2)), ...
 'localForceStdNAtDeclaredNoise',sqrt(diag((JP'*JP)\eye(4)))*sqrt(fr.CU(1,1)), ...
 'forceSensitivityToArc_NperMm',sensitivity,'unexplainedCurvatureDerivativeNorm',unexplained);
summary=struct('noise',noiseInfo,'baselineFirstFrame',baselineSummary,'reconstruction',reconstruction,'conditioning',conditioning);
fid=fopen(fullfile(folder,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);
end
function sh=equilibrium(fr,g,x,scale,o)
z=x./scale;
for j=1:8
 sh=tsfs.ivp(fr,g,z,scale,o,true);if norm(sh.tipMoment,inf)<1e-9,break,end
 z(1:3)=z(1:3)-sh.JtipMoment(:,1:3)\sh.tipMoment;
end
assert(norm(sh.tipMoment,inf)<1e-6);
end
