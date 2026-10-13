function audit_bishop_geometry
[root,paths]=setup_tsfs;src=fullfile(root,'results','runs','base_rotation_bishop_20261007','rotation');a=load(fullfile(src,'truth_mu_300.mat'));b=load(fullfile(paths.results,'audit.mat'));rows=[];sensorRows=[];
for ii=1:2
 k=[7 12];k=k(ii);g=b.cases{ii,1}.geometry;t=a.truth;
 err=g.kinematicShape.p-t.p(:,:,k);r=g.residual;row=struct('angleDeg',t.angleDeg(k),'shapeRmsMm',sqrt(mean(sum(err.^2,1))),'arcErrorMm',g.s-t.contactS(k),'normalErrorDeg',rad2deg(acos(g.normal'*t.planeNormal)),'stage1HalfSquaredCost',.5*sum(r.^2),'whitenedGeometryCost',.5*sum(r(1:2).^2),'environmentPriorCost',.5*sum(r(3:end).^2),'whitenedGapResidual',r(1),'whitenedConditionalTangencyResidual',r(2),'offsetPriorResidual',r(3),'normalPriorResidual1',r(4),'normalPriorResidual2',r(5),'sameGeometryBothSeeds',isequaln(g.s,b.cases{ii,2}.geometry.s)&&isequaln(g.normal,b.cases{ii,2}.geometry.normal));rows=[rows;row];
 for j=1:2
  out=b.cases{ii,j};raw=out.mechanics.evaluation.shape.prediction-a.input.packet.bishopCurvaturePerMm(:,:,k);
  if j==1,name='previous-material';else,name='linear';end
  for i=1:size(raw,2),sensorRows=[sensorRows;struct('angleDeg',t.angleDeg(k),'seed',name,'arcMm',a.input.packet.sFbgMm(i),'w1ResidualPerMm',raw(1,i),'w2ResidualPerMm',raw(2,i))];end
 end
end
writetable(struct2table(rows),fullfile(paths.results,'stage1_blocks.csv'));writetable(struct2table(sensorRows),fullfile(paths.results,'sensor_residuals.csv'));disp(struct2table(rows));
end
