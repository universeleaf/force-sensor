function run_ablations
[root,paths]=setup_tsfs;o=tsfs.defaults;names={'sliding_clean','s_channel_two_contact','two_contact','three_contact','spatial_sliding'};rows=[];
for j=1:numel(names)
 name=names{j};file=fullfile(root,'datasets',[name '.mat']);
 if isfile(file),a=load(file);t=a.truth;else,a=load(fullfile(root,'datasets',name,'input.mat'));b=load(fullfile(root,'datasets',name,'truth.mat'));t=b.truth;end
 frames=tsfs.read_input(a.sensorInput);fr=frames{1};g=tsfs.geometry(fr,o);
 for mode={'estimated','oracle-geometry'}
  gg=g;
  if strcmp(mode{1},'oracle-geometry')
   gg.s=t.contactS(:,1)';
   if isfield(t,'planeNormal')
    for i=1:numel(gg.s),gg.normal(:,gg.plane(i))=t.planeNormal(:,min(i,size(t.planeNormal,2)));end
   else,gg.normal=fr.normal;end
   pt=fr.point;
   if isfield(t,'planePoint'),for i=1:numel(gg.s),pt(:,gg.plane(i))=t.planePoint(:,min(i,size(t.planePoint,2)));end,end
   for p=1:size(gg.normal,2),gg.offset(p)=gg.normal(:,p)'*(pt(:,p)-gg.point(:,p));end
   for i=1:numel(gg.s),gg.B(:,:,i)=tsfs.basis(gg.normal(:,gg.plane(i)));end
   gg.predictedPoint=tsfs.shape_at(gg.kinematicShape,gg.s);
  end
  sol=tsfs.sqp(fr,gg,[],o);d=tsfs.diagnose(fr,gg,sol,o);
  if isfield(t,'contactForces'),fc=t.contactForces(:,:,1);else,fc=t.contactForce(:,1);end
  row=struct('scenario',name,'mode',mode{1},'seconds',sol.seconds,'iterations',sol.iterations,'converged',sol.converged, ...
   'contactErrorN',norm(sol.evaluation.shape.force-fc,'fro'),'tipErrorN',norm(sol.evaluation.shape.x(4:6)-t.tipForce(:,1)), ...
   'fbgRms',d.fbgRms,'penetrationMm',d.maximumPenetrationMm,'tangency',max(abs(d.tangency)), ...
   'projectionN',max(abs(d.projectionN),[],'all'),'stationarity',sol.stationarity,'reason',sol.reason,'flags',strjoin(d.flags,';'));
  rows=[rows;row];disp(row);
  sol.evaluation.shape.pieces={};save(fullfile(paths.results,[name '_' mode{1} '_ablation.mat']),'sol','gg','d','-v7.3');
 end
end
writetable(struct2table(rows),fullfile(paths.results,'geometry_ablations.csv'));
end
