function diagnose_bishop_initialization
[~,paths]=setup_tsfs;folder=fullfile(paths.results,'rotation');a=load(fullfile(folder,'truth_mu_300.mat'));d=load(fullfile(folder,'ours.mat'));fr=tsfs.read_input(a.input);t=a.truth;o=tsfs.defaults;rows=[];
for k=[7 12]
 prev=d.runs{1,2,k-1};warm=d.runs{1,2,k};coldOptions=o;coldOptions.bishopWarmStart=false;cold=tsfs.step(fr{k},prev,coldOptions);
 for j=1:2
  if j==1,out=warm;name='previous-material-state';else,out=cold;name='linear-zero-roll';end
  e=out.mechanics.evaluation;sh=e.shape;scale=out.mechanics.scale;Ju=sh.Jprediction./scale';E=sh.JtipMoment./scale';A=[out.geometry.normal out.geometry.B];
  G=(Ju(:,4:end)-Ju(:,1:3)*(E(:,1:3)\E(:,4:end)))/blkdiag(eye(3),A);sv=svd(G);
  rows=[rows;struct('angleDeg',t.angleDeg(k),'seed',name,'sameHistory',true,'objective',e.objective,'tipErrorN',norm(out.tipForce-t.tipForce(:,k)),'contactErrorN',norm(sum(out.contactForce,2)-t.contactForce(:,k)),'shapeRmsMm',sqrt(mean(sum((sh.p-t.p(:,:,k)).^2,1))),'converged',out.mechanics.converged,'iterations',out.mechanics.iterations,'fbgRms',e.fbgRms,'forceJacobianCondition',sv(1)/sv(end),'flags',strjoin(out.diagnostics.flags,';'))];
 end
end
writetable(struct2table(rows),fullfile(folder,'same_history_initialization.csv'));disp(struct2table(rows));
end
