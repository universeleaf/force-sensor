function test_relaxed
[root,~]=setup_tsfs;a=load(fullfile(root,'results/runs/base_rotation_bishop_20261007/rotation/truth_mu_300.mat'));frames=tsfs.read_input(a.input);o=tsfs.defaults;
for k=[1 7 12]
 fr=frames{k};g=tsfs.geometry(fr,o);[z,sc,prior]=tsfs.initialize(fr,g,[],o);history=tsfs.reconstruct(fr.previous,false);
 e=tsfs.evaluate(fr,g,z,sc,prior,history,o,true);b=vecnorm(e.slip)'+0.01;[v,J]=tsfs.relaxed_constraints(e,g,sc,b,1e-6);y=[z;b];D=zeros(size(J));
 for j=1:numel(y)
  h=1e-5;yp=y;ym=y;yp(j)=yp(j)+h;ym(j)=ym(j)-h;
  ep=tsfs.evaluate(fr,g,yp(1:numel(z)),sc,prior,history,o,false);em=tsfs.evaluate(fr,g,ym(1:numel(z)),sc,prior,history,o,false);
  vp=tsfs.relaxed_constraints(ep,g,sc,yp(numel(z)+1:end),1e-6);vm=tsfs.relaxed_constraints(em,g,sc,ym(numel(z)+1:end),1e-6);D(:,j)=(vp-vm)/(2*h);
 end
 relative=norm(J-D,'fro')/max(1,norm(D,'fro'));fprintf('Relaxed derivatives frame %d relative error %.4g\n',k,relative);assert(relative<2e-5);
end
% Cone apex allows nonzero slip with zero force and beta >= ||slip||.
e.shape.x(7:9)=0;e.gap(:)=0.1;e.slip(:)=[0.3;0.4];[v,~,m]=tsfs.relaxed_constraints(e,g,sc,0.5,1e-6);assert(m.normalProduct==0&&m.frictionGap==0);assert(all(v(end-4:end)<=1e-12));
fprintf('Circular-cone apex and derivative checks passed.\n');
end
