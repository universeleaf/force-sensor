function test_relaxed_v2
[root,~]=setup_tsfs;a=load(fullfile(root,'results/runs/base_rotation_bishop_20261007/rotation/truth_mu_300.mat'));frames=tsfs.read_input(a.input);o=tsfs.defaults;
for k=[1 7]
 fr=frames{k};g=tsfs.geometry(fr,o);[z,sc,prior]=tsfs.initialize(fr,g,[],o);history=tsfs.reconstruct(fr.previous,false);e=tsfs.evaluate(fr,g,z,sc,prior,history,o,true);b=vecnorm(e.slip)'+0.01;
 [old,Aold]=tsfs.relaxed_constraints(e,g,sc,b,1e-6,false);[v,A]=tsfs.relaxed_constraints(e,g,sc,b,1e-6,true);
 ng=numel(e.h)-numel(g.s);keep=true(size(old));keep(3+ng+1)=false;assert(isequal(v,old(keep))&&isequal(A,Aold(keep,:)));
 fr2=fr;fr2.CU=eye(numel(fr.u))*1e-8;e2=tsfs.evaluate(fr2,g,z,sc,prior,history,o,true);[v2,A2]=tsfs.relaxed_constraints(e2,g,sc,b,1e-6,true);
 assert(isequal(A,A2)&&isequal(v,v2));N=numel(fr.u);assert(norm(e2.J(1:N,:)-e.J(1:N,:)/100,'fro')<1e-9);assert(abs(e2.r(1:N)'*e2.r(1:N)-e.r(1:N)'*e.r(1:N)/1e4)<1e-6);
 y=[z;b];D=zeros(size(A));for j=1:numel(y)
  h=1e-5;yp=y;ym=y;yp(j)=yp(j)+h;ym(j)=ym(j)-h;
  ep=tsfs.evaluate(fr2,g,yp(1:numel(z)),sc,prior,history,o,false);em=tsfs.evaluate(fr2,g,ym(1:numel(z)),sc,prior,history,o,false);
  vp=tsfs.relaxed_constraints(ep,g,sc,yp(numel(z)+1:end),1e-6,true);vm=tsfs.relaxed_constraints(em,g,sc,ym(numel(z)+1:end),1e-6,true);D(:,j)=(vp-vm)/(2*h);
 end
 rel=norm(A-D,'fro')/max(1,norm(D,'fro'));assert(rel<2e-5);fprintf('v2 frame %d: dedup exact, covariance leaves constraint Jacobian unchanged, derivative relative error %.3g\n',k,rel);
end
end
