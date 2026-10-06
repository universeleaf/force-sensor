function report=test_spatial_derivatives
[root,paths]=setup_tsfs;a=load(fullfile(root,'datasets','spatial_sliding','input.mat'));frames=tsfs.read_input(a.sensorInput);fr=frames{2};o=tsfs.defaults;g=tsfs.geometry(fr,o);[z,scale,prior]=tsfs.initialize(fr,g,[],o);history=tsfs.reconstruct(fr.previous,false);
e=tsfs.evaluate(fr,g,z,scale,prior,history,o,true);J=[e.J;e.Jc;e.Jh];D=zeros(size(J));tight=o;tight.odeRelativeTolerance=2e-10;tight.odeAbsoluteTolerance=2e-12;
for k=1:numel(z)
 h=1e-5;zp=z;zm=z;zp(k)=zp(k)+h;zm(k)=zm(k)-h;
 ep=tsfs.evaluate(fr,g,zp,scale,prior,history,tight,false);em=tsfs.evaluate(fr,g,zm,scale,prior,history,tight,false);
 D(:,k)=([ep.r;ep.c;ep.h]-[em.r;em.c;em.h])/(2*h);
end
sc=max(1,max(abs(D),[],2));error=norm((D-J)./sc,'fro')/max(1,norm(D./sc,'fro'));assert(error<5e-4);
% Rotate a genuinely spatial loaded rod, all planes, tangent bases, and loads.
Q=expm(tsfs.hat([0.2;0.3;-0.4]));fr2=fr;fr2.base(1:3,:)=Q*fr.base(1:3,:);g2=g;g2.normal=Q*g.normal;g2.point=Q*g.point;
for i=1:numel(g.s),g2.B(:,:,i)=Q*g.B(:,:,i);end
x=z.*scale;x(1:3)=Q*x(1:3);x(4:6)=Q*x(4:6);rot=tsfs.ivp(fr2,g2,x./scale,scale,o,false);rotationError=max(abs(rot.p-Q*e.shape.p),[],'all');assert(rotationError<2e-5);
report=struct('passed',true,'relativeJacobianError',error,'rigidRotationPositionMm',rotationError,'contacts',numel(g.s),'nonplanarContactForceN',norm(e.shape.force(2,:)));
fid=fopen(fullfile(paths.results,'spatial_derivative_tests.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
