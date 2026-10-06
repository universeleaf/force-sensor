function report=test_tsfs
[root,paths]=setup_tsfs;o=tsfs.defaults;checks={};
% Exact cone, no zero-radius division, dissipative sign, and switching cases.
[r,~,v]=tsfs.project_contact(2,[-0.6;0],0,[0.1;0],0.3,1,1);assert(norm(r)<1e-12&&v.work<0);checks{end+1}='sliding maximum dissipation';
[r,J]=tsfs.project_contact(0,[0;0],0,[0;0],0.3,1,1);assert(all(isfinite(J),'all')&&norm(r)==0);checks{end+1}='zero force apex';
[r,~,~]=tsfs.project_contact(0,[1;2],1,[0;0],0.3,1,1);assert(norm(r(2:3)-[1;2])<1e-12);checks{end+1}='separated contact has zero force';
[r,~,~]=tsfs.project_contact(2,[0;0],0,[3;4],0,1,1);assert(norm(r)==0);checks{end+1}='zero friction';
y=[2;0.8;0.2;0.01;0.1;-0.1];[~,J]=projection(y);D=zeros(3,6);
for k=1:6,h=1e-6;yp=y;ym=y;yp(k)=yp(k)+h;ym(k)=ym(k)-h;D(:,k)=(projection(yp)-projection(ym))/(2*h);end
projectionError=norm(D-J,'fro')/max(1,norm(D,'fro'));assert(projectionError<1e-7);checks{end+1}='projection generalized derivative away from switches';
% Full lifted variational derivatives against independently rerun IVPs.
a=load(fullfile(root,'datasets','sliding_clean.mat'));frames=tsfs.read_input(a.sensorInput);fr=frames{1};g=tsfs.geometry(fr,o);[z,scale,prior]=tsfs.initialize(fr,g,[],o);history=tsfs.reconstruct(fr.previous,false);
e=tsfs.evaluate(fr,g,z,scale,prior,history,o,true);J=[e.J;e.Jc;e.Jh];D=zeros(size(J));tight=o;tight.odeRelativeTolerance=2e-10;tight.odeAbsoluteTolerance=2e-12;
for k=1:numel(z)
 h=1e-5;zp=z;zm=z;zp(k)=zp(k)+h;zm(k)=zm(k)-h;
 ep=tsfs.evaluate(fr,g,zp,scale,prior,history,tight,false);em=tsfs.evaluate(fr,g,zm,scale,prior,history,tight,false);
 D(:,k)=([ep.r;ep.c;ep.h]-[em.r;em.c;em.h])/(2*h);
end
rowScale=max(1,max(abs(D),[],2));derivativeError=norm((D-J)./rowScale,'fro')/max(1,norm(D./rowScale,'fro'));assert(derivativeError<5e-4);checks{end+1}='augmented IVP and constraint derivatives';
ref=tsfs.evaluate(fr,g,z,scale,prior,history,tight,false);odeError=max(abs(ref.shape.p-e.shape.p),[],'all');assert(odeError<2e-5);checks{end+1}='ODE tolerance refinement';
% A rigid rotation changes world forces, not curvature observations.
v=[0.3;-0.7;0.2];Q=expm(tsfs.hat(v));shift=[3;-5;2];fr2=fr;fr2.base(1:3,:)=Q*fr.base(1:3,:);fr2.base(1:3,4)=fr2.base(1:3,4)+shift;
g2=g;g2.normal=Q*g.normal;g2.point=Q*g.point+shift;
for k=1:numel(g.s),g2.B(:,:,k)=Q*g.B(:,:,k);end
x=z.*scale;x(1:3)=Q*x(1:3);x(4:6)=Q*x(4:6);rot=tsfs.ivp(fr2,g2,x./scale,scale,o,false);
rotationError=max(abs(rot.p-(Q*e.shape.p+shift)),[],'all');assert(rotationError<2e-5);assert(max(abs(rot.prediction-e.shape.prediction),[],'all')<1e-8);checks{end+1}='3D mechanics coordinate invariance';
% No-contact/no-surface path and immutable, reporting-only warnings.
empty=fr;empty.point=zeros(3,0);empty.normal=zeros(3,0);empty.mu=[];empty.CE=zeros(6,6,0);empty.previous=[];
u0=interp1(empty.tube.s,empty.tube.uhat',empty.arcs,'previous')';empty.u=u0(empty.axes,:);
free=tsfs.step(empty,[],o);assert(isempty(free.contactForce)&&free.mechanics.converged);checks{end+1}='no-contact and no-surface state';
before=free;warnOptions=o;warnOptions.fbgRmsWarning=-1;dd=tsfs.diagnose(empty,free.geometry,free.mechanics,warnOptions);
assert(isequaln(before,free)&&ismember('fbg-mismatch',dd.flags));checks{end+1}='diagnostics cannot mutate state';
invalid=empty;invalid.CU=-eye(size(empty.CU));seq=tsfs.estimate({empty,invalid,empty},o);assert(numel(seq)==3&&seq{1}.valid&&~seq{2}.valid&&seq{3}.valid);checks{end+1}='numerical failure retains output and continues sequence';
% Creation/separation use the exact projection, including a penetrated zero-force start.
[rr,JJ]=tsfs.project_contact(0,[0;0],-0.01,[0;0],0.3,1,1);assert(abs(rr(1)+0.01)<1e-12&&JJ(1,4)==1);
[rr,~]=tsfs.project_contact(0,[0;0],0.01,[0;0],0.3,1,1);assert(norm(rr)==0);checks{end+1}='contact activation and separation branches';
report=struct('passed',true,'checks',{checks},'projectionDerivativeError',projectionError,'mechanicsDerivativeError',derivativeError,'odeRefinementPositionMm',odeError,'rigidRotationPositionMm',rotationError);
save(fullfile(paths.results,'unit_tests.mat'),'report');fid=fopen(fullfile(paths.results,'unit_tests.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
function [r,J]=projection(y)
[r,J]=tsfs.project_contact(y(1),y(2:3),y(4),y(5:6),0.3,1,1);
end
