function report=test_bishop
[root,paths]=setup_tsfs;a=load(fullfile(paths.results,'rotation','truth_mu_300.mat'));frames=tsfs.read_input(a.input);o=tsfs.defaults;
fr=frames{end};g=tsfs.geometry(fr,o);[z,scale,prior]=tsfs.initialize(fr,g,[],o);history=tsfs.reconstruct(fr.previous,false);
tight=o;tight.odeRelativeTolerance=2e-10;tight.odeAbsoluteTolerance=2e-12;
e=tsfs.evaluate(fr,g,z,scale,prior,history,tight,true);D=zeros(size(e.shape.Jprediction));
for k=1:numel(z)
 h=1e-5;zp=z;zm=z;zp(k)=zp(k)+h;zm(k)=zm(k)-h;
 ep=tsfs.ivp(fr,g,zp,scale,tight,false);em=tsfs.ivp(fr,g,zm,scale,tight,false);D(:,k)=(ep.prediction(:)-em.prediction(:))/(2*h);
end
jacError=norm(D-e.shape.Jprediction,'fro')/max(1e-12,norm(D,'fro'));assert(jacError<2e-5);
% Independent exact helical solution: constant material strain exp([u]s).
s=0:0.1:100;u=repmat([0;.02;.01],1,numel(s));[w,psi]=tsfs.material_to_bishop(s,u);
helix=fr;helix.tube=CreatTube(100,s,zeros(3,numel(s)));helix.arcs=s;helix.u=w;helix.base=eye(4);helix.axes=[1 2];
sh=tsfs.reconstruct(helix,false);G=[tsfs.hat(u(:,1)) [0;0;1];zeros(1,4)];exact=zeros(3,numel(s));
for k=1:numel(s),T=expm(G*s(k));exact(:,k)=T(1:3,4);end
helixError=max(vecnorm(sh.p-exact));assert(helixError<2e-5);assert(max(abs(psi-.01*s))<1e-12);
% Rigid world rotation leaves Bishop observations invariant, including base roll gauge.
Q=expm(tsfs.hat([.2;-.3;.4]));fr2=fr;fr2.base(1:3,:)=Q*fr.base(1:3,:);g2=g;g2.normal=Q*g.normal;g2.point=Q*g.point;
for k=1:numel(g.s),g2.B(:,:,k)=Q*g.B(:,:,k);end
x=z.*scale;x(1:3)=Q*x(1:3);x(4:6)=Q*x(4:6);sh2=tsfs.ivp(fr2,g2,x./scale,scale,tight,false);
rotError=max(abs(sh2.prediction-e.shape.prediction),[],'all');assert(rotError<1e-8);
% Evaluate true forces and independent shooting moment with exact contact geometry.
t=a.truth;k=numel(t.angleDeg);gg=g;gg.s=t.contactS(k);gg.normal=t.planeNormal;gg.point=t.planePoint;gg.offset=0;gg.B=reshape(tsfs.basis(t.planeNormal),3,2,1);
xx=[t.shapes{k}.baseMomentNmm;t.tipForce(:,k);[gg.normal gg.B]'*t.contactForce(:,k)];truthFit=tsfs.ivp(fr,gg,xx./scale,scale,tight,false);
conversionError=max(abs(truthFit.prediction-fr.u),[],'all');positionError=max(vecnorm(truthFit.p-t.p(:,:,k)));
assert(conversionError<1e-9&&positionError<2e-5);
report=struct('passed',true,'relativeObservationJacobianError',jacError,'analyticHelixPositionMaxMm',helixError,'worldRotationObservationErrorPerMm',rotError,'independentTruthPredictionMaxPerMm',conversionError,'independentTruthPositionMaxMm',positionError,'sensorCount',numel(fr.arcs));
atomic_write_artifact(fullfile(paths.results,'bishop_tests.json'),'json',report);disp(report);
end
