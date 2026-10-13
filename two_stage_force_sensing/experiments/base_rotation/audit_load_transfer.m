function audit_load_transfer
% Offline nonlinear equilibria: fixed geometry, diagnostic loads only.
[root,~]=setup_tsfs;o=tsfs.defaults;o.collisionStepMm=0.1;
pub=fullfile(root,'results','published','base_rotation_load_transfer_20261007');if ~isfolder(pub),mkdir(pub);end
D=jsondecode(fileread(fullfile(root,'results','published','base_rotation_direction_audit_20261007','matrices.json')));
profiles=readtable(fullfile(root,'results','published','base_rotation_direction_audit_20261007','direction_sensitivity.csv'));
rows=[];geometry={};sensorRows=[];shapeRows=[];pairs=[];
for ic=1:3
 if ic==1,tag='planar';k=1;a=load(fullfile(root,'datasets','sliding_clean.mat'));input=a.sensorInput;t=a.truth;angle=0;
 else,tag='375deg';a=load(fullfile(root,'results','runs','base_rotation_relaxed_v2_375deg_20261007','rotation','truth_mu_300.mat'));input=a.input;t=a.truth;if ic==2,k=10;else,k=13;end;angle=t.angleDeg(k);end
 caseName=sprintf('%s_%g',tag,angle);frames=tsfs.read_input(input);fr=frames{k};fr.observationType='bishop-curvature';fr.axes=[1 2];
 n=t.planeNormal;if size(n,2)>1,n=n(:,k);end;n=n/norm(n);B=null(n');A=[n B];g=struct('s',t.contactS(k),'normal',n,'plane',1,'B',B);fc=t.contactForce(:,k);ft=t.tipForce(:,k);
 if ic==1,pc=t.contactPoint(:,k);else,pc=t.shapes{k}.contactPoints;end
 m=cross(t.p(:,end,k)-t.p(:,1,k),ft)+cross(pc-t.p(:,1,k),fc);scale=[200;200;200;ones(6,1)];x0=[m;ft;A'*fc];
 base=tsfs.ivp(fr,g,x0./scale,scale,o,true);pc=base.contactP;
 if k>1,prev=t.shapes{k-1};old=interp1(prev.collisionS,prev.collisionP',g.s,'pchip')';else,prev=tsfs.reconstruct(frames{k}.previous,false);old=interp1(prev.s,prev.p',g.s,'pchip')';end
 if ic==2,mode='sticking';old=pc;else,mode='sliding';end
 chord=base.p(:,end)-pc;unit=chord/norm(chord);tail=base.collisionP(:,base.collisionS>=g.s)-pc;sag=max(vecnorm(tail-unit*(unit'*tail)));
 dd=D(find(arrayfun(@(r)strcmp(r.caseName,tag)&&r.frame==k,D),1));G=dd.G;weak=dd.weakForceDirection(:);if dot(weak(1:3),unit)<0,weak=-weak;end
 geometry{end+1}=struct('caseName',caseName,'contactArcMm',g.s,'lengthMm',fr.tube.s(end),'fbgArcsMm',fr.arcs,'sensorsAfterContact',sum(fr.arcs>g.s),'sensorCount',numel(fr.arcs), ...
 'contactToTipChordMm',norm(chord),'tailMaxDistanceFromChordMm',sag,'chordUnit',unit,'weakTipChordCosine',dot(weak(1:3),unit)/norm(weak(1:3)), ...
 'weakContactOppositeChordCosine',-dot(weak(4:6),unit)/norm(weak(4:6)),'mode',mode);
 saved={};names={};
 scenarios={'tip_Y','contact_Y','exchange_Y','tip_chord','contact_chord','exchange_chord'};
 for j=1:numel(scenarios)
  name=scenarios{j};if endsWith(name,'Y'),dir=[0;1;0];else,dir=unit;end
  xt=x0;if startsWith(name,'tip'),xt(4:6)=ft+0.5*dir;elseif startsWith(name,'contact'),xt(7:9)=A'*(fc+0.5*dir);else,xt(4:6)=ft+0.5*dir;xt(7:9)=A'*(fc-0.5*dir);end
  [sh,flag]=solveState(xt,false);record(name,sh,flag,0.5,false);saved{end+1}=sh;names{end+1}=name;
 end
 for axisName={'Y','chord'}
  a1=saved{find(strcmp(names,['tip_' axisName{1}]))};a2=saved{find(strcmp(names,['contact_' axisName{1}]))};dif=a1.collisionP-a2.collisionP;du=a1.prediction-a2.prediction;
  pairs=[pairs;struct('caseName',caseName,'direction',axisName{1},'transferredN',0.5,'shapeDifferenceRmsMm',sqrt(mean(sum(dif.^2,1))),'shapeDifferenceMaxMm',max(vecnorm(dif)), ...
   'curvatureDifferenceRmsPerMm',rms(du(:)),'curvatureDifferenceWhitenedRms',rms(du(:))/1e-4)];
 end
 pr=profiles(strcmp(profiles.caseName,tag)&profiles.frame==k&strcmp(profiles.axis,'Y'),:);direction=[pr.compensatingTipX;pr.compensatingTipY;pr.compensatingTipZ;pr.compensatingContactX;pr.compensatingContactY;pr.compensatingContactZ];
 for delta=[-0.5 -0.1 -0.01 0.01 0.1 0.5]
  xt=x0;xt(4:6)=ft+delta*direction(1:3);xt(7:9)=A'*(fc+delta*direction(4:6));[sh,flag]=solveState(xt,true);record(sprintf('branch_tipY_%g',delta),sh,flag,delta,true);
 end
 % Partition the physical force sensitivity across sensor regions.
 for ax=1:3
  labels='XYZ';for reg={'before','after'}
   if strcmp(reg{1},'before'),ix=find(fr.arcs<g.s);else,ix=find(fr.arcs>g.s);end;ix=reshape([2*ix-1;2*ix],1,[]);u=zeros(6,1);u(ax)=1;u(ax+3)=-1;
   sensorRows=[sensorRows;struct('caseName',caseName,'axis',labels(ax),'region',reg{1},'nodes',numel(ix)/2,'tipColumnRms',rms(G(ix,ax)),'contactColumnRms',rms(G(ix,ax+3)),'exchangeRms',rms(G(ix,:)*u))];
  end
 end
 fprintf('Completed nonlinear load transfers for %s.\n',caseName);
end
writetable(struct2table(rows),fullfile(pub,'nonlinear_candidates.csv'));writetable(struct2table(pairs),fullfile(pub,'tip_vs_contact_pairs.csv'));writetable(struct2table(sensorRows),fullfile(pub,'sensor_regions.csv'));writetable(struct2table(shapeRows),fullfile(pub,'shape_profiles.csv'));atomic_write_artifact(fullfile(pub,'geometry.json'),'json',geometry);
 function [sh,flag]=solveState(xx,branch)
  if branch,ids=[1:3 7:9];else,ids=1:3;end
  opts=optimoptions('lsqnonlin','Display','off','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-14,'OptimalityTolerance',1e-12,'StepTolerance',1e-13,'MaxIterations',100,'MaxFunctionEvaluations',300);
  [v,~,~,flag]=lsqnonlin(@fun,xx(ids)./scale(ids),[],[],opts);xx(ids)=v.*scale(ids);sh=tsfs.ivp(fr,g,xx./scale,scale,o,true);
  function [r,J]=fun(v)
   xxx=xx;xxx(ids)=v.*scale(ids);q=tsfs.ivp(fr,g,xxx./scale,scale,o,true);r=q.tipMoment/200;J=q.JtipMoment(:,ids)/200;
   if branch
    P=reshape(q.JcontactP(:,1,:),3,[]);if strcmp(mode,'sticking'),r=[r;q.contactP-pc];J=[J;P(:,ids)];
    else
     slip=B'*(q.contactP-old);normSlip=norm(slip);dir=slip/max(normSlip,1e-12);lam=q.x(7);tau=q.x(8:9);df=zeros(2,9);df(:,7)=fr.mu*dir;df(:,8:9)=eye(2);df=df.*scale';
     ds=fr.mu*lam/max(normSlip,1e-12)*(eye(2)-dir*dir')*B'*P;
     r=[r;n'*(q.contactP-pc);tau+fr.mu*lam*dir];J=[J;n'*P(:,ids);df(:,ids)+ds(:,ids)];
    end
   end
  end
 end
 function record(name,q,flag,delta,branch)
  diff=q.collisionP-base.collisionP;dw=q.prediction-base.prediction;force=A*q.x(7:9);slip=B'*(q.contactP-old);cone=norm(q.x(8:9))-fr.mu*q.x(7);gap=n'*(q.contactP-pc);penetration=max(0,-min(n'*(q.collisionP-pc)));tan=abs(n'*q.contactT);
  if strcmp(mode,'sticking'),law=norm(q.contactP-pc,inf);else,law=max(abs([gap;q.x(8:9)+fr.mu*q.x(7)*slip/max(norm(slip),1e-12)]));end
  sampledPenetration=penetration;[~,minLocalGap]=fminbnd(@(ss)surfaceGap(q,ss),max(q.s(1),g.s-0.5),min(q.s(end),g.s+0.5),optimset('TolX',1e-10,'Display','off'));penetration=max(penetration,-minLocalGap);
  physical=tan<=o.tangencyWarning&&norm(q.tipMoment,inf)<1e-5&&abs(gap)<1e-6&&penetration<1e-6&&cone<1e-6&&q.x(7)>=0&&law<1e-6;
  pre=fr.arcs<g.s;post=fr.arcs>g.s;dpre=dw(:,pre);dpost=dw(:,post);
  rows=[rows;struct('caseName',caseName,'scenario',name,'branchEnforced',branch,'parameterDeltaN',delta,'exitFlag',flag,'physicalChecksPass',physical,'tipChangeN',norm(q.x(4:6)-ft),'contactChangeN',norm(force-fc), ...
   'tipFx',q.x(4),'tipFy',q.x(5),'tipFz',q.x(6),'contactFx',force(1),'contactFy',force(2),'contactFz',force(3), ...
   'shapeRmsMm',sqrt(mean(sum(diff.^2,1))),'shapeMaxMm',max(vecnorm(diff)),'shapeTipMm',norm(diff(:,end)), ...
   'curvatureRmsPerMm',rms(dw(:)),'curvatureMaxPerMm',max(abs(dw),[],'all'),'fbgRmsSigma1e4',rms(dw(:))/1e-4,'fbgRmsSigma1e6',rms(dw(:))/1e-6, ...
   'beforeContactCurvatureRms',rms(dpre(:)),'afterContactCurvatureRms',rms(dpost(:)), ...
   'terminalMomentNmm',norm(q.tipMoment,inf),'gapMm',gap,'sampledPenetrationMm',sampledPenetration,'penetrationMm',penetration,'tangency',tan,'coneExcessN',max(0,cone),'contactLawResidual',law)];
  nn=numel(q.collisionS);block=struct('caseName',repmat({caseName},nn,1),'scenario',repmat({name},nn,1),'sMm',num2cell(q.collisionS(:)), ...
   'baseX',num2cell(base.collisionP(1,:)'),'baseY',num2cell(base.collisionP(2,:)'),'baseZ',num2cell(base.collisionP(3,:)'), ...
   'newX',num2cell(q.collisionP(1,:)'),'newY',num2cell(q.collisionP(2,:)'),'newZ',num2cell(q.collisionP(3,:)'),'differenceMm',num2cell(vecnorm(diff)'));shapeRows=[shapeRows;block];
 end
 function gap=surfaceGap(q,ss)
  ix=find(q.edges<=ss,1,'last');ix=min(ix,numel(q.pieces));yy=deval(q.pieces{ix},ss);gap=n'*(yy(1:3)-pc);
 end

end
