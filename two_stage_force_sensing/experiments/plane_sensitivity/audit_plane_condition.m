function audit_plane_condition
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'plane_sensitivity');a=load(fullfile(folder,'sensitivity.mat'));out=cell(2,1);
for k=1:2
 m=a.models{k,1};rec=a.records{k,1};q=rec.q;theta=zeros(2*m.C+1,1);[r,~,~]=evaluate(m,q,theta);J=zeros(numel(r),numel(q));
 for j=1:numel(q),h=1e-5;qp=q;qm=q;qp(j)=qp(j)+h;qm(j)=qm(j)-h;J(:,j)=(evaluate(m,qp,theta)-evaluate(m,qm,theta))/(2*h);end
 idx=[2 4:numel(q)];outidx=[1 3];[~,S,V]=svd(J);sj=diag(S);sin=svd(J(idx,idx));sout=svd(J(outidx,outidx));
 out{k}=struct('scenario',rec.scenario,'J',J,'inPlaneCondition',sin(1)/sin(end),'outOfPlaneCondition',sout(1)/sout(end),'smallestRightMode',V(:,end),'inPlaneSingularValues',sin,'outOfPlaneSingularValues',sout);
end
fid=fopen(fullfile(folder,'condition_blocks.json'),'w');fprintf(fid,'%s',jsonencode(out,PrettyPrint=true));fclose(fid);disp(out);
end
function [r,sh,y]=evaluate(m,q,theta)
C=m.C;n=m.normal;point=m.point;
for i=1:C,n(:,i)=cos(theta(C+i))*m.normal(:,i)+sin(theta(C+i))*[0;1;0];point(:,i)=point(:,i)+theta(i)*m.normal(:,i);end
s=q(3+C+1:end)'*m.L;fn=q(4:3+C)'*m.Fref;B=zeros(3,2,C);x=zeros(6+3*C,1);x(1:3)=q(1:3)*m.Mref;x(4:6)=m.tip+[0;theta(end);0];
for i=1:C
 B(:,:,i)=tsfs.basis(n(:,i));ft=zeros(3,1);
 if m.mu(i)>0,v=m.slide(:,i)-n(:,i)*(n(:,i)'*m.slide(:,i));ft=-m.mu(i)*fn(i)*v/norm(v);end
 x(6+3*i-2:6+3*i)=[fn(i);B(:,:,i)'*ft];
end
g=struct('s',s,'normal',n,'point',point,'offset',zeros(1,C),'plane',1:C,'B',B,'mu',m.mu);
scale=[repmat(m.Mref,3,1);repmat(m.Fref,3+3*C,1)];sh=tsfs.ivp(m.fr,g,x./scale,scale,m.o,false);
gap=sum(n.*(sh.contactP-point),1);tangent=sum(n.*sh.contactT,1);
r=[sh.tipMoment/m.Mref;gap(:)/m.L;tangent(:)];
y=[s(:);sh.contactP(:);fn(:);sh.force(:);sh.prediction(:)];
end
function [q,r,sh,y,flag]=solve(m,q0,theta)
C=m.C;ss=q0(4+C:end);edges=[0;(ss(1:end-1)+ss(2:end))/2;1];lb=[-Inf(3,1);zeros(C,1);edges(1:end-1)+1e-6];ub=[Inf(3+C,1);edges(2:end)-1e-6];
opts=optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-15,'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',80,'MaxFunctionEvaluations',1500,'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-5);
[q,~,r,flag]=lsqnonlin(@(v)evaluate(m,v,theta),q0,lb,ub,opts);[r,sh,y]=evaluate(m,q,theta);
end
function pen=penetration(m,sh,theta)
pen=0;for i=1:m.C,n=cos(theta(m.C+i))*m.normal(:,i)+sin(theta(m.C+i))*[0;1;0];point=m.point(:,i)+theta(i)*m.normal(:,i);pen=max(pen,-min(n'*(sh.collisionP-point)));end
end
