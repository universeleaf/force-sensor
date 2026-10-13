function [A,b,Aeq,beq,lb,ub,meta]=polygon_qp_constraints(fr,g,model,codes,scale,radius)
% Inscribed regular octagon. Codes 0=separation,1=stick,2:9=edge,10:17=vertex.
% Maximum dissipation: -v belongs to the normal cone of the force polygon.
z0=model.z;e=model.e;nx=numel(z0);C=numel(codes);ny=nx+2*sum(codes>=10)+sum(codes>=2&codes<10);A=[];b=[];Aeq=[];beq=[];lb=-inf(ny,1);ub=inf(ny,1);lb(1:nx)=z0-radius;ub(1:nx)=z0+radius;
E=e.Jc(1:3,:);Aeq=[E zeros(3,ny-nx)];beq=E*z0-e.c(1:3);ng=numel(e.h)-C;keep=true(ng,1);ns=numel(e.shape.collisionS);
for i=1:C,if codes(i)>0,index=find(abs(e.shape.collisionS-g.s(i))<1e-10);keep((g.plane(i)-1)*ns+index)=false;end,end
A=[e.Jh(find(keep),:) zeros(sum(keep),ny-nx)];b=e.Jh(find(keep),:)*z0-e.h(find(keep));
H=[cos((0:7)'*pi/4+pi/8) sin((0:7)'*pi/4+pi/8)];rho=cos(pi/8);pos=nx;slots=cell(1,C);
for i=1:C
 cc=6+3*i-2:6+3*i;L=zeros(1,ny);L(cc(1))=scale(cc(1));T=zeros(2,ny);T(:,cc(2:3))=diag(scale(cc(2:3)));JP=reshape(e.shape.JcontactP(:,i,:),3,nx);Dg=g.normal(:,g.plane(i))'*JP;Dv=g.B(:,:,i)'*JP;v0=e.slip(:,i)-Dv*z0;
 if codes(i)==0
  Q=zeros(3,ny);Q(:,cc)=eye(3);Aeq=[Aeq;Q];beq=[beq;zeros(3,1)];continue
 end
 lb(cc(1))=max(0,lb(cc(1)));Aeq=[Aeq;Dg zeros(1,ny-nx)];beq=[beq;Dg*z0-e.gap(i)];A=[A;H*T-g.mu(i)*rho*ones(8,1)*L];b=[b;zeros(8,1)];
 if codes(i)==1
  Aeq=[Aeq;Dv zeros(2,ny-nx)];beq=[beq;-v0];slots{i}=[];
 elseif codes(i)<10
  f=codes(i)-1;slots{i}=pos+1;pos=pos+1;lb(pos)=0;V=[Dv zeros(2,ny-nx)];V(:,pos)=H(f,:)';Aeq=[Aeq;H(f,:)*T-g.mu(i)*rho*L;V];beq=[beq;0;-v0];
 else
  vertex=codes(i)-10;u=[cos(vertex*pi/4);sin(vertex*pi/4)];adj=[mod(vertex-1,8)+1 vertex+1];slots{i}=pos+(1:2);pos=pos+2;lb(slots{i})=0;V=[Dv zeros(2,ny-nx)];V(:,slots{i})=H(adj,:)';Aeq=[Aeq;T-g.mu(i)*u*L;V];beq=[beq;zeros(2,1);-v0];
 end
end
meta=struct('slots',{slots},'H',H,'rho',rho,'nx',nx,'ny',ny);
end
