function [violation,l1,eq,ineq]=mode_residual(e,g,modes,y,scale)
% Evaluate original mode constraints on an actual nonlinear IVP result.
C=numel(g.s);ng=numel(e.h)-C;keep=true(ng,1);ns=numel(e.shape.collisionS);
for i=1:C
 if modes(i)>0
  index=find(abs(e.shape.collisionS-g.s(i))<1e-10);keep((g.plane(i)-1)*ns+index)=false;
 end
end
ineq=e.h(1:ng);ineq=ineq(keep);eq=e.c(1:3);pos=6;
for i=1:C
 cc=6+3*i-2:6+3*i;lambda=e.shape.x(cc(1));tau=e.shape.x(cc(2:3));
 if modes(i)>0,eq=[eq;e.gap(i)];ineq=[ineq;-lambda/scale(cc(1))];end
 switch modes(i)
 case 0,eq=[eq;e.shape.x(cc)./scale(cc)];
 case 1
  eq=[eq;e.slip(:,i)];ineq=[ineq;norm(tau)-g.mu(i)*lambda];pos=pos+3;
 case 2
  theta=y(pos+2);amp=y(pos+3);d=[cos(theta);sin(theta)];
  eq=[eq;e.slip(:,i)-amp*d;(tau+g.mu(i)*lambda*d)./scale(cc(2:3))];ineq=[ineq;-amp];pos=pos+3;
 case 3,eq=[eq;tau./scale(cc(2:3))];pos=pos+1;
 end
end
violation=max([abs(eq);ineq;0]);l1=sum(abs(eq))+sum(max(0,ineq));
end
