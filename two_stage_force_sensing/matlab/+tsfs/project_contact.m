function [r,J,info]=project_contact(fn,tau,gap,slip,mu,rn,rt)
% Columns of J: fn,tau(1:2),gap,slip(1:2). Exact Coulomb disk.
w=fn-rn*gap;a=max(0,w);Da=double(w>0)*[1 0 0 -rn 0 0];q=tau-rt*slip;R=mu*a;nq=norm(q);
Dq=[zeros(2,1) eye(2) zeros(2,1) -rt*eye(2)];
if nq<R
 p=q;Dp=Dq;branch='stick';
elseif nq>0
 e=q/nq;p=R*e;Dp=(R/nq)*(eye(2)-e*e')*Dq+e*(mu*Da);branch='slide';
else
 % At the disk apex choose zero derivative in q and radius. This is a
 % valid limiting branch with q/r unbounded. The inactive normal branch is chosen at w=0; penetration activates it.
 p=zeros(2,1);Dp=zeros(2,6);branch='apex';
end
r=[fn-a;tau-p];J=[1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0]-[Da;Dp];
info=struct('a',a,'radius',R,'branch',branch,'work',tau'*slip,'coneExcess',max(0,norm(tau)-mu*fn));
end
