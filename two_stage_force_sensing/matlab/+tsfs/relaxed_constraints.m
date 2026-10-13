function [v,J,meta]=relaxed_constraints(e,g,scale,beta,epsilon,deduplicateGap)
% Smooth circular Coulomb cones and relaxed nonnegative duality gap.
% Units normalized by F0=1 N, G0=V0=1 mm. beta is slip-norm epigraph.
% No division by normal force/slip; valid also at the zero-force cone apex.
if nargin<6,deduplicateGap=false;end
C=numel(g.s);nx=numel(scale);nv=nx+C;sh=e.shape;
ng=numel(e.h)-C;h=e.h(1:ng);Dh=[e.Jh(1:ng,:) zeros(ng,C)];
gapProduct=zeros(C,1);frictionGap=zeros(C,1);
for i=1:C
 cols=6+3*i-2:6+3*i;lambda=sh.x(cols(1));tau=sh.x(cols(2:3));R=g.mu(i)*lambda;
 dl=zeros(1,nv);dl(cols(1))=scale(cols(1));dt=zeros(2,nv);dt(:,cols(2:3))=diag(scale(cols(2:3)));
 dg=[g.normal(:,g.plane(i))'*reshape(sh.JcontactP(:,i,:),3,nx) zeros(1,C)];
 dv=[g.B(:,:,i)'*reshape(sh.JcontactP(:,i,:),3,nx) zeros(2,C)];
 if isfield(e,'relaxedHasHistory')&&~e.relaxedHasHistory,dv(:)=0;end
 db=zeros(1,nv);db(nx+i)=1;slip=e.slip(:,i);gap=e.gap(i);b=beta(i);
 gapProduct(i)=lambda*gap;frictionGap(i)=R*b+tau'*slip;
 if ~deduplicateGap,h=[h;-gap];Dh=[Dh;-dg];else,assert(any(abs(sh.collisionS-g.s(i))<1e-10),'Contact gap must already be sampled.');end
 h=[h;lambda*gap-epsilon;tau'*tau-R^2;slip'*slip-b^2;frictionGap(i)-epsilon];
 Dh=[Dh;gap*dl+lambda*dg;2*tau'*dt-2*R*g.mu(i)*dl;2*slip'*dv-2*b*db;g.mu(i)*b*dl+R*db+slip'*dt+tau'*dv];
end
ceq=e.c(1:3);Dceq=[e.Jc(1:3,:) zeros(3,C)];
v=[ceq;h];J=[Dceq;Dh];meta=struct('normalProduct',gapProduct,'frictionGap',frictionGap,'violation',max([abs(ceq);h;0]));
end
