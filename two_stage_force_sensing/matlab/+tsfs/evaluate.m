function e=evaluate(fr,g,z,scale,prior,history,o,derivatives)
sh=tsfs.ivp(fr,g,z,scale,o,derivatives);x=sh.x;nx=numel(z);C=numel(g.s);P=size(g.normal,2);
L=chol(fr.CU,'lower');ru=L\(sh.prediction(:)-fr.u(:));Ju=L\sh.Jprediction;
r=[ru;(x(4:6)-prior.tip)/prior.tipStd];J=[Ju;zeros(3,nx)];J(end-2:end,4:6)=diag(scale(4:6))/prior.tipStd;
for i=1:C
 cols=6+3*i-2:6+3*i;A=[g.normal(:,g.plane(i)) g.B(:,:,i)];r=[r;(sh.force(:,i)-prior.force(:,i))/prior.forceStd];JJ=zeros(3,nx);JJ(:,cols)=A*diag(scale(cols))/prior.forceStd;J=[J;JJ];
end
momentScale=max(scale(1:3));c=sh.tipMoment/momentScale;Jc=sh.JtipMoment/momentScale;
gap=zeros(1,C);slip=zeros(2,C);projection=zeros(3,C);Jproj=zeros(3*C,nx);work=zeros(1,C);branches=cell(1,C);
for i=1:C
 cols=6+3*i-2:6+3*i;n=g.normal(:,g.plane(i));B=g.B(:,:,i);pp=sh.contactP(:,i);JP=reshape(sh.JcontactP(:,i,:),3,nx);
 gap(i)=n'*(pp-g.point(:,g.plane(i)))-g.offset(g.plane(i));Dg=n'*JP;
 if ~isempty(history),prev=tsfs.shape_at(history,g.s(i));slip(:,i)=B'*(pp-prev);Dv=B'*JP;else,Dv=zeros(2,nx);end
 [rr,G,inf]=tsfs.project_contact(x(cols(1)),x(cols(2:3)),gap(i),slip(:,i),g.mu(i),o.rhoNormal,o.rhoTangent);
 DX=zeros(6,nx);DX(1:3,cols)=diag(scale(cols));DX(4,:)=Dg;DX(5:6,:)=Dv;
 projection(:,i)=rr;Jproj(3*i-2:3*i,:)=G*DX;work(i)=inf.work;branches{i}=inf.branch;
end
c=[c;projection(:)];Jc=[Jc;Jproj];ng=size(sh.collisionP,2);h=zeros(P*ng,1);Jh=zeros(P*ng,nx);
for j=1:P
 rows=(j-1)*ng+(1:ng);h(rows)=-(g.normal(:,j)'*(sh.collisionP-g.point(:,j))-g.offset(j))';
 for a=1:nx,Jh(rows,a)=-(g.normal(:,j)'*sh.JcollisionP(:,:,a))';end
end
% Redundant exact circular-cone inequality supplies a useful SQP boundary
% derivative while the disk projection is identically zero in sticking.
for i=1:C
 cols=6+3*i-2:6+3*i;tau=x(cols(2:3));nt=norm(tau);hc=nt-g.mu(i)*x(cols(1));row=zeros(1,nx);
 row(cols(1))=-g.mu(i)*scale(cols(1));
 if nt>0,row(cols(2:3))=(tau'/nt).*scale(cols(2:3))';end
 h=[h;hc];Jh=[Jh;row];
end
e=struct('r',r,'J',J,'c',c,'Jc',Jc,'h',h,'Jh',Jh,'objective',0.5*(r'*r),'shape',sh, ...
 'gap',gap,'slip',slip,'projection',projection,'work',work,'branches',{branches},'fbgRms',sqrt(mean(ru.^2)));
end
