function [Jp,Jt]=reconstruct_bishop_sensitivity(fr,h)
% Same central differences as tsfs.reconstruct (step h on every Bishop
% channel), but all 2*N perturbed fields are interpolated and integrated in
% one batched SE(3) midpoint pass instead of 2*N sequential reconstructions.
nu=numel(fr.u);B=2*nu;U=repmat(fr.u,1,1,B);
for j=1:nu,[a,b]=ind2sub(size(fr.u),j);U(a,b,2*j-1)=U(a,b,2*j-1)+h;U(a,b,2*j)=U(a,b,2*j)-h;end
s=fr.tube.s(:)';mid=(s(1:end-1)+s(2:end))/2;wm=batch_field(fr,mid,U);N=numel(s);
R=repmat(fr.base(1:3,1:3),1,1,B);p=repmat(fr.base(1:3,4),1,1,B);P=zeros(3,N,B);T=zeros(3,N,B);P(:,1,:)=p;T(:,1,:)=R(:,3,:);
I=repmat(eye(3),1,1,B);
for j=1:N-1
 ds=s(j+1)-s(j);v1=-wm(2,j,:)*ds;v2=wm(1,j,:)*ds;z=zeros(1,1,B);
 W=[z z v2;z z -v1;-v2 v1 z];a=sqrt(v1.^2+v2.^2);small=a<1e-5;
 A=sin(a)./a;D=(1-cos(a))./a.^2;E=(a-sin(a))./a.^3;
 A(small)=1-a(small).^2/6;D(small)=0.5-a(small).^2/24;E(small)=1/6-a(small).^2/120;
 W2=pagemtimes(W,W);M=pagemtimes(R,I+D.*W+E.*W2);p=p+M(:,3,:)*ds;R=pagemtimes(R,I+A.*W+D.*W2);
 P(:,j+1,:)=p;T(:,j+1,:)=R(:,3,:);
end
Jp=(P(:,:,1:2:end)-P(:,:,2:2:end))/(2*h);Jt=(T(:,:,1:2:end)-T(:,:,2:2:end))/(2*h);
end

function w=batch_field(fr,query,U)
% tsfs.bishop_field for a batch of measurement arrays (2 x stations x B).
s=fr.tube.s(:)';K=reshape(getTubeK(fr.tube),3,[]);B=size(U,3);
change=find(any(abs(diff(fr.tube.uhat,1,2))>1e-14,1)|any(abs(diff(K,1,2))>1e-10,1))+1;
edges=unique([s(1) s(change) s(end)]);query=query(:)';w=zeros(2,numel(query),B);
for j=1:numel(edges)-1
 if j<numel(edges)-1,obs=fr.arcs>=edges(j)&fr.arcs<edges(j+1);take=query>=edges(j)&query<edges(j+1);
 else,obs=fr.arcs>=edges(j)&fr.arcs<=edges(j+1);take=query>=edges(j)&query<=edges(j+1);end
 assert(sum(obs)>=2,'tsfs:BishopSampling','Each calibrated segment needs at least two sensor stations.');
 Y=reshape(permute(U(:,obs,:),[2 1 3]),sum(obs),2*B);V=interp1(fr.arcs(obs),Y,query(take),'pchip','extrap');
 if sum(take)==1,V=V(:)';end
 w(:,take,:)=permute(reshape(V,sum(take),2,B),[2 1 3]);
end
end
