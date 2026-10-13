function [w,Jw]=bishop_observation(u,psi,Ju,Jpsi)
% t'=w1*b1+w2*b2; R=B*Rz(psi), psi'=u3, B(base)=R(base).
c=cos(psi);s=sin(psi);w=[c.*u(2,:)+s.*u(1,:);s.*u(2,:)-c.*u(1,:)];
Jw=[];
if nargin>2
 Jw=zeros(2,size(u,2),size(Ju,3));
 for j=1:size(Ju,3)
  Jw(:,:,j)=[c.*Ju(2,:,j)+s.*Ju(1,:,j);s.*Ju(2,:,j)-c.*Ju(1,:,j)]+[-w(2,:);w(1,:)].*Jpsi(:,:,j);
 end
end
end
