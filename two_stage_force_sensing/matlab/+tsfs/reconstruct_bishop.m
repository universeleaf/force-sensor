function sh=reconstruct_bishop(fr)
% SE(3) midpoint integration of Bishop curvature, including known breaks.
s=fr.tube.s(:)';w=tsfs.bishop_field(fr,s);mid=(s(1:end-1)+s(2:end))/2;wm=tsfs.bishop_field(fr,mid);
R=zeros(3,3,numel(s));p=zeros(3,numel(s));R(:,:,1)=fr.base(1:3,1:3);p(:,1)=fr.base(1:3,4);
for j=1:numel(s)-1
 ds=s(j+1)-s(j);v=[-wm(2,j);wm(1,j);0]*ds;W=tsfs.hat(v);a=norm(v);
 if a<1e-5,A=1-a^2/6;D=0.5-a^2/24;E=1/6-a^2/120;
 else,A=sin(a)/a;D=(1-cos(a))/a^2;E=(a-sin(a))/a^3;end
 p(:,j+1)=p(:,j)+R(:,:,j)*(eye(3)+D*W+E*(W*W))*[0;0;ds];
 R(:,:,j+1)=R(:,:,j)*(eye(3)+A*W+D*(W*W));
end
% u and R here are Bishop angular strain and frame, not material states.
% The legacy linear initializer uses them only as an approximate starting guess.
sh=struct('s',s,'p',p,'t',reshape(R(:,3,:),3,[]),'R',R,'u',[-w(2,:);w(1,:);zeros(1,numel(s))], ...
 'w',w,'frameType','bishop','Jp',[],'Jt',[]);
end
