function sh=reconstruct(fr,withSensitivity)
% Sparse elastic curvature interpolation preserves calibrated intrinsic jumps.
s=fr.tube.s;u0=fr.tube.uhat; measuredIntrinsic=interp1(s,u0',fr.arcs,'previous')';
e=fr.u-measuredIntrinsic(fr.axes,:);u=u0;
u(fr.axes,:)=u(fr.axes,:)+interp1(fr.arcs(:),e',s(:),'pchip','extrap')';
[R,p]=integrate_curvature_field(fr.tube,u,fr.base);
sh=struct('s',s,'p',p,'t',reshape(R(:,3,:),3,[]),'R',R,'u',u,'Jp',[],'Jt',[]);
if withSensitivity
 nu=numel(fr.u);sh.Jp=zeros(3,numel(s),nu);sh.Jt=sh.Jp;
 for j=1:nu
  h=1e-7;fp=fr;fm=fr;fp.u(j)=fp.u(j)+h;fm.u(j)=fm.u(j)-h;
  sp=tsfs.reconstruct(fp,false);sm=tsfs.reconstruct(fm,false);
  sh.Jp(:,:,j)=(sp.p-sm.p)/(2*h);sh.Jt(:,:,j)=(sp.t-sm.t)/(2*h);
 end
end
end
