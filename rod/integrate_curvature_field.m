function [R,p] = integrate_curvature_field(tube,u,base)
%INTEGRATE_CURVATURE_FIELD SE(3) exponential midpoint integration.
% Keep intrinsic discontinuities; interpolate only elastic curvature.
% Unlike rotate-then-translate Euler integration, integrates a circular
% unloaded segment exactly and has second-order elastic discretization.
if nargin<3,base=tube.T_base;end
n=numel(tube.s); R=zeros(3,3,n); p=zeros(3,n);
R(:,:,1)=base(1:3,1:3); p(:,1)=base(1:3,4);
elastic=u-tube.uhat;
for j=1:n-1
    ds=tube.s(j+1)-tube.s(j);
    omega=(tube.uhat(:,j)+0.5*(elastic(:,j)+elastic(:,j+1)))*ds;
    a=norm(omega); W=[0 -omega(3) omega(2);omega(3) 0 -omega(1);-omega(2) omega(1) 0];
    if a<1e-5
        A=1-a^2/6; B=0.5-a^2/24; C=1/6-a^2/120;
    else
        A=sin(a)/a; B=(1-cos(a))/a^2; C=(a-sin(a))/a^3;
    end
    W2=W*W;
    p(:,j+1)=p(:,j)+R(:,:,j)*(eye(3)+B*W+C*W2)*[0;0;ds];
    R(:,:,j+1)=R(:,:,j)*(eye(3)+A*W+B*W2);
end
end
