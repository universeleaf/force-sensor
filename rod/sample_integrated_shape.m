function [point,tangent] = sample_integrated_shape(tube,shape,arcMm)
%SAMPLE_INTEGRATED_SHAPE Continuous point on an SE(3)-midpoint reconstruction.
% Use the SAME constant midpoint strain on each cell as
% integrate_curvature_field. Partial exponential integration matches both
% endpoint positions and tangents; the position is C1 across grid nodes.
s=tube.s(:)';arcMm=min(max(arcMm,s(1)),s(end));
j=find(s<=arcMm,1,'last');j=min(j,numel(s)-1);ds=arcMm-s(j);
elastic=shape.u-tube.uhat;
omega=(tube.uhat(:,j)+0.5*(elastic(:,j)+elastic(:,j+1)))*ds;
a=norm(omega);W=[0 -omega(3) omega(2);omega(3) 0 -omega(1);-omega(2) omega(1) 0];
if a<1e-5
    A=1-a^2/6;B=0.5-a^2/24;C=1/6-a^2/120;
else
    A=sin(a)/a;B=(1-cos(a))/a^2;C=(a-sin(a))/a^3;
end
W2=W*W;R=shape.R(:,:,j);
point=shape.p(:,j)+R*(eye(3)+B*W+C*W2)*[0;0;ds];
tangent=R*(eye(3)+A*W+B*W2)*[0;0;1];
end
