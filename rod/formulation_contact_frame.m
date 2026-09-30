function [normal, directions] = formulation_contact_frame(reference, eta, m)
%FORMULATION_CONTACT_FRAME Exponential normal chart and polyhedral friction.
reference=reference(:)/norm(reference); B=basis(reference); angle=norm(eta);
if angle<1e-12, normal=reference; else
    tangent=B*(eta(:)/angle); axis=cross(reference,tangent);
    H=[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
    Q=eye(3)+sin(angle)*H+(1-cos(angle))*(H*H);
    normal=Q*reference; B=Q*B;
end
assert(m>=2&&m==round(m),'rod:InvalidFrictionDirections','At least two friction generators are required.');
if m==2, directions=[B(:,1),-B(:,1)]; else
    angles=(0:m-1)*2*pi/m; directions=B*[cos(angles);sin(angles)];
end
end
function B=basis(n)
ref=[1;0;0]; if abs(ref'*n)>0.9, ref=[0;1;0]; end
t=ref-n*(n'*ref); t=t/norm(t); B=[t,cross(n,t)];
end
