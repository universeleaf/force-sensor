function t = contact_tangent(n)
%CONTACT_TANGENT Deterministic unit tangent for the contact direction basis.
n=n(:)/norm(n);
% The smallest normal component switches axes under tiny perturbations of
% an axis-aligned plane, rotating friction direction labels by 90 degrees.
% Use a chart continuous around each axis-aligned plane in this study.
[~,normalAxis]=max(abs(n));
axisIdx=mod(normalAxis,3)+1;
axisVector=zeros(3,1); axisVector(axisIdx)=1;
t=axisVector-n*(n'*axisVector);
t=t/norm(t);
end
