function [value,J,evaluations] = formulation_derivative_bundle(fun,x)
% One central finite-difference sweep for all residual/constraint quantities.
% Identical step rule to the original formulation Jacobian; no approximation
% of rod mechanics, predecessor equilibrium or any posterior factor.
value=fun(x);value=value(:);J=zeros(numel(value),numel(x));evaluations=1;
for j=1:numel(x)
    h=1e-5*max(1,abs(x(j)));a=x;b=x;a(j)=a(j)+h;b(j)=b(j)-h;
    va=fun(a);vb=fun(b);
    assert(numel(va)==numel(value)&&numel(vb)==numel(value), ...
        'rod:DerivativeDimension','Residual/constraint dimensions changed during a derivative sweep.');
    J(:,j)=(va(:)-vb(:))/(2*h);evaluations=evaluations+2;
end
end
