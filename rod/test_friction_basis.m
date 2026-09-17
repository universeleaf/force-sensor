function test_friction_basis()
% Small normal perturbations must not rotate friction direction labels.
for n={[0;0;-1],[-1;0;0]}
    n0=n{1}; t0=contact_tangent(n0);
    for j=1:3
        np=n0; np(j)=np(j)+1e-7; np=np/norm(np);
        tp=contact_tangent(np);
        assert(abs(np'*tp)<1e-12 && abs(norm(tp)-1)<1e-12);
        assert(t0'*tp>0.999999, 'Tiny normal perturbation rotated the friction basis.');
    end
end
disp('Friction basis continuity checks passed.');
end
