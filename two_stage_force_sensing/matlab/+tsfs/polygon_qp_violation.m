function [violation,circular]=polygon_qp_violation(e,g,codes,y,scale)
% Nonlinear forward audit of polygon maximum dissipation; circular projection separate.
nx=numel(e.shape.x);z=e.shape.x./scale;fake=struct('z',z,'e',e);fr=struct;[A,b,Aeq,beq,lb,ub]=tsfs.polygon_qp_constraints(fr,g,fake,codes,scale,Inf);y(1:nx)=z;violation=max([abs(Aeq*y-beq);A*y-b;lb-y;y-ub;0]);
% rho=1 proximal law uses original circular Coulomb cone, including its zero vertex.
circular=max([abs(e.c(:));e.h(:);0]);
end
