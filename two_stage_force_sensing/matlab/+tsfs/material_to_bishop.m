function [w,psi]=material_to_bishop(s,u)
% Simulation adapter: integrate dense material twist's piecewise cubic exactly.
% This consumes simulator state, never runs inside the estimator input adapter.
s=s(:)';pp=pchip(s,u(3,:));h=diff(pp.breaks);c=pp.coefs;
increments=c(:,1)'.*h.^4/4+c(:,2)'.*h.^3/3+c(:,3)'.*h.^2/2+c(:,4)'.*h;
psi=[0 cumsum(increments)];w=tsfs.bishop_observation(u,psi);
end
