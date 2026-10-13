function d=diagnose_relaxed(fr,g,sol,o)
% Reporting only. Separate relaxed feasibility from exact-law warning targets.
clock=tic;d=tsfs.diagnose(fr,g,sol,o);strictFlags=d.flags;
physical={'penetration','terminal-equilibrium','contact-projection','friction-cone','normal-complementarity','positive-friction-work'};
d.exactLawFlags=intersect(strictFlags,physical,'stable');d.flags=setdiff(strictFlags,physical,'stable');
tol=sol.configuration.constraintTolerance;eps=sol.epsilon;e=sol.evaluation;C=numel(g.s);fn=e.shape.x(7:3:end);
if d.maximumPenetrationMm>tol,d.flags{end+1}='penetration';end
if norm(e.shape.tipMoment,inf)>max(sol.scale(1:3))*tol,d.flags{end+1}='terminal-equilibrium';end
coneSquared=zeros(1,C);slipSquared=zeros(1,C);
for i=1:C
 cols=6+3*i-2:6+3*i;tau=e.shape.x(cols(2:3));coneSquared(i)=tau'*tau-(g.mu(i)*fn(i))^2;
 slipSquared(i)=sum(e.slip(:,i).^2)-sol.auxiliaryBeta(i)^2;
end
if any(fn < -tol)||any(sol.auxiliaryBeta < -tol),d.flags{end+1}='negative-contact-variable';end
if any(coneSquared>tol),d.flags{end+1}='relaxed-force-cone';end
if any(slipSquared>tol),d.flags{end+1}='relaxed-slip-cone';end
if any(sol.relaxed.normalProduct>eps+tol),d.flags{end+1}='relaxed-normal-product';end
if any(sol.relaxed.frictionGap>eps+tol),d.flags{end+1}='relaxed-friction-gap';end
if sol.relaxed.violation>tol,d.flags{end+1}='relaxed-nlp-infeasible';end
d.relaxedFeasible=sol.relaxed.violation<=tol&&all(fn>=-tol)&&all(sol.auxiliaryBeta>=-tol);
d.forceConeSquaredExcess=coneSquared;d.slipConeSquaredExcess=slipSquared;
d.thresholds=struct('gapMm',tol,'momentNmm',max(sol.scale(1:3))*tol,'forceConeN2',tol,'slipConeMm2',tol,'normalProductUpperNmm',eps+tol,'frictionGapUpperNmm',eps+tol);
d.seconds=toc(clock);
end
