function d=diagnose(fr,g,sol,o)
% Pure reporting function. No returned state is consumed by either solver.
clock=tic;e=sol.evaluation;sh=e.shape;
assert(numel(sh.pieces)==numel(sh.edges)-1,'tsfs:MissingDenseSolution','Dense ODE solution required; use saved diagnostics for serialized benchmark results.');C=numel(g.s);P=size(g.normal,2);arcs=unique([sh.s(1):o.diagnosticStepMm:sh.s(end) sh.s(end) g.s]);p=zeros(3,numel(arcs));
for j=1:numel(sh.pieces),take=arcs>=sh.edges(j)&arcs<=sh.edges(j+1);if any(take),v=deval(sh.pieces{j},arcs(take));p(:,take)=v(1:3,:);end,end
mingap=Inf;for j=1:P,mingap=min(mingap,min(g.normal(:,j)'*(p-g.point(:,j))-g.offset(j)));end
fn=sh.x(7:3:end);cone=zeros(1,C);tangent=zeros(1,C);slidingError=zeros(1,C);
for i=1:C
 cols=6+3*i-2:6+3*i;tau=sh.x(cols(2:3));cone(i)=max(0,norm(tau)-g.mu(i)*fn(i));tangent(i)=g.normal(:,g.plane(i))'*sh.contactT(:,i);
 if norm(e.slip(:,i))>1e-8,slidingError(i)=norm(tau+g.mu(i)*fn(i)*e.slip(:,i)/norm(e.slip(:,i)));end
end
flags=g.flags;
if g.exitflag<=0,flags{end+1}='geometry-not-converged';end
if norm(g.residual(1:2*C))>4*sqrt(max(1,2*C)),flags{end+1}='geometry-map-mismatch';end
if ~sol.converged,flags{end+1}=['mechanics-' sol.reason];end
if mingap < -o.gapTolerance,flags{end+1}='penetration';end
if any(abs(tangent(fn>1e-3))>o.tangencyWarning),flags{end+1}='active-contact-tangency';end
if norm(sh.tipMoment,inf)>o.momentTolerance,flags{end+1}='terminal-equilibrium';end
if any(abs(e.projection)>o.projectionTolerance,'all'),flags{end+1}='contact-projection';end
if any(cone>o.coneTolerance),flags{end+1}='friction-cone';end
if any(abs(fn(:)'.*e.gap)>o.productTolerance),flags{end+1}='normal-complementarity';end
if any(e.work>1e-8),flags{end+1}='positive-friction-work';end
if e.fbgRms>o.fbgRmsWarning,flags{end+1}='fbg-mismatch';end
if isempty(fr.previous)&&C>0&&any(g.mu>0),flags{end+1}='slip-history-unavailable';end
missing=setdiff(1:3,fr.axes);unobservedDelta=sh.u(missing,:)-fr.tube.uhat(missing,:);unobservedMismatch=max([0;abs(unobservedDelta(:))]);
if ismember(3,missing)&&max(abs(sh.u(3,:)-fr.tube.uhat(3,:)))>1e-5,flags{end+1}='stage1-unobserved-twist';end
if numel(fr.axes)<2,flags{end+1}='one-channel-observability';end
J=e.J(1:numel(fr.u),:);E=sh.JtipMoment;
if rcond(E(:,1:3))>1e-12,Jforce=J(:,4:end)-J(:,1:3)*(E(:,1:3)\E(:,4:end));else,Jforce=J(:,4:end);flags{end+1}='singular-moment-elimination';end
sing=svd(Jforce,'econ');
condition=Inf;if ~isempty(sing)&&sing(end)>0,condition=sing(1)/sing(end);end
if condition>1e6,flags{end+1}='weak-contact-tip-separation';end
mismatch=vecnorm(sh.contactP-g.predictedPoint);
d=struct('flags',{flags},'minimumGapMm',mingap,'maximumPenetrationMm',max(0,-mingap), ...
 'gapMm',e.gap,'tangency',tangent,'coneExcessN',cone,'projectionN',e.projection,'normalProductNmm',fn(:)'.*e.gap, ...
 'frictionWorkNmm',e.work,'slidingLawErrorN',slidingError,'tipMomentNmm',sh.tipMoment,'fbgRms',e.fbgRms, ...
 'contactPointMismatchMm',mismatch,'forceJacobianSingularValues',sing,'forceCondition',condition, ...
 'unobservedCurvatureMismatchPerMm',unobservedMismatch,'candidateCount',C,'activeCount',sum(fn>1e-3),'seconds',toc(clock));
end
