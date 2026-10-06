function g=geometry(fr,o)
clock=tic;sh=tsfs.reconstruct(fr,true);P=size(fr.point,2);s=sh.s;
candidateS=[];plane=[];lower=[];upper=[];flags={};
for j=1:P
 gap=fr.normal(:,j)'*(sh.p-fr.point(:,j));
 idx=find(gap(2:end-1)<=gap(1:end-2)&gap(2:end-1)<gap(3:end))+1;
 idx=idx(s(idx)>s(1)+o.endpointMarginMm&s(idx)<s(end)-o.endpointMarginMm&gap(idx)<o.candidateDistanceMm);
 for k=1:numel(idx)
  ii=idx(k);lo=s(1)+o.endpointMarginMm;hi=s(end)-o.endpointMarginMm;
  if k>1,lo=mean(s(idx(k-1:k)));end
  if k<numel(idx),hi=mean(s(idx(k:k+1)));end
  candidateS(end+1)=s(ii);plane(end+1)=j;lower(end+1)=lo;upper(end+1)=hi;
 end
end
[candidateS,order]=sort(candidateS);plane=plane(order);lower=lower(order);upper=upper(order);
if numel(plane)>o.maxContacts
 flags{end+1}='candidate-cap-exceeded';error('tsfs:CandidateBudget','Candidate budget exceeded; increase maxContacts.');
end
C=numel(plane);B=zeros(3,2,P);E=zeros(3,3,P);escale=zeros(3,P);
for j=1:P
 B(:,:,j)=tsfs.basis(fr.normal(:,j));A=blkdiag(fr.normal(:,j)',B(:,:,j)');cov=A*fr.CE(:,:,j)*A';
 [E(:,:,j),flag]=chol((cov+cov')/2,'lower');assert(flag==0,'tsfs:GeometryPrior','Positive definite projected environment covariance required.');
 escale(:,j)=sqrt(diag(cov));
end
arcscale=max(1,(upper-lower)/10);z=zeros(C+3*P,1);
lb=[(lower-candidateS)'./arcscale';repmat([-Inf;-o.geometryNormalChartBound;-o.geometryNormalChartBound],P,1)./escale(:)];
ub=[(upper-candidateS)'./arcscale';repmat([Inf;o.geometryNormalChartBound;o.geometryNormalChartBound],P,1)./escale(:)];
opts=optimoptions('lsqnonlin','Algorithm','trust-region-reflective','Display','off', ...
 'SpecifyObjectiveGradient',true,'MaxIterations',o.geometryMaxIterations,'MaxFunctionEvaluations',o.geometryMaxEvaluations, ...
 'FunctionTolerance',o.geometryFunctionTolerance,'OptimalityTolerance',o.geometryOptimalityTolerance,'StepTolerance',o.geometryStepTolerance);
if isempty(z),cost=0;r=[];exitflag=1;out=struct('iterations',0,'funcCount',0,'firstorderopt',0);
else,[z,cost,r,exitflag,out]=lsqnonlin(@objective,z,lb,ub,opts);end
[arcs,n,delta]=decode(z);[pc,t]=tsfs.shape_at(sh,arcs);Bc=zeros(3,2,C);
for i=1:C,Bc(:,:,i)=tsfs.basis(n(:,plane(i)));end
if any(diff(arcs)<1e-3),flags{end+1}='coincident-candidates-corner-or-ambiguity';end
g=struct('s',arcs,'plane',plane,'normal',n,'offset',delta,'point',fr.point, ...
 'B',Bc,'mu',fr.mu(plane),'kinematicShape',sh,'predictedPoint',pc,'predictedTangent',t, ...
 'residual',r,'cost',cost,'exitflag',exitflag,'iterations',out.iterations,'evaluations',out.funcCount, ...
 'optimality',out.firstorderopt,'flags',{flags},'seconds',toc(clock),'arcBounds',[lower;upper],'assumedIntrinsicAxes',setdiff(1:3,fr.axes));
 function [arcs,n,delta]=decode(v)
  arcs=candidateS+arcscale.*v(1:C)';q=reshape(v(C+1:end),3,P).*escale;n=zeros(3,P);delta=q(1,:);
  for a=1:P,n(:,a)=fr.normal(:,a)+B(:,:,a)*q(2:3,a);n(:,a)=n(:,a)/norm(n(:,a));end
 end
 function r=residual(v)
  [arcs,n,delta]=decode(v);[pc,t,Jp,Jt]=tsfs.shape_at(sh,arcs);raw=zeros(2*C,1);Ju=zeros(2*C,numel(fr.u));
  for a=1:C
   j=plane(a);raw(2*a-1)=n(:,j)'*(pc(:,a)-fr.point(:,j))-delta(j);raw(2*a)=n(:,j)'*t(:,a);
   Ju(2*a-1,:)=n(:,j)'*reshape(Jp(:,a,:),3,[]);Ju(2*a,:)=n(:,j)'*reshape(Jt(:,a,:),3,[]);
  end
  cov=Ju*fr.CU*Ju'+diag(repmat([o.geometryPositionModelStdMm^2;o.geometryTangentModelStd^2],C,1));
  [L,flag]=chol((cov+cov')/2,'lower');assert(flag==0||C==0,'tsfs:GeometryCovariance','Unsupported deterministic geometric residual; declare model errors.');
  r=zeros(2*C+3*P,1);if C>0,r(1:2*C)=L\raw;end
  q=reshape(v(C+1:end),3,P).*escale;
  for j=1:P,r(2*C+3*j-2:2*C+3*j)=E(:,:,j)\q(:,j);end
 end
 function [r,J]=objective(v)
  r=residual(v);if nargout>1
   J=zeros(numel(r),numel(v));
   % Central derivatives include changing cross-contact covariance; no mechanics.
   for a=1:numel(v),h=2e-5*max(1,abs(v(a)));vp=v;vm=v;vp(a)=vp(a)+h;vm(a)=vm(a)-h;J(:,a)=(residual(vp)-residual(vm))/(2*h);end
  end
 end
end
