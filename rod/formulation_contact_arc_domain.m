function [lower,upper,A,b] = formulation_contact_arc_domain(s,seeds,spec,T,minSeparation,mode)
% Physical ordering across the whole rod; partitions are a legacy ablation.
s=s(:)';seeds=seeds(:);K=spec.contactCount;
assert(numel(s)>=2&&all(isfinite(s))&&all(diff(s)>0)&&numel(seeds)==K&& ...
    all(isfinite(seeds))&&all(diff(seeds)>0)&&isscalar(minSeparation)&&minSeparation>0, ...
    'rod:InvalidContactDomain','Invalid ordered contact domain.');
assert(ismember(mode,{'ordered','partitioned'}),'rod:InvalidContactDomain','Unknown arc-domain mode.');
if strcmp(mode,'partitioned')
    edges=[s(1);(seeds(1:end-1)+seeds(2:end))/2;s(end)];
    lower=edges(1:K)+minSeparation/2;upper=edges(2:K+1)-minSeparation/2;
else
    lower=s(1)+minSeparation/2+(0:K-1)'*minSeparation;
    upper=s(end)-minSeparation/2-(K-1:-1:0)'*minSeparation;
end
assert(all(lower<upper)&&all(seeds>=lower)&&all(seeds<=upper), ...
    'rod:InvalidContactDomain','Candidate positions cannot satisfy rod endpoints and minimum separation.');
A=sparse(T*max(K-1,0),spec.stateLength*T);b=-minSeparation*ones(size(A,1),1);
for k=1:T
    for j=1:K-1
        row=(k-1)*(K-1)+j;offset=(k-1)*spec.stateLength;
        A(row,offset+spec.contact(j).s)=1;A(row,offset+spec.contact(j+1).s)=-1;
    end
end
end
