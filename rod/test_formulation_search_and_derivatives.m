function test_formulation_search_and_derivatives()
% A feasible migrating contact must not be excluded by its initial partition.
spec=formulation_window_spec(2,[1 2],4);seeds=[20 40];
[lo,hi,A,b]=formulation_contact_arc_domain([0 100],seeds,spec,2,.1,'ordered');
X=zeros(spec.stateLength,2);X(spec.contact(1).s,:)=[20 60];X(spec.contact(2).s,:)=[40 80];
assert(all(X([spec.contact.s],:)>=lo,'all')&&all(X([spec.contact.s],:)<=hi,'all')&&all(A*X(:)<=b), ...
    'rod:ContactMigrationRegression','An ordered feasible migration remains locked in its initial partition.');
[~,legacyUpper]=formulation_contact_arc_domain([0 100],seeds,spec,2,.1,'partitioned');
assert(X(spec.contact(1).s,2)>legacyUpper(1),'Regression did not cross the original partition.');
X(spec.contact(2).s,2)=59;assert(any(A*X(:)>b),'Reversed contacts passed the ordering constraint.');
options=struct('candidateMergeMm',10,'minArcSeparationMm',.1,'maxContacts',4, ...
    'candidateTrackMaxFraction',.25,'referencePeriodSeconds',1);
raw={[1 20 0],[1 35 0],[1 50 0]};
[c,a]=track_formulation_candidates(raw,[0 100],[0 1 2],options);
assert(numel(c.planeIndex)==1&&isequal(c.seedArcByFrameMm,[20 35 50])&&a.generatedCount==3, ...
    'rod:MovingCandidateRegression','One moving minimum became several contact slots.');
raw={[1 20 0;1 70 0],[1 35 0;1 85 0]};
[c,~]=track_formulation_candidates(raw,[0 100],[0 1],options);
assert(numel(c.planeIndex)==2&&isequal(c.seedArcByFrameMm,[20 35;70 85]), ...
    'rod:DistinctCandidateRegression','Two separate contacts were merged during temporal tracking.');
x=[.3;-.4;1.2];fun=@(v)[sin(v(1))*v(2);v(1)^2+v(3);exp(v(2)-v(3));v(1)*v(3)];
[value,J,n]=formulation_derivative_bundle(fun,x);
reference=[cos(x(1))*x(2),sin(x(1)),0;2*x(1),0,1;0,exp(x(2)-x(3)),-exp(x(2)-x(3));x(3),0,x(1)];
assert(max(abs(J-reference),[],'all')<1e-8&&isequal(value,fun(x))&&n==7, ...
    'rod:SharedDerivativeRegression','Combined finite differences disagree with independent analytic derivatives.');
[accept,initial,candidate]=formulation_restoration_merit([.02;.001],[1.135;.002]);
assert(~accept&&candidate>initial,'rod:RestorationRegression','A worse restoration replaced its initializer.');
assert(formulation_restoration_merit([.02;.001],[.001;.0005]), ...
    'rod:RestorationRegression','An improved restoration was rejected.');
assert(~formulation_restoration_merit([.02;.001],[NaN;0]), ...
    'rod:RestorationRegression','A nonfinite restoration was accepted.');
fprintf('test_formulation_search_and_derivatives: PASS\n');
end
