function reports = audit_force_identifiability()
%AUDIT_FORCE_IDENTIFIABILITY Linear force-to-curvature sensitivity audit.
% Conditional on known geometry/contact arclength. This is not a global
% observability certificate for the complete nonlinear augmented state.
root=fileparts(fileparts(mfilename('fullpath')));
validate_lcp_dependency(root);
names={'wall','senior'};
reports=struct;
for k=1:numel(names)
    d=load(fullfile(root,'out',names{k},'results.mat'),'results'); r=d.results;
    tube=make_experiment_tube(r.config);
    J=computeJacobian(r.forward.R(:,:,:,end),r.forward.p(:,:,end));
    Jtip=J(end-2:end,:);
    s=r.forward.contactArcLength(end);
    j=find(tube.s<=s,1,'last'); j=min(j,numel(tube.s)-1);
    a=(s-tube.s(j))/(tube.s(j+1)-tube.s(j));
    Jc=(1-a)*J(3*j-2:3*j,:)+a*J(3*j+1:3*j+3,:);
    G=bsxfun(@times,1./getTubeK(tube),[Jc',Jtip']);
    rows=reshape(3*r.measurements.fbgIdx(:)'+(-2:0)',[],1);
    G=G(rows,:);
    sigma=svd(G);
    v.contactS=s;
    v.singularValues=sigma;
    v.rankRelativeTolerance1e8=sum(sigma>max(sigma)*1e-8);
    v.contactTipJacobianMaxDifference=max(abs(Jc-Jtip),[],'all');
    delta=[1;-2;0.5];
    v.curvatureChangeForEqualOppositeForce=max(abs(G*[delta;-delta]));
    v.scope='Known shape and arclength; sparse curvature sensitivity to six force components';
    reports.(names{k})=v;
end
assert(reports.senior.curvatureChangeForEqualOppositeForce<1e-12);
fid=fopen(fullfile(root,'docs','audit_assets','identifiability.json'),'w');
fprintf(fid,'%s',jsonencode(reports,'PrettyPrint',true)); fclose(fid);
disp(reports);
end
