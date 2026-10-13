function e=linearized_evaluate(fr,g,z,scale,model)
% Affine rod/sensor outputs in common Cartesian load coordinates.
% Friction is NOT linearized: mode_branch retains its exact circular law.
e=model.e;dz=z-model.z;sh=e.shape;nx=numel(z);C=numel(g.s);ng=numel(e.h)-C;
sh.x=z.*scale;sh.tipMoment=sh.tipMoment+sh.JtipMoment*dz;
sh.prediction=reshape(sh.prediction(:)+sh.Jprediction*dz,size(sh.prediction));
sh.contactP=sh.contactP+reshape(reshape(sh.JcontactP,3*C,nx)*dz,3,C);
sh.collisionP=sh.collisionP+reshape(reshape(sh.JcollisionP,[],nx)*dz,size(sh.collisionP));
for i=1:C
 cc=6+3*i-2:6+3*i;JP=reshape(sh.JcontactP(:,i,:),3,nx);
 sh.force(:,i)=[g.normal(:,g.plane(i)) g.B(:,:,i)]*sh.x(cc);
 e.gap(i)=e.gap(i)+g.normal(:,g.plane(i))'*JP*dz;
 if ~isempty(fr.previous),e.slip(:,i)=e.slip(:,i)+g.B(:,:,i)'*JP*dz;end
end
e.r=e.r+e.J*dz;e.c(1:3)=e.c(1:3)+e.Jc(1:3,:)*dz;e.h(1:ng)=e.h(1:ng)+e.Jh(1:ng,:)*dz;
e.objective=.5*(e.r'*e.r);e.fbgRms=sqrt(mean(e.r(1:numel(fr.u)).^2));
sh.rhsEvaluations=0;sh.segments=0;sh.pieces={};e.shape=sh;
% Only the affine fields above may be used by the inner solver. Nonlinear
% p/R/u/projection are not predictions; final results always replace e by IVP.
end
