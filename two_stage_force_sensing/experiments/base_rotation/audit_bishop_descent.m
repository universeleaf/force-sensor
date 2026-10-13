function audit_bishop_descent
% Demonstrate (or reject) feasible local descent from the stalled 22.5-degree state.
[root,paths]=setup_tsfs;src=fullfile(root,'results','runs','base_rotation_bishop_20261007','rotation');a=load(fullfile(src,'truth_mu_300.mat'));d=load(fullfile(src,'ours.mat'));b=load(fullfile(paths.results,'audit.mat'));frames=tsfs.read_input(a.input);fr=frames{7};g=b.cases{1,1}.geometry;sol=b.continued{1,1};prev=d.runs{1,2,6};history=prev.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];o=tsfs.defaults;
% Keep the original ODE and objective for directly comparable cost values.
rows=[];
for state={'original','tight-resume'}
 if strcmp(state{1},'original'),sol=b.cases{1,1}.mechanics;else,sol=b.continued{1,1};end
 z=sol.z;scale=sol.scale;prior=sol.prior;e=tsfs.evaluate(fr,g,z,scale,prior,history,o,true);
N=null(e.Jc);pg=N'*(e.J'*e.r);direction=-N*pg/norm(pg);
opts=optimoptions('lsqnonlin','Display','off','SpecifyObjectiveGradient',true,'MaxIterations',50,'MaxFunctionEvaluations',150,'FunctionTolerance',1e-15,'StepTolerance',1e-13,'OptimalityTolerance',1e-12);
for alpha=[0 1e-5 1e-4 1e-3 1e-2]
 target=z+alpha*direction;
 [zz,~,~,flag]=lsqnonlin(@retract,target,[],[],opts);ee=tsfs.evaluate(fr,g,zz,scale,prior,history,o,true);fn=ee.shape.x(7);
 row=struct('state',state{1},'step',alpha,'objective',ee.objective,'costChange',ee.objective-e.objective,'equalityInf',norm(ee.c,inf),'inequalityInf',max([0;ee.h]),'tipMomentInfNmm',norm(ee.shape.tipMoment,inf),'projectionInfN',norm(ee.projection,inf),'gapMm',ee.gap,'coneExcessN',max(0,norm(ee.shape.x(8:9))-g.mu*fn),'slipMm',norm(ee.slip),'branch',ee.branches{1},'retractionExitFlag',flag);
 rows=[rows;row];
end
end
writetable(struct2table(rows),fullfile(paths.results,'feasible_descent.csv'));disp(struct2table(rows));
 function [r,J]=retract(v)
  ee=tsfs.evaluate(fr,g,v,scale,prior,history,o,true);r=[ee.c;N'*(v-target)];J=[ee.Jc;N'];
 end
end
