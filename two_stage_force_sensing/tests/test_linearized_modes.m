function test_linearized_modes
[root,~]=setup_tsfs;addpath(fullfile(root,'experiments','inclined_wall'));a=load(fullfile(root,'results','runs','inclined_wall_rotation_20261010','truth.mat'));b=load(fullfile(root,'results','runs','inclined_wall_rotation_20261010','comparison.mat'),'runs');frames=tsfs.read_input(a.input);o=tsfs.defaults;cfg=linearized_mode_settings;fr=frames{2};fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;g=b.runs{1,2}.geometry;previous=b.runs{1,1};[z,scale,prior]=tsfs.initialize(fr,g,previous,o);model=tsfs.linearized_model(fr,g,z,scale,prior,previous,o);
% Affine callbacks must agree exactly at the expansion point and have second-order error.
e0=tsfs.linearized_evaluate(fr,g,z,scale,model);assert(norm(e0.r-model.e.r)<1e-12);assert(norm(e0.h-model.e.h)<1e-12);
direction=sin((1:numel(z))');errors=[];
for h=[1e-3 5e-4]
 ea=tsfs.linearized_evaluate(fr,g,z+h*direction,scale,model);en=tsfs.evaluate(fr,g,z+h*direction,scale,prior,model.history,o,true);errors(end+1)=norm(ea.shape.prediction-en.shape.prediction,'fro');
end
assert(errors(2)<.35*errors(1));fprintf('AFFINE Taylor error ratio %.4f\n',errors(2)/errors(1));
local=cfg;local.mechanicalModel=model;local.trustRadius=4;local.checkDerivatives=true;
for mode=[0 1 2]
 q=tsfs.mode_branch(fr,g,previous,o,local,mode,z,scale,prior);assert(q.counts.augmentedIVPs==0&&q.counts.affineEvaluations>0);fprintf('AFFINE mode%d derivative %.3g\n',mode,q.derivativeCheck);
end
g0=g;g0.mu(:)=0;q=tsfs.mode_branch(fr,g0,previous,o,local,3,z,scale,prior);assert(q.counts.augmentedIVPs==0);fprintf('AFFINE mu0 derivative %.3g\n',q.derivativeCheck);
sol=tsfs.mode_solve(fr,g,previous,o,cfg);assert(sol.converged&&sol.modeAccepted&&sol.modeFeasible);d=tsfs.diagnose(fr,g,sol,o);fprintf('LINEAR frame2 %.3fs mode%s success%d accepted%d IVP%d outer%d FBG%.3g tiperror%.4g flags%s\n',sol.seconds,mat2str(sol.mode),sol.converged,sol.modeAccepted,sol.counts.augmentedIVPs,sol.outerIterations,d.fbgRms,norm(sol.evaluation.shape.x(4:6)-a.truth.tipForce(:,2)),strjoin(d.flags,';'));
save(fullfile(root,'results','runs','linearized_modes_20261011','smoke.mat'),'sol','d','errors','-v7.3');
end
