function test_mode_formulation
[root,~]=setup_tsfs;folder=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall');a=load(fullfile(folder,'truth.mat'));b=load(fullfile(folder,'comparison_unreduced_attempt.mat'));frames=tsfs.read_input(a.input);o=tsfs.defaults;cfg=struct('constraintTolerance',1e-6,'checkDerivatives',true);fr=frames{5};fr.CU=eye(numel(fr.u))*1e-8;fr.previous.CU=fr.CU;g=b.runs{1,5}.geometry;previous=b.runs{1,4};[z,scale,prior]=tsfs.initialize(fr,g,previous,o);
for mode=0:2
 sol=tsfs.mode_branch(fr,g,previous,o,cfg,mode,z,scale,prior);fprintf('CHECK mode %d derivative %.3g %s violation %.3g cost %.4g\n',mode,sol.derivativeCheck,sol.reason,sol.modeViolation,sol.evaluation.objective);
end
fr.mu(:)=0;fr.previous.mu(:)=0;g.mu(:)=0;sol=tsfs.mode_branch(fr,g,previous,o,cfg,3,z,scale,prior);assert(norm(sol.evaluation.shape.force-g.normal*sol.evaluation.shape.x(7))<1e-10);fprintf('CHECK mu0 derivative %.3g %s violation %.3g\n',sol.derivativeCheck,sol.reason,sol.modeViolation);
end
