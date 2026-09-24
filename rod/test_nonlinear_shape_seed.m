function test_nonlinear_shape_seed()
root=fileparts(fileparts(mfilename('fullpath')));validate_lcp_dependency(root);
cfg=run_rod_plane_force_sensing_experiment('sensor-config');
cfg=formulation_solver_config(cfg);cfg=calibrate_fbg_likelihood(cfg,0,1e-6);
s=0:2:90;tube=CreatTube(90,s,repmat([0;.02;0],1,numel(s)));
known=[63;.15;-.03;-.5;.1;.02;-.12];
shape=solve_cosserat_force_map(tube,known(1),known(2:4),known(5:7));
idx=unique(round(linspace(1,numel(s),24)));
initial=known+[2;.08;-.02;-.1;-.04;.01;.04];
[answer,info]=refine_cosserat_shape_seed(tube,shape.u(:,idx),idx,initial,cfg.forceSensor);
assert(info.accepted&&info.exitflag>0&&info.finalSquaredResidual<.01*info.initialSquaredResidual);
assert(abs(answer(1)-known(1))<.02&&norm(answer(2:7)-known(2:7))<.02, ...
    'Sparse current bending did not recover the synthetic nonlinear seed.');
fprintf('Nonlinear sparse-shape seed passed: force error %.4g N, %.2f s.\n',norm(answer(2:7)-known(2:7)),info.seconds);
end
