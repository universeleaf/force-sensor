function report = run_project_checks(includeReplay)
% Repeatable checks with individual failures retained in a machine-readable ledger.
% Saved fixtures are explicit dependencies; estimates are never overwritten.
if nargin<1,includeReplay=false;end
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'rod'));
folder=fullfile(root,'out','completion');if ~isfolder(folder),mkdir(folder);end
tests={@test_lcp_dependency,@test_sensor_history_boundary,@test_sensor_contracts, ...
    @test_sensor_config,@test_fbg_likelihood_calibration,@test_result_lifecycle,@test_force_sensitivity, ...
    @test_environment_covariance,@test_depth_plane,@test_reduced_mode_bounds,@test_plane_contact_geometry, ...
    @test_cosserat_collision_geometry,@test_cosserat_work_budget,@test_nonlinear_shape_seed,@test_observation_scaled_map, ...
    @test_latent_fbg_history,@test_integrated_history_point, ...
    @test_history_resolution,@test_result_integrity,@test_friction_regression,@test_friction_quality, ...
    @test_planar_energy_rod,@test_contact_shooting,@test_contact_separation_gate, ...
    @test_intrinsic_curvature_reconstruction,@test_rotation_equivariance, ...
    @test_multi_contact_map, ...
    @()run_rod_plane_force_sensing_experiment('jacobian-audit', ...
        fullfile(root,'out','video','inverse_results.mat')),@checkSyntax};
if includeReplay
    tests=[tests,{@()test_sensor_replay(fullfile(root,'out','stage1','video_seeded')), ...
        @()test_sensor_replay(fullfile(root,'out','stage1','wall_tip_seeded'))}];
end
report=struct('scope','Code correctness and fixed regressions; not publication readiness or universal force accuracy.', ...
    'includeReplay',includeReplay,'state','running','runRecord',new_run_record(), ...
    'allPassed',false,'checks',{{}});
path=fullfile(folder,'project_checks.json');atomic_write_artifact(path,'json',report);
for k=1:numel(tests)
    timer=tic; item=struct('name',func2str(tests{k}),'passed',false,'seconds',0,'error','');
    try,tests{k}();item.passed=true;catch err,item.error=getReport(err,'extended','hyperlinks','off');end
    item.seconds=toc(timer);report.checks{k}=item;
    fprintf('Check %d/%d: %s, passed=%d\n',k,numel(tests),item.name,item.passed);
    atomic_write_artifact(path,'json',report);
end
report.state='complete';report.allPassed=all(cellfun(@(v)v.passed,report.checks));
report.runRecord.state='complete';
atomic_write_artifact(path,'json',report);
assert(report.allPassed,'rod:ProjectCheckFailure','Project checks failed; inspect out/completion/project_checks.json.');
disp('PROJECT_CHECKS_PASSED');
    function checkSyntax()
        files=dir(fullfile(root,'rod','*.m'));files(end+1)=dir(fullfile(root,'force.m'));
        for j=1:numel(files)
            issues=checkcode(fullfile(files(j).folder,files(j).name),'-id');
            assert(~any(strcmp({issues.id},'SYNER')),'Syntax error in %s.',files(j).name);
        end
    end
end
