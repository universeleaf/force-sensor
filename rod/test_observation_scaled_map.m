function test_observation_scaled_map()
% Regression: near-exact force estimates still failed complementarity with
% prior-only coordinates after tightening the FBG likelihood. No truth seed.
root=fileparts(fileparts(mfilename('fullpath')));
comparisonPath=fullfile(root,'out','demos','comparison.json');
assert(isfile(comparisonPath),'Missing current contact-demo comparison; run force(''demos'') first.');
comparison=jsondecode(fileread(comparisonPath));
entry=find(strcmp({comparison.cases.id},'inclined_plane') & [comparison.cases.completed],1);
assert(~isempty(entry),'Current contact-demo comparison has no completed inclined_plane case.');
source=fullfile(root,'out','demos',comparison.cases(entry).artifactFolder,'results.mat');
assert(isfile(source),'Missing current inclined-plane results: %s',source);
d=load(source,'sensorInput','truth');input=d.sensorInput;
input.packet=subset_sensor_packet(input.packet,1);
input.config=formulation_solver_config(input.config);
input.config=calibrate_fbg_likelihood(input.config,0,1e-6);
input.config.forceSensor.showProgress=false;
o=estimate_sensor_forces(input);trace=o.ours.solverTrace{1}{1};
assert(trace.stateScaling.observationScaled && trace.exitflag>0 && ...
    o.ours.activeConstraintResidual<1e-7, ...
    'Tight FBG likelihood regressed to an infeasible SQP endpoint.');
assert(norm(o.ours.contactForceResultant-d.truth.contactForce(:,1))<5e-4, ...
    'Feasibility was recovered at the expense of clean force accuracy.');
if trace.polishAccepted
    assert(strcmp(trace.finalStateSource,'polish') && ...
        trace.firstOrderOptimality==trace.polish.firstOrderOptimality && ...
        trace.iterations==trace.polish.iterations, ...
        'Final convergence diagnostics describe a discarded homotopy state.');
end
fprintf('Observation-scaled MAP passed: constraint %.3g, exit %d.\n', ...
    o.ours.activeConstraintResidual,trace.exitflag);
end
