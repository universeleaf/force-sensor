function run_inclined_rotation_modes
[root,~]=setup_tsfs;tag='inclined_wall_rotation_20261010';folder=fullfile(root,'results','runs',tag);a=load(fullfile(folder,'maximum_insertion_seed.mat'),'seed');
replay=struct('source',fullfile(folder,'truth.mat'),'initialEstimates',{a.seed.estimates});
search=struct('modeSearch','first-acceptable','allAssignments',false,'fbgRmsAcceptance',0.01,'solver','conditional-modes/fmincon-SQP','initialization','restore each estimator from insertion frame 21; no truth force initialization','sourceFrame',21,'sourceRun','inclined_wall_modes_conditional_1e6_20261010','acceptanceRule','positive native exit AND violation <= 1e-6 AND whitened FBG RMS <= .01');
run_inclined_modes(Inf,tag,search,replay);
end
