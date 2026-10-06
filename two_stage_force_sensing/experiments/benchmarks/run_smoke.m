function run_smoke
[root,paths]=setup_tsfs;names={'sliding_clean','s_channel_two_contact'};o=tsfs.defaults;o.verbose=true;
for j=1:numel(names)
 a=load(fullfile(root,'datasets',[names{j} '.mat']));frames=tsfs.read_input(a.sensorInput);
 fprintf('\n%s: %d frames\n',names{j},numel(frames));result=tsfs.step(frames{1},[],o);
 save(fullfile(paths.results,[names{j} '_smoke.mat']),'result','o','-v7.3');
 fprintf('time %.3f geometry %.3f mechanics %.3f s=%s tip=%s fc=%s flags=%s\n',result.seconds,result.geometry.seconds,result.mechanics.seconds,mat2str(result.geometry.s),mat2str(result.tipForce),mat2str(result.contactForce),strjoin(result.diagnostics.flags,','));
end
end
