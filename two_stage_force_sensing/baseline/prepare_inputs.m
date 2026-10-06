function folder=prepare_inputs(mode)
% Recorded fixtures by default; regeneration calls upstream builders unchanged.
if nargin<1,mode='recorded';end
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'baseline');if ~isfolder(folder),mkdir(folder);end
if strcmp(mode,'recorded')
 copyfile(fullfile(root,'datasets','sliding_clean.mat'),fullfile(folder,'sliding_input.mat'));
 copyfile(fullfile(root,'datasets','s_channel_two_contact.mat'),fullfile(folder,'two_contact_input.mat'));
elseif strcmp(mode,'regenerate')
 scenes=contact_demo_scenes();scene=scenes(strcmp({scenes.id},'sliding_clean'));assert(isscalar(scene));
 [sensorInput,truth]=build_contact_demo_truth(scene);save(fullfile(folder,'sliding_input.mat'),'sensorInput','truth','scene','-v7.3');
 scenes=multi_contact_demo_scenes();scene=scenes(strcmp({scenes.id},'s_channel_two_contact'));assert(isscalar(scene));
 [truth,scene,sensorInput]=build_multi_contact_demo_truth(scene);save(fullfile(folder,'two_contact_input.mat'),'sensorInput','truth','scene','-v7.3');
else,error('tsfs:InputMode','Choose recorded or regenerate.');end
end
