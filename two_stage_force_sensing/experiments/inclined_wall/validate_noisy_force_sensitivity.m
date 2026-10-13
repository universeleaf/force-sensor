function validate_noisy_force_sensitivity
% Finite differences of the exact nonlinear IVP; no inverse solves/settings changed.
[root,~]=setup_tsfs;pub=fullfile(root,'results','published','noisy_force_sensitivity_20261012');d=jsondecode(fileread(fullfile(pub,'derivative_inputs.json')));rows=[];
for j=1:numel(d)
 a=d(j);sc=a.scenario;
 if strcmp(sc,'insertion'),p=fullfile(root,'results','runs','inclined_wall_bidirectional_20261008','inclined_wall','truth.mat');else,p=fullfile(root,'results','runs','inclined_wall_rotation_20261010','truth.mat');end
 t=load(p,'input');frames=tsfs.read_input(t.input);fr=frames{a.frame};s=load(fullfile(root,'results','runs',['inclined_' sc '_linearized_20261011_v2'],'comparison.mat'),'runs','o');out=s.runs{1,a.frame};m=out.mechanics;
 for h=[1e-2 1e-3]
  sp=tsfs.ivp(fr,out.geometry,m.z+h*a.dz,m.scale,s.o,false);sm=tsfs.ivp(fr,out.geometry,m.z-h*a.dz,m.scale,s.o,false);fd=(sp.prediction(:)-sm.prediction(:))/(2*h);
  err=norm(fd-a.Jdz)/norm(a.Jdz);rows=[rows;struct('scenario',sc,'frame',a.frame,'h',h,'relativeError',err)];
 end
end
writetable(struct2table(rows),fullfile(pub,'derivative_checks.csv'));disp(struct2table(rows));assert(max([rows.relativeError])<1e-3);
end
