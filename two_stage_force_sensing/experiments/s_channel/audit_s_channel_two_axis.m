function audit_s_channel_two_axis
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'s_channel_investigation');a=load(fullfile(root,'datasets','s_channel_two_contact.mat'));t=a.truth;frames=tsfs.read_input(a.sensorInput);o=tsfs.defaults;fr=frames{1};g=tsfs.geometry(fr,o);
% Explicit additional-information control. Add the actual synthetic zero
% first bending channel, without constraining the generic solver to 2-D.
fr.axes=[1 2];fr.u=[zeros(size(fr.u));fr.u];fr.CU=eye(numel(fr.u))*1e-14;rows=[];
for mask=0:7
 gg=g;if bitget(mask,1),gg.s=t.contactS(:,1)';end
 if bitget(mask,2),gg.normal=t.planeNormal;end
 if bitget(mask,3),gg.offset=zeros(size(g.offset));end
 for i=1:2,gg.B(:,:,i)=tsfs.basis(gg.normal(:,gg.plane(i)));end
 gg.predictedPoint=tsfs.shape_at(gg.kinematicShape,gg.s);
 sol=tsfs.sqp(fr,gg,[],o);d=tsfs.diagnose(fr,gg,sol,o);sh=sol.evaluation.shape;
 row=struct('mask',mask,'correctArc',logical(bitget(mask,1)),'correctNormal',logical(bitget(mask,2)),'correctOffset',logical(bitget(mask,3)), ...
 'seconds',sol.seconds,'iterations',sol.iterations,'converged',sol.converged,'contactErrorN',norm(sh.force-t.contactForces(:,:,1),'fro'), ...
 'tipErrorN',norm(sh.x(4:6)-t.tipForce(:,1)),'tipX',sh.x(4),'tipY',sh.x(5),'tipZ',sh.x(6), ...
 'fbgRms',d.fbgRms,'penetrationMm',d.maximumPenetrationMm,'projectionN',max(abs(d.projectionN),[],'all'),'flags',strjoin(d.flags,';'));rows=[rows;row];disp(row);
end
writetable(struct2table(rows),fullfile(folder,'two_axis_factorial_control.csv'));
end
