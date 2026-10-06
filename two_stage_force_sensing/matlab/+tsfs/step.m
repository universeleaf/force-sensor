function out=step(fr,previous,o)
if nargin<2,previous=[];end
if nargin<3,o=tsfs.defaults;end
clock=tic;g=tsfs.geometry(fr,o);sol=tsfs.sqp(fr,g,previous,o);d=tsfs.diagnose(fr,g,sol,o);
out=struct('valid',true,'time',fr.time,'geometry',g,'mechanics',sol,'diagnostics',d, ...
 'contactForce',sol.evaluation.shape.force,'tipForce',sol.evaluation.shape.x(4:6),'seconds',toc(clock));
end
