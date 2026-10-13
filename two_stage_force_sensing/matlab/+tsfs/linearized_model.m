function model=linearized_model(fr,g,z,scale,prior,previous,o)
% Physical previous-frame history remains fixed during all outer iterations.
history=[];
if ~isempty(fr.previous)
 if ~isempty(previous)&&previous.valid&&abs(previous.time-fr.previous.time)<1e-9
  history=previous.mechanics.evaluation.shape;history.t=reshape(history.R(:,3,:),3,[]);history.Jp=[];history.Jt=[];
 else,history=tsfs.reconstruct(fr.previous,false);end
end
e=tsfs.evaluate(fr,g,z,scale,prior,history,o,true);N=numel(fr.u);e.r=e.r(1:N);e.J=e.J(1:N,:);e.objective=.5*(e.r'*e.r);
model=struct('z',z,'e',e,'history',history);
end
