function outputs=estimate(frames,o)
% One geometry/mechanics/diagnostic pass per packet; keep every output frame.
if nargin<2,o=tsfs.defaults;end
outputs=cell(size(frames));previous=[];
for k=1:numel(frames)
 clock=tic;
 try
  outputs{k}=tsfs.step(frames{k},previous,o);previous=outputs{k};
 catch err
  outputs{k}=struct('valid',false,'time',frames{k}.time,'seconds',toc(clock), ...
   'errorIdentifier',err.identifier,'error',getReport(err,'extended','hyperlinks','off'));
 end
end
end
