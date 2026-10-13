function m=bishop_baseline_measurements(input)
% Observation-only adapter for unchanged Tongyu Aloi; identical sparse geometry.
frames=tsfs.read_input(input);p=input.packet;idx=p.fbgIdx;N=numel(input.tube.s);T=numel(frames);
m=struct('betaMm',p.actuationMm,'baseTraj',p.basePose,'fbgIdx',idx,'pSparse',zeros(3,numel(idx),T),'p',zeros(3,N,T));
for k=1:T
 assert(tsfs.is_bishop(frames{k}));sh=tsfs.reconstruct(frames{k},false);
 m.p(:,:,k)=sh.p;m.pSparse(:,:,k)=sh.p(:,idx);
end
end
