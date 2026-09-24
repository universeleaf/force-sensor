function test_reduced_mode_bounds()
% A dependent sliding beta must still respect its user-configured upper bound.
m=2; nx=11+m; x=zeros(nx,1); x(6)=1; x(7)=0.1; x(8)=0.05;
lo=-inf(nx,1); hi=inf(nx,1); lo(7:8+m)=0; hi(8)=0.1;
cfg=struct('numFrictionDirs',m,'currentMu',0.5,'linearizedSolveMaxIter',50);
mode=struct('name','sliding-contact','activeDirection',1);
[answer,info]=solve_contact_mode_map(x,@(v)(v(7)-1)^2,@decode,{lo,hi},ones(nx,1),cfg,mode);
assert(info.exitflag>0 && abs(answer(7)-0.2)<1e-5 && answer(8)<=0.1+1e-7, ...
    'Mode reduction discarded a dependent-coordinate force bound.');
cfg.useMultiStart=true; rejected=false;
try
    solve_contact_mode_map(x,@(v)(v(7)-1)^2,@decode,{lo,hi},ones(nx,1),cfg,mode);
catch info
    rejected=strcmp(info.identifier,'rod:UnsupportedReducedMultiStart');
end
assert(rejected,'Reduced solver silently ignored the requested multiple starts.');
% Two active friction generators describe an edge of the polyhedral cone.
% The eliminated second coefficient must retain BOTH bounds. Closed-form
% optimum with beta2<=.2 is fn=.98, beta1=.29, beta2=.2.
m4=4; y=zeros(15,1); y(7)=1;y(8)=.3;y(9)=.2;y(12)=1;
lower=-inf(15,1);upper=inf(15,1);lower(7:12)=0;upper(9)=.2;
face=struct('name','sliding-contact','activeDirection',[1 2]);
settings=struct('numFrictionDirs',4,'currentMu',.5,'linearizedSolveMaxIter',80, ...
    'complementaritySolver','scholtes');
[answer,info]=solve_contact_mode_map(y,@(v)(v(7)-1)^2+(v(8)-.25)^2+(v(9)-.25)^2, ...
    @faceDecode,{lower,upper},ones(15,1),settings,face);
assert(info.exitflag>0&&norm(answer(7:9)-[.98;.29;.2])<1e-5, ...
    'Two-generator sliding face lost a bound or force-balance degree of freedom.');
% For a separated rod the cone is zero and lambda is not fixed by
% complementarity. The optimal feasible MAP value is 4, not min work 1.
face.name='no-contact';face.activeDirection=[];y(7:11)=0;y(12)=1;
[answer,info]=solve_contact_mode_map(y,@(v)(v(12)-4)^2,@faceDecode, ...
    {lower,upper},ones(15,1),settings,face);
assert(info.exitflag>0&&abs(answer(12)-4)<1e-5,'Free friction dual was incorrectly fixed to minimum work.');
disp('Reduced-mode dependent-coordinate bounds passed.');
    function d=faceDecode(v)
        gap=0;if v(7)==0,gap=1;end
        d=struct('n',[0;0;1],'D',[1 0 -1 0;0 1 0 -1;0 0 0 0], ...
            'vTangential',[-1;-1;0],'gap',gap,'frictionW',[-1;-1;1;1]+v(12), ...
            'frictionConeSlack',.5*v(7)-sum(v(8:11)));
    end
    function d=decode(v)
        d=struct('n',[0;0;1],'D',[1 -1;0 0;0 0],'vTangential',[-1;0;0], ...
            'gap',0,'frictionW',[-1;1]+v(8+m),'frictionConeSlack',0.5*v(7)-sum(v(8:9)));
    end
end
