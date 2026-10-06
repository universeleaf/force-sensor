function [p,t,Jp,Jt]=shape_at(sh,s)
if isempty(s),p=zeros(3,0);t=p;Jp=zeros(3,0,size(sh.Jp,3));Jt=Jp;return,end
p=interp1(sh.s,sh.p',s,'pchip')';t=interp1(sh.s,sh.t',s,'pchip')';
t=t./vecnorm(t);Jp=[];Jt=[];
if ~isempty(sh.Jp)
 nu=size(sh.Jp,3);Jp=zeros(3,numel(s),nu);Jt=Jp;
 A=reshape(permute(sh.Jp,[2 1 3]),numel(sh.s),[]);D=interp1(sh.s,A,s,'pchip');
 Jp=permute(reshape(D,numel(s),3,nu),[2 1 3]);
 A=reshape(permute(sh.Jt,[2 1 3]),numel(sh.s),[]);D=interp1(sh.s,A,s,'pchip');
 Jt=permute(reshape(D,numel(s),3,nu),[2 1 3]);
end
end
