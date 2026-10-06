function audit_fbg_integration
[root,paths]=setup_tsfs;folder=fullfile(paths.results,'fbg_integration');if ~isfolder(folder),mkdir(folder);end
names={'sliding_clean','s_channel_two_contact'};methods={'current_pchip','direct_left','direct_average','direct_centered'};rows=[];curves=cell(2,4);
for a=1:2
 data=load(fullfile(root,'datasets',[names{a} '.mat']));frames=tsfs.read_input(data.sensorInput);truth=data.truth;
 for k=1:numel(frames)
  fr=frames{k};s=fr.tube.s;
  for b=1:numel(methods)
   clock=tic;
   if b==1,sh=tsfs.reconstruct(fr,false);p=sh.p;t=sh.t;else,[p,t,edges,U]=direct(fr,s,methods{b});end
   seconds=toc(clock);err=vecnorm(p-truth.p(:,:,k));tt=[];if isfield(truth,'R'),tt=reshape(truth.R(:,3,:,k),3,[]);end
   row=struct('scenario',names{a},'frame',k,'method',methods{b},'positionRmsMm',sqrt(mean(err.^2)),'maximumPositionErrorMm',max(err),'tipPositionErrorMm',err(end),'seconds',seconds,'segments',numel(s)-1,'tangentRmsRad',NaN);
   if ~isempty(tt),angles=atan2(vecnorm(cross(t,tt,1)),sum(t.*tt,1));row.tangentRmsRad=sqrt(mean(angles.^2));end
   if b>1,row.segments=numel(edges)-1;end
   rows=[rows;row];if k==1,curves{a,b}=struct('s',s,'p',p,'t',t);end
  end
 end
end
writetable(struct2table(rows),fullfile(folder,'frames.csv'));save(fullfile(folder,'reconstructions.mat'),'curves','rows','methods','-v7.3');disp(struct2table(rows(1:4,:)));
end
function [p,t,edges,U]=direct(fr,query,method)
s=fr.tube.s;u0=fr.tube.uhat;intr=interp1(s,u0',fr.arcs(:),'previous')';elastic=zeros(3,numel(fr.arcs));elastic(fr.axes,:)=fr.u-intr(fr.axes,:);
changes=s([false any(abs(diff(u0,1,2))>1e-14,1)]);
if strcmp(method,'direct_centered'),cuts=(fr.arcs(1:end-1)+fr.arcs(2:end))/2;else,cuts=fr.arcs;end
edges=unique([s(1) cuts changes s(end)]);U=zeros(3,numel(edges)-1);
for j=1:numel(edges)-1
 mid=mean(edges(j:j+1));u=interp1(s,u0',mid,'previous')';
 if strcmp(method,'direct_centered'),[~,idx]=min(abs(fr.arcs-mid));e=elastic(:,idx);
 elseif mid<fr.arcs(1),e=elastic(:,1);
 elseif mid>fr.arcs(end),e=elastic(:,end);
 else
  idx=find(fr.arcs<=mid,1,'last');idx=min(idx,numel(fr.arcs)-1);e=elastic(:,idx);
  if strcmp(method,'direct_average'),e=(elastic(:,idx)+elastic(:,idx+1))/2;end
 end
 U(:,j)=u+e;
end
R=fr.base(1:3,1:3);base=fr.base(1:3,4);p=zeros(3,numel(query));t=p;
for j=1:size(U,2)
 mask=find(query>=edges(j)&query<=edges(j+1));
 for idx=mask,[p(:,idx),Rq]=advance(base,R,U(:,j),query(idx)-edges(j));t(:,idx)=Rq(:,3);end
 [base,R]=advance(base,R,U(:,j),edges(j+1)-edges(j));
end
end
function [pnext,Rnext]=advance(p,R,u,ds)
w=u*ds;angle=norm(w);W=tsfs.hat(w);
if angle<1e-5,A=1-angle^2/6;B=.5-angle^2/24;C=1/6-angle^2/120;else,A=sin(angle)/angle;B=(1-cos(angle))/angle^2;C=(angle-sin(angle))/angle^3;end
pnext=p+R*(eye(3)+B*W+C*W^2)*[0;0;ds];Rnext=R*(eye(3)+A*W+B*W^2);
end
