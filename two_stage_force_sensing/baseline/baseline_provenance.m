function info=baseline_provenance(baselineRoot,lcpRoot)
% Record actual source hashes, including local modifications, without writing there.
info=struct('baselineRoot',baselineRoot,'lcpRoot',lcpRoot,'files',struct([]));
roots={fullfile(baselineRoot,'rod'),lcpRoot};labels={'rod','LCP-Continuum'};rows=[];
for j=1:numel(roots)
 files=dir(fullfile(roots{j},'**','*.m'));
 for k=1:numel(files)
  file=fullfile(files(k).folder,files(k).name);fid=fopen(file,'rb');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
  md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);digest=typecast(md.digest(),'uint8');
  hash=lower(reshape(dec2hex(digest,2)',1,[]));relative=file(numel(roots{j})+2:end);
  rows=[rows;struct('path',[labels{j} '/' strrep(relative,'\','/')],'sha256',hash)];
 end
end
info.files=rows;
end
