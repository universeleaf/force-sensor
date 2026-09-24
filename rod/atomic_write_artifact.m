function atomic_write_artifact(path,kind,value)
% Write beside the destination, close, then rename. Never publish partial data.
folder=fileparts(path); if isempty(folder),folder=pwd;end
assert(isfolder(folder),'rod:ArtifactDirectory','Output directory does not exist.');
temporary=[tempname(folder) '.tmp']; cleanup=onCleanup(@()removeTemporary(temporary));
switch kind
    case 'json'
        fid=fopen(temporary,'w','n','UTF-8');
        assert(fid>=0,'rod:ArtifactWrite','Cannot open temporary artifact.');
        closeFile=onCleanup(@()fclose(fid));
        count=fprintf(fid,'%s',jsonencode(value,'PrettyPrint',true));
        assert(count>0,'rod:ArtifactWrite','Empty artifact write.');
        [message,number]=ferror(fid);
        assert(number==0,'rod:ArtifactWrite','Artifact write failed: %s',message);
        clear closeFile;
    case 'mat'
        assert(isstruct(value)&&isscalar(value),'MAT payload must be a scalar variable struct.');
        save(temporary,'-struct','value','-v7.3');
    otherwise
        error('rod:ArtifactWrite','Use json or mat.');
end
[ok,message]=movefile(temporary,path,'f');
assert(ok,'rod:ArtifactWrite','Cannot publish artifact: %s',message);
end
function removeTemporary(path)
if isfile(path),delete(path);end
end
