function hash = file_sha256(path)
% Stream artifacts instead of duplicating a large MAT file in RAM.
fid=fopen(path,'rb'); assert(fid>=0,'rod:ArtifactRead','Cannot read %s.',path);
closer=onCleanup(@()fclose(fid)); digest=java.security.MessageDigest.getInstance('SHA-256');
while true
    bytes=fread(fid,1024*1024,'*uint8');
    if isempty(bytes),break;end
    digest.update(typecast(bytes,'int8'));
end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
