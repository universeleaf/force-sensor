function B=basis(n)
[~,j]=min(abs(n)); e=zeros(3,1);e(j)=1; b=cross(n,e); b=b/norm(b);B=[b cross(n,b)];
end
