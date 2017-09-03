function unknown= informationgain(A,frontier,pu,D)
unknown=[];
n=size(frontier(1,:));
for i=1:n
r=frontier(1,i);
c=frontier(2,i);
infor=0;
for j=1:2*D
   for jj=1:2*D
       d=sqrt((r-D+j)^2+(c-D+jj)^2);
   if d<=D&&A(r-D+j,c-D+jj)==pu
       infor=infor+1;
   end
   end
end
unknown=[unknown infor];
end
end

