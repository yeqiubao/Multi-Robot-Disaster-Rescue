function [B,frontier]= definefrontier(d,A,pf,pc,pu)
B=A;
C=ones(size(A));
frontier=[];
for j=2:size(A,1)% start from second raw,assume first raw is obstacle
    for jj=2:size(A,2)% start from second column, assume first column is obstacle
    if B(j,jj)==pc&&(B(j-1,jj)==pu||B(j+1,jj)==pu||B(j,jj-1)==pu||B(j,jj+1)==pu)&&C(j,jj)~=0
        B(j,jj)=pf;%denote frontier
        frontier=[frontier [j;jj]];
        for i=1:2*d % replace cell within d as 0
         for ii=1:2*d
            C(j-d-1+i,jj-d-1+ii)=0;
         end
        end
    end
    end
end

end

