function pcost=pathcost(frontier,x,y)
n=size(frontier(1,:));
pcost=zeros(1,n);
 for j=1:n % replace cell within d as 0
 pcost(j)=abs(frontier(1,j)-x)+abs(frontier(2,j)-y);
 end
end

