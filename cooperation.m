function [inforG,cost]=cooperation(x,y,Aupdate,frontier)
x1=x;%current location for x1
y1=y;
D=10;%detection range for robot
A=Aupdate;% update map
pu=0.5;% denote unknown spave as 2 in map

%calculate path cost for robot x1
cost=pathcost(frontier,x1,y1);
%calculate information gain for all fonrtiers
inforG=informationgain(A,frontier,pu,D);
end