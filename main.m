clc;
clear all
close all

D=10;%detection range for robot
po=1;% denote obstacle as 1 in map
pc=0;% denote clear space as 0 in map
pu=0.5;% denote unknown spave as 2 in map
pf=2;% denote frontier as 3 in map
d=5;% minimum distance from all other frontiers

x1=a1;y1=b1;%current location for robots
x2=a2;y2=b2;
x3=a3;y3=b3;

Aupdate=zeros(10,10);%combine all single maps from robots
[A,frontier]=definefrontier(d,Aupdate,pf,pc,pu); %find frontier and denote in new map
n=size(frontier(1,:));
%do while(n>0)
if n==0
    %mapping finished
end

%calculate unknown area,path cost for all frontiers
[cost1,inforG1]=cooperation(x1,y1,A,frontier);
[cost2,inforG2]=cooperation(x3,y3,A,frontier);
[cost3,inforG3]=cooperation(x3,y3,A,frontier);
%reorder the frontier from large to small based on utility
[f1,cost_1,inforG_1,ut1] = reorder(frontier,cost1,inforG1);
[f2,cost_2,inforG_2,ut2] = reorder(frontier,cost2,inforG2);
[f3,cost_3,inforG31,ut3] = reorder(frontier,cost3,inforG3);

m=3;

   if f1(:,1)-f2(:,1)~=0&&f2(:,1)-f3(:,1)~=0
       % go to first frontier
   elseif f1(:,1)-f2(:,1)==0&&f2(:,1)-f3(:,1)~=0
       %robot3 go to the frontier1
     if n==2
         if ut1(1)>ut2(1)
          %robot1 go to the frontier1, robot2 stop
         else
          %robot2 go to the frontier1, robot1 stop
         end
     elseif n~=2
      if ut1(1)>ut2(1)
          %robot1 go to the frontier1
          if f2(:,2)==f3(:,1)
              %robot2 go to the frontier3
          else
              %robot2 go to the frontier2
          end
      else
          %robot2 go to the frontier1
          if f1(:,2)==f3(:,1)
              %robot1 go to the frontier3
          else
              %robot1 go to the frontier2
          end
      end
     end
   elseif f1(:,1)-f3(:,1)==0&&f2(:,1)-f3(:,1)~=0
       %robot2 go to the frontier1
     if n==2
         if ut1(1)>ut3(1)
          %robot1 go to the frontier1, robot3 stop
         else
          %robot3 go to the frontier1, robot1 stop
         end
     elseif n~=2
      if ut1(1)>ut3(1)
          %robot1 go to the frontier1
          if f3(:,2)==f2(:,1)
              %robot3 go to the frontier3
          else
              %robot3 go to the frontier2
          end
      else
          %robot3 go to the frontier1
          if f1(:,2)==f2(:,1)
              %robot1 go to the frontier3
          else
              %robot1 go to the frontier2
          end
      end
     end
    elseif f2(:,1)-f3(:,1)==0&&f1(:,1)-f3(:,1)~=0
       %robot1 go to the frontier1
     if n==2
         if ut3(1)>ut2(1)
          %robot3 go to the frontier1, robot2 stop
         else
          %robot2 go to the frontier1, robot3 stop
         end
     elseif n~=2
      if ut2(1)>ut3(1)
          %robot2 go to the frontier1
          if f3(:,2)==f1(:,1)
              %robot3 go to the frontier3
          else
              %robot3 go to the frontier2
          end
      else
          %robot3 go to the frontier1
          if f2(:,2)==f1(:,1)
              %robot2 go to the frontier3
          else
              %robot2 go to the frontier2
          end
      end
     end
   elseif f1(:,1)-f2(:,1)==0&&f2(:,1)-f3(:,1)==0
      if n==1
          if ut1(1)>=ut2(1)&&ut1(1)>=ut3(1)
          %robot1 go to frontier1, others stop
          elseif ut2(1)>=ut1(1)&&ut2(1)>=ut3(1)
          %robot2 go to frontier1, others stop
          elseif ut3(1)>=ut1(1)&&ut3(1)>=ut2(1)
          %robot3 go to frontier1, others stop
          end
      elseif n~=1    
       if ut1(1)>=ut2(1)&&ut1(1)>=ut3(1)
           %robot1 go to the frontier1
           if f2(:,2)-f3(:,2)~=0
               %robot2 and robot3 go to frontier2
           else
               if ut2(2)>=ut3(2)
                   %robot2 go to frontier 2,robot3 go to frontier 3
               elseif ut2(2)<ut3(2)
                   %robot3 go to frontier2,robot2 go to frontier 3
               end
           end
       elseif ut2(1)>=ut1(1)&&ut2(1)>=ut3(1)
            %robot2 go to the frontier1
           if f1(:,2)-f3(:,2)~=0
               %robot1 and robot3 go to frontier2
           else
               if ut1(2)>=ut3(2)
                    %robot1 go to frontier 2,robot3 go to frontier 3
               elseif ut1(2)<ut3(2)
                   %robot3 go to frontier2,robot1 go to frontier 3
               end
           end
       elseif ut3(1)>=ut1(1)&&ut3(1)>=ut2(1)
            %robot3 go to the frontier1
           if f2(:,2)-f1(:,2)~=0
               %robot2 and robot1 go to frontier2
           else
               if ut2(2)>=ut1(2)
                    %robot2 go to frontier 2,robot1 go to frontier 3
               elseif ut2(2)<ut1(2)
                   %robot go to frontier2,robot2 go to frontier 3
               end
           end
       end
      end
   end
