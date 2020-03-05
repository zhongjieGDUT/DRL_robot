function [x1,y1,x2,y2,x3,y3,danger,goal,d1x,d1y,d2x,d2y,d3x,d3y] = get_status(th1,th2,th3,x,y)
%GET_STATUS 
%  输入角度和绘图标志位
%  输出末端位置x,y，
%  是否进入危险区域机械臂之间相撞：danger
%  
%  是否到达目的地
a=plannar3();
th=[th1,th2,th3];
xy=a.get_dist(th);%得到每个关节的坐标
x1=xy(1,1);x2=xy(1,2);x3=xy(1,3);
y1=xy(2,1);y2=xy(2,2);y3=xy(2,3);

if x3>=330&&y3<=310&&y3>=290 
    danger=false;
else if x3<100||y3<280
        danger=false;
     else
        danger=false;
     end
end

d=norm(xy(:,3)'-[x,y]);
if d<5
    goal=true;
else
    goal=false;
end
d1x=x-xy(1,1);
d2x=x-xy(1,2);
d3x=x-xy(1,3);
d1y=y-xy(2,1);
d2y=y-xy(2,2);
d3y=y-xy(2,3);





    
end

