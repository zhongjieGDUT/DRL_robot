function [cd1,cd2,dx1,dy1,dx2,dy2,dx3,dy3,dx,dy]= get_state(th1,th2,th3,x,y)
%GET_STATE 得到状态
%   
a=plannar3();
goal=[x,y]';
th=[th1,th2,th3];

xy123=a.get_dist(th);

center_dis=(a.base'-goal)/300; %基座和期望点的坐标差
cd1=center_dis(1);cd2=center_dis(2);

t_arms1 = (xy123(:,1)-goal)/300;
dx1=t_arms1(1);dy1=t_arms1(2);

t_arms2 = (xy123(:,2)-goal)/300;
dx2=t_arms2(1);dy2=t_arms2(2);

t_arms3 = (xy123(:,3)-goal)/300;
dx3=t_arms3(1);dy3=t_arms3(2);

dx=xy123(1,3)-x;
dy=xy123(2,3)-y;


end

