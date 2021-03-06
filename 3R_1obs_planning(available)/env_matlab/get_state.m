function [cd1,cd2,dx1,dy1,dx2,dy2,dx3,dy3,dx,dy,dox1,doy1,...
    dox2,doy2,dox3,doy3,dxo1,dyo1,dxo2,dyo2,dxo3,dyo3]= get_state(th1,th2,th3,x,y,obs)
%GET_STATE 得到状态

obs=cell2mat(obs);
obsc=obs(1:2);
r=obs(3);
a=plannar3();
goal=[x,y]';
th=[th1,th2,th3];

xy123=a.get_dist(th);

%% 归一化的目标点与机器人坐标差
% 每一个关节与目标点的差，归一化处理
center_dis=(a.base'-goal)/300;       %基座和期望点的坐标差
cd1=center_dis(1);cd2=center_dis(2);

t_arms1 = (xy123(:,1)-goal)/300;     %关节一
dx1=t_arms1(1);dy1=t_arms1(2);

t_arms2 = (xy123(:,2)-goal)/300;    %关节二
dx2=t_arms2(1);dy2=t_arms2(2);

t_arms3 = (xy123(:,3)-goal)/300;   %关节三
dx3=t_arms3(1);dy3=t_arms3(2);
%% 没有归一化的末端和目标点的坐标差
dx=xy123(1,3)-x;
dy=xy123(2,3)-y;
%% 归一化的机器人与障碍的坐标差
t_arms1 = (xy123(:,1)-obsc')/300;     %关节一
dox1=t_arms1(1);doy1=t_arms1(2);


t_arms2 = (xy123(:,2)-obsc')/300;     %关节二
dox2=t_arms2(1);doy2=t_arms2(2);

t_arms3 = (xy123(:,3)-obsc')/300;    %关节三
dox3=t_arms3(1);doy3=t_arms3(2);
%% 没有归一化的机器人关节（2和3）和障碍的距离
dxo1=xy123(1,1)-obs(1);
dyo1=xy123(2,1)-obs(2);

dxo2=xy123(1,2)-obs(1);
dyo2=xy123(2,2)-obs(2);

dxo3=xy123(1,3)-obs(1);
dyo3=xy123(2,3)-obs(2);
end

