clear all;
clc
close all;
ToDeg=180.0/pi;
ToRad=pi/180.0;
%% 机器人初始关节角【-38.4 0 -48.6 -11.2 -54 10.6】
q0=[-38.4 0 -48.6 -11.2 -54 10.6]*ToRad;
%% 机器人的期望关节角【32 -2.7 -29.7 0 -36 0】
qf=[60.8 8.1 -29.7 -5.6 -28 0]*ToRad;
%% 生成机器人的碰撞模型
% load('D:\pHRIWARE-master\next\Help\Demos\p560_col.mat')
% robot=p560;
mdl_puma560;
p560.qlim=[-160,160;
    -45,225;
    -225,45;
    -110,170;
    -100,100;
    -266,266]/180*pi;
% p560.points=robot.points;
figure(1);
p560.plot3d(q0);T0=p560.fkine(q0);T0=se2t(T0);
hold on;
T=p560.fkine(qn); T=se2t(T);
%% 生成圆柱、球形、方形障碍模型
% sphere = Sphere(T*transl(0,0,0.4), 0.15);     %球形
% sphere_collision = CollisionModel(sphere);
% sphere.plot();axis equal;
% box = Box(T*transl(-0.1,0.5,0),[0.15 0.15 0.15]); %方形
% box_collision = CollisionModel(box);
% box.plot();axis equal;
cylinder=Cylinder(T*troty(-pi/2)*transl(0.3,0,-0.5),[0.2,0.2,1.5]);                                              %圆柱
cylinder_collision=CollisionModel(cylinder);
cylinder.plot();%axis equal;
% set(gca,'Position',[0.1 0.1 0.9 0.9],'outerPosition',[0 0 1 1]);
p560.collisions(q0,cylinder_collision)
cylinder_xyz=transl(T*troty(-pi/2)*transl(0.3,0,-0.5));
r=0.2;
%% 期望的末端位姿
Te=p560.fkine(qf);Te=se2t(Te);
Te_xyz=transl(Te)
Te_zyz=tr2eul(Te)
%% 绘图
close all;
p560.plot3d(q0,'workspace',[-1 1.2 -1 1 -1.2 1.1]);hold on;
cylinder.plot();
plottcp(Te,'target',0.2);
plottcp(T0,'start',0.2);