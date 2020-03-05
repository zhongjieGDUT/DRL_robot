clear;
clc
close all;
deg=180.0/pi;
rad=pi/180.0;
mdl_puma560;
p560.qlim=[-160,160;
    -45,225;
    -225,45;
    -110,170;
    -100,100;
    -266,266]*rad;
p560.plot3d(qz);close all;
%ÆÚÍûÄ©¶ËÎ»×Ë
p560.plot(qz);
T0=p560.fkine(qn);hold on;
T0=se2t(T0);plottcp(se2t(p560.fkine(qz)),'start',0.2);
T0=T0*transl(-0.2,0.2,0.1)*trotz(pi/6)*troty(pi/6);
plottcp(T0,'goal',0.2);
T0=T2e(T0);




