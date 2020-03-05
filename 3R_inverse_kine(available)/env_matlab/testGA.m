clear all;close all;
f=apf();

figure;
a=plannar3();
base=[600,300];

b=plannar3(base);
axis([0,600,0,600]);
b.plot([pi,0,0],0);
th0=[pi/2,0,0];
a.plot(th0,0);  
xy=a.fk(th0);
xygoal=[300,180]
plot(xygoal(1),xygoal(2),'r*','MarkerSize',20);
th=th0;

while true
    %��һ��У���������Ŵ��㷨���ޱ��ϣ�
    dth1=a.GAdth(xygoal(1),xygoal(2),th);
    th=th+dth1;
  
    %�ڶ���У��������ģ������+�˹��Ƴ����� 
    dth2=a.fuzzydth(xygoal(1),xygoal(2),th,f);
    th=th+dth2;
    dth2*180.0/pi;
    a.plot(th,0);
    xy=a.fk(th);
    norm(xygoal-a.fk(th));
    if norm(xygoal-a.fk(th))<=1
        break; 
    end
end
a.plot(th,0)
th=a.fk(th)
    
    