function [theta,T2,T3,T4,T5,T6,dT2,dT3,dT4,dT5,dT6,dT6_] = state(q,goal)
p560=evalin('base','p560');
theta=cell2mat(q);
goal=cell2mat(goal);
qlim=p560.qlim;
%% 限幅
%%joint1
if theta(1)<=qlim(1,1)
    theta(1)=qlim(1,1);
else if theta(1)>=qlim(1,2)
        theta(1)=qlim(1,2);
    end
end
%%joint2
if theta(2)<=qlim(2,1)
    theta(2)=qlim(2,1);
else if theta(2)>=qlim(2,2)
        theta(2)=qlim(2,2);
    end
end
%%joint3
if theta(3)<=qlim(3,1)
    theta(3)=qlim(3,1);
else if theta(3)>=qlim(3,2)
        theta(3)=qlim(3,2);
    end
end
%%joint4
if theta(4)<=qlim(4,1)
    theta(4)=qlim(4,1);
else if theta(4)>=qlim(4,2)
        theta(4)=qlim(4,2);
    end
end
%%joint5
if theta(5)<=qlim(5,1)
    theta(5)=qlim(5,1);
else if theta(5)>=qlim(5,2)
        theta(5)=qlim(5,2);
    end
end
%%joint6
if theta(6)<=qlim(6,1)
    theta(6)=qlim(6,1);
else if theta(6)>=qlim(6,2)
        theta(6)=qlim(6,2);
    end
end
%% 获取状态
A2=p560.A(1:2,theta);
A3=p560.A(1:3,theta);
A4=p560.A(1:4,theta);
A5=p560.A(1:5,theta);
A6=p560.A(1:6,theta);

T2=T2e(se2t(A2));
T3=T2e(se2t(A3));
T4=T2e(se2t(A4));
T5=T2e(se2t(A5));
T6=T2e(se2t(A6));

dT2=T2-goal;
dT3=T3-goal;
dT4=T4-goal;
dT5=T5-goal;
dT6=T6-goal;dT6_=dT6;
%%归一化
T2(1:3)=T2(1:3)/3;T2(4:6)=T2(4:6)/pi;
T3(1:3)=T3(1:3)/3;T3(4:6)=T3(4:6)/pi;
T4(1:3)=T4(1:3)/3;T4(4:6)=T4(4:6)/pi;
T5(1:3)=T5(1:3)/3;T5(4:6)=T5(4:6)/pi;
T6(1:3)=T6(1:3)/3;T6(4:6)=T6(4:6)/pi;

dT2(1:3)=dT2(1:3)/3;dT2(4:6)=dT2(4:6)/pi;
dT3(1:3)=dT3(1:3)/3;dT3(4:6)=dT3(4:6)/pi;
dT4(1:3)=dT4(1:3)/3;dT4(4:6)=dT4(4:6)/pi;
dT5(1:3)=dT5(1:3)/3;dT5(4:6)=dT5(4:6)/pi;
dT6(1:3)=dT6(1:3)/3;dT6(4:6)=dT6(4:6)/pi;


end

