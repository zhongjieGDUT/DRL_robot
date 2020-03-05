function T = se2t(s)
%SE2T 此处显示有关此函数的摘要
%   此处显示详细说明
T=eye(4);
T(1:3,1)=s.n;
T(1:3,2)=s.o;
T(1:3,3)=s.a;
T(1:3,4)=s.t;





end

