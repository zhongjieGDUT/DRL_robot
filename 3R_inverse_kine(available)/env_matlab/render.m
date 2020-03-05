function render(th1,th2,th3,x,y)
%RENDER 
%
% clf;
th0=[th1,th2,th3];
a=plannar3();
% base=[600,300];
% b=plannar3(base);
%  axis([0 600 0 600]);
% 
% b.plot([pi,0,0],0);
a.plot(th0,0);
% pause(0.2);
% hold on;
% xygoal=[300,180];
plot(x,y,'r*','MarkerSize',20);
% axis equal;
end

