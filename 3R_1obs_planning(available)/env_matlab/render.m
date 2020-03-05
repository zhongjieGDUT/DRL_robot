function render(th1,th2,th3,x,y,xx,yy,r)
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
plot_circle([xx,yy],r);
% axis equal;
% hold on;
% xygoal=[300,180];
plot(x,y,'r*','MarkerSize',20);
axis equal;
end

