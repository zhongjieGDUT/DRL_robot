function draw
close all;
base=[600,300];
a=plannar3();
b=plannar3(base);
clf=true;
h=hggroup;g=hggroup;
axis([0,600,0,600]);
% a.plot([0,0,0],false);
b.plot([-pi,0,0],0);
for i=-pi/2:pi/100:pi/2
    if i==pi
        clf=false;
    end
    a.plot([i,0,0],clf);
    xy=a.fk([i,0,0]);
    plot(xy(1),xy(2),'r*','parent',h);
%     axis equal;
end
a.plot([pi/2,0,0],false);
