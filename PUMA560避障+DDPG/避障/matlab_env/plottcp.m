function plottcp(H,l,s)
% HT/PLOT Command figure plot of a HT

    if nargin < 3
        s = 0.1;
    end
    
    if nargin < 2
        l = '';
    end
    
    vx = H(1:3,1) * s;
    vy = H(1:3,2) * s;
    vz = H(1:3,3) * s;
    p = H(1:3,4);

    quiver3(p(1), p(2), p(3), vx(1), vx(2), vx(3),0,'r','Linewidth',2);
    
%     prevhold = ishold;
% 
%     if ~prevhold
%         hold on;
%     end

    b=quiver3(p(1), p(2), p(3), vy(1), vy(2), vy(3),0,'g','Linewidth',2);
    b=quiver3(p(1), p(2), p(3), vz(1), vz(2), vz(3),0,'b','Linewidth',2);
    b=text(p(1),p(2)-s/10,p(3)-s/10,l);

%     if ~prevhold
%         hold off;
%     end
    
