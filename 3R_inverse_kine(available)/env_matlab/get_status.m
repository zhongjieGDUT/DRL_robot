function [x1,y1,x2,y2,x3,y3,danger,goal,d1x,d1y,d2x,d2y,d3x,d3y] = get_status(th1,th2,th3,x,y)
%GET_STATUS 
%  ����ǶȺͻ�ͼ��־λ
%  ���ĩ��λ��x,y��
%  �Ƿ����Σ�������е��֮����ײ��danger
%  
%  �Ƿ񵽴�Ŀ�ĵ�
a=plannar3();
th=[th1,th2,th3];
xy=a.get_dist(th);%�õ�ÿ���ؽڵ�����
x1=xy(1,1);x2=xy(1,2);x3=xy(1,3);
y1=xy(2,1);y2=xy(2,2);y3=xy(2,3);

if x3>=330&&y3<=310&&y3>=290 
    danger=false;
else if x3<100||y3<280
        danger=false;
     else
        danger=false;
     end
end

d=norm(xy(:,3)'-[x,y]);
if d<5
    goal=true;
else
    goal=false;
end
d1x=x-xy(1,1);
d2x=x-xy(1,2);
d3x=x-xy(1,3);
d1y=y-xy(2,1);
d2y=y-xy(2,2);
d3y=y-xy(2,3);





    
end

