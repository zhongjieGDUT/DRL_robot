function  render(q)
%RENDER ���ƻ�е��
%   
% q=cell2mat(q);
p560=evalin('base','p560');
L={'r','LineWidth',2};
p560.plot(q,'trail',L);

end

