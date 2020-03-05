function  render(q)
%RENDER »æÖÆ»úÐµ±Û
%   
% q=cell2mat(q);
p560=evalin('base','p560');
L={'r','LineWidth',2};
% cylinder=evalin('base','cylinder_collision');
p560.plot(q,'workspace',[-1.2 1.2 -1.2 1.2 -0.8 1],'trail',L);


end

