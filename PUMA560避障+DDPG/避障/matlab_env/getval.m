function temp = getval(name)
%GETVAL 获取数据
%   主要是目标的位姿，
temp=zeros(1,9);
if name=='goal'
    Te_xyz=evalin('base','Te_xyz');
    Te_zyz=evalin('base','Te_zyz');
    temp(1:3)=Te_xyz';
    temp(4:6)=Te_zyz;
else if name=='angl'
        temp=evalin('base','q0');
    end    
end
cylinder_xyz=evalin('base','cylinder_xyz');
r=evalin('base','r');
temp(7)=cylinder_xyz(1);
temp(8)=cylinder_xyz(2);
temp(9)=r;
end

