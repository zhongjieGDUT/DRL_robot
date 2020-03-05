load('q.mat');
L={'r','LineWidth',2};
for i=1:68
    qq(i,:)=cell2mat(q{1,i});
end
T=p560.fkine(qn); T=se2t(T);
cylinder=Cylinder(T*troty(-pi/2)*transl(0.3,0,-0.5),[0.2,0.2,1.5]);                                              %Ô²Öù
cylinder_collision=CollisionModel(cylinder);
cylinder.plot();%axis equal;
p560.plot3d(qq,'view',[-40,50],'workspace',[-1.2 1.2 -1.2 1.2 -0.8 1],'trail',L);
