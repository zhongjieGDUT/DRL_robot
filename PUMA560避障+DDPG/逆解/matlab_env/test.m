load('q.mat');
L={'r','LineWidth',2};
for i=1:62
    qq(i,:)=cell2mat(q{1,i});
end
p560.plot(qq,'trail',L);
