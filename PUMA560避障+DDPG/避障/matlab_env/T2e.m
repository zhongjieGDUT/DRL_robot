function A = T2e(T)
%T2E ����α任����任����ά������
%�Ƕȹ�һ����
A=zeros(1,6);
A(1:3)=transl(T)';
A(4:6)=tr2eul(T);

end

