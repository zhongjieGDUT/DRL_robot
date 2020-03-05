function A = T2e(T)
%T2E 将齐次变换矩阵变换成六维的向量
%角度归一化了
A=zeros(1,6);
A(1:3)=transl(T)';
A(4:6)=tr2eul(T);

end

