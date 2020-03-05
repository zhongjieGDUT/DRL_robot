import numpy as np
import matlab.engine
import math
eng = matlab.engine.connect_matlab('robot')
# goal=np.zeros(6,dtype=float)
# Te = eng.getval('goal',nargout=1)         #从MATLAB获取期望的位姿：六维向量
# print(Te[0][0])
# #目标位姿和初始的机械臂角度
# #注意：采用这个上面这一条语句要加上[one.item() for one in self.arm_info]
# for i in range(6):
#     goal[i]=Te[0][i]            #自定义固定的位姿
# for i in range(3):                   #归一化弧度角
#     goal[i+3]=goal[i+3]/np.pi
# print(goal)
[T2,T3,T4,T5,T6,dT2,dT3,dT4,dT5,dT6,doT3,doT6,obs] = eng.get_state([0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.],[0.,0.,0.2],
                                                                     nargout=13)
print(doT3[0])
print(type(T2[0]))
abs_distance= np.sqrt(np.sum(np.square(dT6[0])))
print(abs_distance)
# for i in range(6):
#     g[i]=goal[0][i]
# print(g)
# print(type(g))