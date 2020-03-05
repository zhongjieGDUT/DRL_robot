"""
Environment for Robot Arm.
You can customize this script in a way you want.

此代码为 训练 puma560逆运动程序
注！！！
期望末端位姿为
【Tx,Ty,Tz,Rz,Ry,Rz】
Requirement:
pyglet >= 1.2.4
numpy >= 1.12.1
"""
import numpy as np
import matlab.engine
import math
import random


class ArmEnv(object):
    action_bound = [-1, 1]  #动作的幅度限制 弧度制
    action_dim = 6         #动作维度：三自由度机械臂
    state_dim = 61         # 状态维度
    dt = .1                 # refresh rate sample time Ts
    get_point = False      #到达期望点
    grab_counter = 0       #统计到达期望点的次数
    area=0.001
    qz=[0.,0.,0.,0.,0.,0.] #机械臂初始角度

    def __init__(self):
        # 该变量表示机械臂6个轴的转动量
        self.arm_info = np.zeros(6, dtype=float)
        self.goal = np.zeros(6, dtype=float)
        # 建立与matlab的联系
        self.eng = matlab.engine.connect_matlab('robot')
        self.eng.eval('init',nargout=0)


    def step(self, action):
        action = np.clip(action, *self.action_bound)
        self.arm_info += action*self.dt
        # self.arm_info %= np.pi*2.0
        # 由于matlab 不受支持的 Python 数据类型: numpy.float64
        # self.goal= [one.item() for one in self.goal]
        self.arm_info=[one.item() for one in self.arm_info]

        [theta,T2,T3,T4,T5,T6,dT2,dT3,dT4,dT5,dT6,dT6_] = self.eng.state(self.arm_info,self.goal,nargout=12)
        self.arm_info = theta[0]
        # self.render(theta[0])
        in_point = 1 if self.grab_counter > 0 else 0

        r = self._r_func(dT6_[0])
        s = np.concatenate((T2[0],T3[0],T4[0],T5[0],T6[0],dT2[0],dT3[0],dT4[0],dT5[0],dT6[0],[in_point]))
        return s, r, self.get_point

    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        #目标位姿和初始的机械臂角度
        #注意：采用这个上面这一条语句要加上[one.item() for one in self.arm_info]

        self.goal = [0.6963,0.0499,0.1856,0.2810,2.0186,0.5880]
        self.arm_info = self.qz

        # self.arm_info = [one.item() for one in self.arm_info]
        # self.goal = [one.item() for one in self.goal]
        # self.Obstacles = [one.item() for one in self.Obstacles]

        [theta,T2,T3,T4,T5,T6,dT2,dT3,dT4,dT5,dT6,dT6_] = self.eng.state(self.arm_info,self.goal,nargout=12)
        self.arm_info = theta[0]
        self.render(self.arm_info)
        # print(self.arm_info)
        in_point = 1 if self.grab_counter > 0 else 0
        s = np.concatenate((T2[0],T3[0],T4[0],T5[0],T6[0],dT2[0],dT3[0],dT4[0],dT5[0],dT6[0],[in_point]))
        return s

    def render(self,q):
        # self.arm_info = [one.item() for one in self.arm_info]
        self.eng.render(q,nargout=0)
        return 0


    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self,dT6_):
        t = 50
        abs_distance = np.sqrt(np.sum(np.square([dT6_[0],dT6_[1],dT6_[2]])))
        abs_angle = np.sqrt(np.sum(np.square([dT6_[3],dT6_[4],dT6_[5]])))
        r = -abs_distance/3- abs_angle/np.pi

        if abs_distance <=0.01 and abs_angle<=0.01 and (not self.get_point):
            r += 1.
            self.grab_counter += 1
            if self.grab_counter > t:
                r += 10.
                self.get_point = True
        elif abs_distance > 0.01 or abs_angle>0.01:
            self.grab_counter= 0
            self.get_point = False
        return r

