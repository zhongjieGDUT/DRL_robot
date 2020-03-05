"""
Environment for Robot Arm.
You can customize this script in a way you want.

此代码为 训练 puma560避障程序
注！！！
只有一个圆柱形障碍，训练到达随机目标点用一个六维向量表示，
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
    state_dim = 66         # 状态维度
    dt = .05                 # refresh rate sample time Ts
    get_point = False      #到达期望点
    grab_counter = 0       #统计到达期望点的次数
    area=0.1
    get_obstacles=False   #触碰到障碍
    obstacles_counter = 0  #触碰障碍计数


    def __init__(self):
        # 该变量表示机械臂6个轴的转动量
        self.arm_info = np.zeros(6, dtype=float)
        self.goal = np.zeros(6, dtype=float)
        # 建立与matlab的联系
        self.eng = matlab.engine.connect_matlab('robot')
        # self.eng.eval('init',nargout=0)
        # 圆柱形障碍的圆心位置（x,y)和半径r
        self.Obstacles = np.zeros(3, dtype=float)

    def step(self, action):

        action = np.clip(action, *self.action_bound)
        self.arm_info += action*self.dt

        # 由于matlab 不受支持的 Python 数据类型: numpy.float64
        # self.goal= [one.item() for one in self.goal]
        self.arm_info=[one.item() for one in self.arm_info]

        [theta,T2,T3,T4,T5,T6,dT2,dT3,dT4,dT5,dT6,doT3,doT6,obs,dT6_,doT3_,doT6_] = self.eng.get_state(self.arm_info,
                                                                                          self.goal,self.Obstacles,
                                                                                       nargout=17)
        self.arm_info=theta[0]
        # self.render(self.arm_info)
        in_point = 1 if self.grab_counter > 0 else 0
        danger=1 if self.obstacles_counter>0 else 0
        r = self._r_func(dT6_,doT6_,obs)
        s = np.concatenate((T2[0],T3[0],T4[0],T5[0],T6[0],dT2[0],dT3[0],dT4[0],dT5[0],dT6[0],doT3[0],doT6[0],[in_point,danger]))
        return s, r, self.get_point,self.get_obstacles

    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        self.get_obstacles = False
        self.obstacles_counter=0
        #目标位姿和初始的机械臂角度
        #注意：采用这个上面这一条语句要加上[one.item() for one in self.arm_info]
        self.goal=[0.4263,0.4552,0.4548,1.0009,0.8646,-0.0472]

        self.arm_info = [-0.6702,0.,-0.8482,-0.1955,-0.9425,0.1850]
        self.Obstacles=[0.8963,-0.1501,0.2]          #圆形障碍的圆心【x,y】和半径r

        # self.arm_info = [one.item() for one in self.arm_info]
        # self.goal = [one.item() for one in self.goal]
        # self.Obstacles = [one.item() for one in self.Obstacles]

        [theta, T2, T3, T4, T5, T6, dT2, dT3, dT4, dT5, dT6, doT3, doT6, obs, dT6_, doT3_, doT6_] = self.eng.get_state(self.arm_info,
                                                                                                                       self.goal,
                                                                                                                       self.Obstacles,
                                                                                                                       nargout=17)
        self.arm_info=theta[0]
        # self.render(self.arm_info)
        in_point = 1 if self.grab_counter > 0 else 0
        danger = 1 if self.obstacles_counter > 0 else 0

        s = np.concatenate((T2[0],T3[0],T4[0],T5[0],T6[0],dT2[0],dT3[0],dT4[0],dT5[0],dT6[0],doT3[0],doT6[0],[in_point,danger]))
        return s

    def render(self,q):
        # self.arm_info = [one.item() for one in self.arm_info]
        self.eng.render(q,nargout=0)
        return 0


    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self,dT6_,doT6_,obs):
        t = 50
        abs_distance = np.sqrt(np.sum(np.square(dT6_[0])))
        abs_distance1 = np.sqrt(np.sum(np.square(doT6_[0])))
        r = -abs_distance/3

        if abs_distance <= 0.05 and (not self.get_point):
            r += 1.
            self.grab_counter += 1
            if self.grab_counter > t/5:
                r += 10.
                self.get_point = True
        elif abs_distance > 0.05:
            self.grab_counter = 0
            self.get_point = False

        if obs and (not self.get_obstacles):
            r -= 1.
            self.obstacles_counter += 1
            if self.obstacles_counter >= t/25:
                r -= 10.
                self.get_obstacles=True
        elif not obs:
            self.obstacles_counter=0
            self.get_obstacles=False
        return r
