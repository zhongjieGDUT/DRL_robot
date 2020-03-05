"""
Environment for Robot Arm.
You can customize this script in a way you want.

View more on [莫烦Python] : https://morvanzhou.github.io/tutorials/


Requirement:
pyglet >= 1.2.4
numpy >= 1.12.1
"""
import numpy as np
import matlab.engine
import math
import random


class ArmEnv(object):

    action_bound = [-1, 1]  #动作的幅度限制
    action_dim = 3          #动作维度：三自由度机械臂
    state_dim = 9           # 状态维度
    dt = .1                 # refresh rate sample time Ts
    get_point = False      #到达期望点
    grab_counter = 0
    area=10.

    def __init__(self):

        # 该变量表示机械臂三个轴的转动量
        self.arm_info = np.zeros(3, dtype=float)
        self.goal = np.zeros(2, dtype=float)
        # 建立与matlab的联系
        self.eng = matlab.engine.connect_matlab('robot')



    def step(self, action):

        action = np.clip(action, *self.action_bound)
        self.arm_info += action*self.dt
        self.arm_info %= np.pi*2.0
        # 由于matlab 不受支持的 Python 数据类型: numpy.float64

        # self.goal= [one.item() for one in self.goal]
        self.arm_info=[one.item() for one in self.arm_info]
        [cd1,cd2,dx1,dy1,dx2,dy2,dx3,dy3,dx,dy] = self.eng.get_state(self.arm_info[0],self.arm_info[1],self.arm_info[2],
                                                      self.goal[0],self.goal[1], nargout=10)

        in_point = 1 if self.grab_counter > 0 else 0

        s = np.concatenate(([dx1,dy1,dx2,dy2,dx3,dy3,cd1,cd2], [in_point]))

        r = self._r_func(dx,dy)



        return s, r, self.get_point

    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        theta=random.random()*np.pi*2
        L=random.random()*300
        self.goal =[300+np.cos(theta)*L,300+np.sin(theta)*L]
        # self.goal=[100.,500.]
        self.arm_info=[np.pi/2,0.,0.]
        # self.arm_info=np.random.rand(3) * np.pi * 2
        # self.arm_info=self.arm_info.tolist()
        # print(self.arm_info)
        # print(self.goal[0],self.goal[1])
        # self.arm_info = [one.item() for one in self.arm_info]
        self.goal = [one.item() for one in self.goal]
        [cd1, cd2, dx1, dy1, dx2, dy2, dx3, dy3, dx, dy] = self.eng.get_state(self.arm_info[0], self.arm_info[1],self.arm_info[2],
                                                                              self.goal[0], self.goal[1], nargout=10)
        in_point = 1 if self.grab_counter > 0 else 0

        s = np.concatenate(([dx1, dy1, dx2, dy2, dx3, dy3, cd1, cd2], [in_point]))



        return s

    def render(self):
        # self.arm_info = [one.item() for one in self.arm_info]
        self.eng.render(self.arm_info[0], self.arm_info[1],self.arm_info[2], self.goal[0],self.goal[1], nargout=0)
        return 0


    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self,dx,dy):
        t = 50
        abs_distance = np.sqrt(np.sum(np.square([dx,dy])))
        r = -abs_distance / 300
        if abs_distance <= self.area and (not self.get_point):
            r += 1.
            self.grab_counter += 1
            if self.grab_counter > t:
                r += 10.
                self.get_point = True
        elif abs_distance > self.area:
            self.grab_counter= 0
            self.get_point = False
        return r