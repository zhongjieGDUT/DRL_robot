"""
Environment for Robot Arm.
You can customize this script in a way you want.

View more on [莫烦Python] : https://morvanzhou.github.io/tutorials/
此代码为 三自由度避障环境
注！！！
只有一个圆形障碍，训练到达随机目标点
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
    state_dim = 16          # 状态维度
    dt = .1                 # refresh rate sample time Ts
    get_point = False      #到达期望点
    grab_counter = 0       #统计到达期望点的次数
    area=10.
    get_obstacles=False   #触碰到障碍
    obstacles_counter = 0  #触碰障碍计数

    def __init__(self):

        # 该变量表示机械臂三个轴的转动量
        self.arm_info = np.zeros(3, dtype=float)
        self.goal = np.zeros(2, dtype=float)
        # 建立与matlab的联系
        self.eng = matlab.engine.connect_matlab('robot')
        # 障碍的圆心和半径
        self.Obstacles = np.zeros(3, dtype=float)



    def step(self, action):

        action = np.clip(action, *self.action_bound)
        self.arm_info += action*self.dt
        self.arm_info %= np.pi*2.0
        # 由于matlab 不受支持的 Python 数据类型: numpy.float64
        # self.goal= [one.item() for one in self.goal]
        self.arm_info=[one.item() for one in self.arm_info]

        [cd1, cd2, dx1, dy1, dx2, dy2, dx3, dy3, dx, dy, dox1, doy1,
         dox2, doy2, dox3, doy3, dxo1, dyo1, dxo2, dyo2, dxo3, dyo3] = self.eng.get_state(self.arm_info[0],
                                                                                          self.arm_info[1],
                                                                                          self.arm_info[2],
                                                                                          self.goal[0], self.goal[1],
                                                                                          self.Obstacles, nargout=22)
        in_point = 1 if self.grab_counter > 0 else 0
        danger=1 if self.obstacles_counter>0 else 0
        r = self._r_func(dx, dy, dxo1, dyo1, dxo2, dyo2, dxo3, dyo3)
        s = np.concatenate(([dx1, dy1, dx2, dy2, dx3, dy3, cd1, cd2, dox1, doy1,
                             dox2, doy2, dox3, doy3], [in_point, danger]))
        return s, r, self.get_point,self.get_obstacles
    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        self.get_obstacles = False
        self.obstacles_counter=0

        #产生目标点和初始的机械臂角度
        theta=random.uniform(-np.pi/2,-np.pi/10)  #目标点的角度 （极坐标形式）
        L=random.random()*300                     #目标点的长度  （极坐标形式）
        self.goal =[300+np.cos(theta)*L,300+np.sin(theta)*L]   #生成目标点的横纵坐标
        #注意：采用这个上面这一条语句要加上[one.item() for one in self.arm_info]
        self.goal=[516.,185.]                                #自定义固定的横纵坐标
        self.arm_info=[np.pi/2,0.,0.]
        # self.arm_info=np.random.rand(3) * np.pi * 2
        # self.arm_info=self.arm_info.tolist()

        self.Obstacles=[550.,300.,50.]          #圆形障碍的圆心【x,y】和半径r

        print(self.arm_info)
        print(self.goal[0],self.goal[1])
        # self.arm_info = [one.item() for one in self.arm_info]
        # self.goal = [one.item() for one in self.goal]
        # self.Obstacles = [one.item() for one in self.Obstacles]
        [cd1, cd2, dx1, dy1, dx2, dy2, dx3, dy3, dx, dy,dox1,doy1,
         dox2,doy2,dox3,doy3,dxo1,dyo1,dxo2,dyo2,dxo3,dyo3] = self.eng.get_state(self.arm_info[0], self.arm_info[1],
                                                                                 self.arm_info[2],self.goal[0], self.goal[1],
                                                                                 self.Obstacles ,nargout=22)
        in_point = 1 if self.grab_counter > 0 else 0
        danger = 1 if self.obstacles_counter > 0 else 0
        s = np.concatenate(([dx1, dy1, dx2, dy2, dx3, dy3, cd1, cd2,dox1,doy1,
         dox2,doy2,dox3,doy3], [in_point,danger]))
        return s

    def render(self):
        # self.arm_info = [one.item() for one in self.arm_info]
        self.eng.render(self.arm_info[0], self.arm_info[1],self.arm_info[2],
                        self.goal[0],self.goal[1],self.Obstacles[0],
                        self.Obstacles[1],self.Obstacles[2],nargout=0)
        return 0


    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def _r_func(self,dx,dy,dxo1,dyo1,dxo2,dyo2,dxo3,dyo3):
        t = 50
        abs_distance = np.sqrt(np.sum(np.square([dx,dy])))
        abs_distance1 = np.sqrt(np.sum(np.square([dxo1,dyo1])))
        abs_distance2 = np.sqrt(np.sum(np.square([dxo2,dyo2])))
        abs_distance3 = np.sqrt(np.sum(np.square([dxo3,dyo3])))
        r = -abs_distance / 300 #+(abs_distance1-600)/600+(abs_distance2-600)/600+(abs_distance3-600)/600
        if abs_distance <= self.area and (not self.get_point):
            r += 1.
            self.grab_counter += 1
            if self.grab_counter > t:
                r += 10.
                self.get_point = True
        elif abs_distance > self.area:
            self.grab_counter= 0
            self.get_point = False

        if (abs_distance2-50<self.area or abs_distance3-50<self.area) and (not self.get_obstacles):
            r-=1.
            self.obstacles_counter+=1
            if self.obstacles_counter>=t/25:
                r-=10.
                self.get_obstacles=True
        elif abs_distance2-50>=self.area and abs_distance3-50>=self.area:
            self.obstacles_counter=0
            self.get_obstacles=False
        return r