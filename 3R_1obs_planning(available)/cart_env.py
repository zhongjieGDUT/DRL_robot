# data:2019/12/27
# 动作空间是笛卡尔坐标系内
import vrep
import math
import time
import numpy as np

class robot(object):
    # 常量定义
    ToDeg = 180.0 / math.pi  # 常数，弧度转度数
    ToRad = math.pi / 180.0  # 常数，角度转弧度
    dt = 0.05  # 定义仿真步长 50 ms
    stepsize = 0.01 # 每次末端动的固定距离
    state_dim = 10
    action_dim = 2
    # 配置关节信息
    jointNum = 3
    jointName = 'joint'
    linkName = 'link'

    config1 = [56. * ToRad, -30. * ToRad, -30. * ToRad]
    config2 = [60. * ToRad, -35. * ToRad, -30. * ToRad]

    action_bound = [-np.pi, np.pi]  # 动作的幅度限制
    get_point = False  # 到达期望点
    grab_counter = 0  # 统计到达期望点的次数
    get_obstacles = False  # 触碰到障碍
    obstacles_counter = 0  # 触碰障碍计数
    def __init__(self):
        # 建立通信
        vrep.simxFinish(-1)
        # 每隔0.2s检测一次，直到连接上V-rep
        while True:
            self.clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
            if self.clientID != -1:
                break
            else:
                time.sleep(0.1)
                print("Failed connecting to remote API server!")
        print("Connection success!")
        _, collisionHandle = vrep.simxGetCollisionHandle(self.clientID, 'Collision', vrep.simx_opmode_blocking)
        _, self.end1_Handle = vrep.simxGetObjectHandle(self.clientID, 'end', vrep.simx_opmode_blocking)
        _, self.end2_Handle = vrep.simxGetObjectHandle(self.clientID, 'end0', vrep.simx_opmode_blocking)
        _, self.goal1_Handle = vrep.simxGetObjectHandle(self.clientID, 'goal_1', vrep.simx_opmode_blocking)
        _, self.goal2_Handle = vrep.simxGetObjectHandle(self.clientID, 'goal_2', vrep.simx_opmode_blocking)
        _, self.ball1Handle = vrep.simxGetObjectHandle(self.clientID, 'ball1',
                                                 vrep.simx_opmode_blocking)
        _, self.ball2Handle = vrep.simxGetObjectHandle(self.clientID, 'ball2',
                                                  vrep.simx_opmode_blocking)
        _,self.base1Handle = vrep.simxGetObjectHandle(self.clientID, 'base', vrep.simx_opmode_blocking)
        _,self.base2Handle = vrep.simxGetObjectHandle(self.clientID, 'base0', vrep.simx_opmode_blocking)
        _,self.distanceHandle=vrep.simxGetDistanceHandle(self.clientID, 'robots_dist', vrep.simx_opmode_blocking)
        _, self.collisionHandle = vrep.simxGetCollisionHandle(self.clientID, 'Collision', vrep.simx_opmode_blocking)
        print('Handles available!')
        # _, self.pos1 = vrep.simxGetObjectPosition(self.clientID,self.ball1Handle, -1, vrep.simx_opmode_blocking)
        # _, self.pos2 = vrep.simxGetObjectPosition(self.clientID, self.ball2Handle, -1, vrep.simx_opmode_blocking)
        self.pos1 = [-0.5750,0.2959,0.0750]   # robot1末端初始位置
        self.pos2 = [0.5750,-0.2959,0.0750] # robot2末端初始位置
        _, self.goal_1 = vrep.simxGetObjectPosition(self.clientID, self.goal1_Handle,self.base1Handle, vrep.simx_opmode_blocking)
        _, self.goal_2 = vrep.simxGetObjectPosition(self.clientID, self.goal2_Handle,self.base2Handle, vrep.simx_opmode_blocking)
        del self.goal_1[2]
        del self.goal_2[2]
        self.goal_1 = np.array(self.goal_1)
        self.goal_2 = np.array(self.goal_2)
        # 获取 joint 句柄
        self.robot1_jointHandle = np.zeros((self.jointNum,), dtype=np.int)  # joint 句柄
        self.robot2_jointHandle = np.zeros((self.jointNum,), dtype=np.int)  # joint 句柄
        for i in range(self.jointNum):
            _, returnHandle = vrep.simxGetObjectHandle(self.clientID, self.jointName + str(i + 1),
                                                       vrep.simx_opmode_blocking)
            self.robot1_jointHandle[i] = returnHandle
        for i in range(self.jointNum):
            _, returnHandle = vrep.simxGetObjectHandle(self.clientID, self.jointName + str(i + 4),
                                                       vrep.simx_opmode_blocking)
            self.robot2_jointHandle[i] = returnHandle
    def step(self,action):
        action = np.clip(action, *self.action_bound)
        _, currentball1_pos = vrep.simxGetObjectPosition(self.clientID, self.ball1Handle,self.base1Handle, vrep.simx_opmode_blocking)
        _, currentball2_pos = vrep.simxGetObjectPosition(self.clientID, self.ball2Handle,self.base2Handle, vrep.simx_opmode_blocking)
        target1_pos =  currentball1_pos
        target2_pos =  currentball2_pos
        target1_pos[0] += self.stepsize*np.cos(action[0])
        target1_pos[1] += self.stepsize*np.sin(action[0])
        target2_pos[0] += self.stepsize*np.cos(action[1])
        target2_pos[1] += self.stepsize*np.sin(action[1])
        target1_pos = np.array(target1_pos)
        target2_pos = np.array(target2_pos)
        r1 = np.sqrt(np.sum(target1_pos[0:2]**2))
        if target1_pos[0]>=0:
            th1 = math.atan(target1_pos[1]/(target1_pos[0]+1e-9))
        else:
            th1 = np.pi - math.atan(target1_pos[1]/(-target1_pos[0]))
        if r1 < 0.4:
            r1 = 0.4
        if r1 > 2.30/2.0:
            r1 = 2.30/2.0

        if th1<30.*self.ToRad:
            th1 = 30.*self.ToRad
        if th1>150.*self.ToRad:
            th1 =150. * self.ToRad
        target1_pos[0] = r1 * math.cos(th1)
        target1_pos[1] = r1 * math.sin(th1)
        r2 = np.sqrt(np.sum(target2_pos[0:2] ** 2))
        if target2_pos[0] >= 0:
            th2 = math.atan(target2_pos[1] / (target2_pos[0] + 1e-9))
        else:
            th2 = np.pi - math.atan(target2_pos[1] / (-target2_pos[0]))
        if r2 < 0.4:
            r2 = 0.4

        if r2 > 2.30/2:
            r2 = 2.30/2

        if th2<30.*self.ToRad:
            th2 = 30.*self.ToRad
        if th2>150.*self.ToRad:
            th2 =150. * self.ToRad
        target2_pos[0] = r2 * math.cos(th2)
        target2_pos[1] = r2 * math.sin(th2)
        # time.sleep(0.1)
        _ = vrep.simxSetObjectPosition(self.clientID, self.ball1Handle, self.base1Handle, target1_pos, vrep.simx_opmode_oneshot)
        _ = vrep.simxSetObjectPosition(self.clientID, self.ball2Handle, self.base2Handle, target2_pos,vrep.simx_opmode_oneshot)
        jointConfig1 = np.zeros((self.jointNum,))
        jointConfig2 = np.zeros((self.jointNum,))
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot1_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig1[i] = jpos
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot2_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig2[i] = jpos
        s = np.hstack((jointConfig1, jointConfig2))
        _, end1 = vrep.simxGetObjectPosition(self.clientID, self.end1_Handle,self.base1Handle, vrep.simx_opmode_blocking)
        _, end2 = vrep.simxGetObjectPosition(self.clientID, self.end2_Handle,self.base2Handle, vrep.simx_opmode_blocking)
        del end1[2]
        del end2[2]
        end1 = np.array(end1)
        end2 = np.array(end2)
        d1 = self.goal_1 - end1
        d2 = self.goal_2 - end2
        s = np.hstack((s, np.hstack((d1, d2))))
        r=0.
        dis = np.sqrt(np.sum(d1**2))+np.sqrt(np.sum(d2**2))
        r -= dis/2.3
        if dis<=0.05:
            self.grab_counter +=1
            r+=10.
            if self.grab_counter>10:
                r+=50.
                self.get_point = True
        else:
            self.grab_counter = 0
            self.get_point = False
        _, d = vrep.simxReadDistance(self.clientID, self.distanceHandle, vrep.simx_opmode_blocking)
        if d<0.15 and d>1e-3:
            r-=(0.2/(0.2+d))**8
        if d<=1e-3:
            r-=10.
        return s,r,self.get_point,self.get_obstacles
    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        self.get_obstacles = False
        self.obstacles_counter = 0
        _ = vrep.simxSetObjectPosition(self.clientID,self.ball1Handle, -1,self.pos1 , vrep.simx_opmode_oneshot)
        _ = vrep.simxSetObjectPosition(self.clientID,self.ball2Handle , -1,self.pos2 , vrep.simx_opmode_oneshot)
        # for i in range(self.jointNum):
        #     vrep.simxSetJointPosition(self.clientID,self.robot1_jointHandle[i], self.config1[i], vrep.simx_opmode_oneshot)
        # for i in range(self.jointNum):
        #     vrep.simxSetJointPosition(self.clientID,self.robot2_jointHandle[i], self.config2[i],vrep.simx_opmode_oneshot)
        time.sleep(0.1)
        jointConfig1 = np.zeros((self.jointNum,))
        jointConfig2 = np.zeros((self.jointNum,))
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot1_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig1[i] = jpos
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot2_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig2[i] = jpos
        s = np.hstack((jointConfig1, jointConfig2))
        _, end1 = vrep.simxGetObjectPosition(self.clientID, self.end1_Handle,self.base1Handle, vrep.simx_opmode_blocking)
        _, end2 = vrep.simxGetObjectPosition(self.clientID, self.end2_Handle,self.base2Handle, vrep.simx_opmode_blocking)
        del end1[2]
        del end2[2]
        end1 = np.array(end1)
        end2 = np.array(end2)
        d1 = self.goal_1-end1
        d2 = self.goal_2-end2
        s  = np.hstack((s, np.hstack((d1,d2))))
        return s


if __name__ == '__main__':
    # env = robot()
    # print(np.cos(np.pi/2))
   print(math.atan(1/1e-9))
    # for i in range(1000):
    #     a = env.sample_action()
    #     s,_,_,danger= env.step(a)
    #
    #     print(s[-2:])


