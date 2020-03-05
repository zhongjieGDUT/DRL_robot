#  VREP 环境
import vrep
import math
import time
import numpy as np

class VREP_server(object):
    ToDeg = 180.0 / math.pi  # 常数，弧度转度数
    ToRad = math.pi / 180.0  # 常数，角度转弧度
    dt = 0.05                # 定义仿真步长 50 ms
    state_dim = 12
    action_dim = 6
    # 配置关节信息
    rewardflag = [1,1,1]
    jointNum = 3
    jointName = 'joint'
    linkName = 'link'
    obsName = 'obs1'
    config1 = [56. *ToRad,-30.*ToRad,-30.*ToRad]
    config2 = [60.*ToRad, -35.*ToRad, -30.*ToRad]
    action_bound = [-1, 1]  #动作的幅度限制
    joint_bound = [-90.*ToRad,90.*ToRad]
    get_point = False       # 到达期望点
    grab_counter = 0        # 统计到达期望点的次数

    get_obstacles = False  # 触碰到障碍
    obstacles_counter = 0  # 触碰障碍计数

    def __init__(self):
        #建立通信
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
        # # 设置机械臂仿真步长，为了保持API端与V-rep端相同步长
        vrep.simxSetFloatingParameter(self.clientID, vrep.sim_floatparam_simulation_time_step, self.dt,\
                                      vrep.simx_opmode_oneshot)
        # # 然后打开同步模式
        vrep.simxSynchronous(self.clientID, False)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)
        # vrep.simxSynchronousTrigger(self.clientID)
        #获取 joint 句柄
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
        # 获取 Link 句柄
        self.robot1_linkHandle = np.zeros((self.jointNum,), dtype=np.int)  # link 句柄
        self.robot2_linkHandle = np.zeros((self.jointNum,), dtype=np.int)  # link 句柄
        for i in range(self.jointNum):
            _, returnHandle = vrep.simxGetObjectHandle(self.clientID, self.linkName + str(i + 1),
                                                       vrep.simx_opmode_blocking)
            self.robot1_linkHandle[i] = returnHandle
        for i in range(self.jointNum):
            _, returnHandle = vrep.simxGetObjectHandle(self.clientID, self.linkName + str(i + 4),
                                                       vrep.simx_opmode_blocking)
            self.robot2_linkHandle[i] = returnHandle
        # 获取碰撞句柄
        _, self.robot1_collisionHandle = vrep.simxGetCollisionHandle(self.clientID, 'Collision_robot1', vrep.simx_opmode_blocking)
        _, self.robot2_collisionHandle = vrep.simxGetCollisionHandle(self.clientID, 'Collision_robot2', vrep.simx_opmode_blocking)

        # 获取距离句柄
        # _,self.mindist_robot1_Handle = vrep.simxGetDistanceHandle(self.clientID,'dis_robot1',vrep.simx_opmode_blocking)
        # _,self.mindist_robot2_Handle = vrep.simxGetDistanceHandle(self.clientID,'dis_robot2', vrep.simx_opmode_blocking)
        # _,self.mindist_robots_Handle = vrep.simxGetDistanceHandle(self.clientID,'robots_dist', vrep.simx_opmode_blocking)
        _,self.robot1_goal_Handle = vrep.simxGetDistanceHandle(self.clientID,'robot1_goal',vrep.simx_opmode_blocking)
        _,self.robot2_goal_Handle = vrep.simxGetDistanceHandle(self.clientID,'robot2_goal',vrep.simx_opmode_blocking)
        # 获取末端句柄
        _,self.end1_Handle = vrep.simxGetObjectHandle(self.clientID, 'end',vrep.simx_opmode_blocking)
        _,self.end2_Handle = vrep.simxGetObjectHandle(self.clientID,'end0',vrep.simx_opmode_blocking)

        _,self.goal1_Handle = vrep.simxGetObjectHandle(self.clientID, 'goal_1',vrep.simx_opmode_blocking)
        _,self.goal2_Handle = vrep.simxGetObjectHandle(self.clientID, 'goal_2', vrep.simx_opmode_blocking)
        print('Handles available!')
        _,self.goal_1 = vrep.simxGetObjectPosition(self.clientID,self.goal1_Handle,-1,vrep.simx_opmode_blocking)
        _,self.goal_2 = vrep.simxGetObjectPosition(self.clientID, self.goal2_Handle, -1,vrep.simx_opmode_blocking)
        del self.goal_1[2]
        del self.goal_2[2]
        self.goal_1 = np.array(self.goal_1)
        self.goal_2 = np.array(self.goal_2)
        self.jointConfig1 = np.zeros((self.jointNum,))
        self.jointConfig2 = np.zeros((self.jointNum,))
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot1_jointHandle[i],vrep.simx_opmode_blocking)
            self.jointConfig1[i] = jpos
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot2_jointHandle[i],vrep.simx_opmode_blocking)
            self.jointConfig2[i] = jpos

        _, collision1 = vrep.simxReadCollision(self.clientID, self.robot1_collisionHandle, vrep.simx_opmode_blocking)
        _, collision2 = vrep.simxReadCollision(self.clientID, self.robot2_collisionHandle, vrep.simx_opmode_blocking)
        _, pos1 = vrep.simxGetObjectPosition(self.clientID, self.end1_Handle, -1, vrep.simx_opmode_blocking)
        _, pos2 = vrep.simxGetObjectPosition(self.clientID, self.end2_Handle, -1, vrep.simx_opmode_blocking)
        _, d1 = vrep.simxReadDistance(self.clientID, self.robot1_goal_Handle, vrep.simx_opmode_blocking)
        _, d2 = vrep.simxReadDistance(self.clientID, self.robot2_goal_Handle, vrep.simx_opmode_blocking)
        for i in range(self.jointNum):
            _,returnpos = vrep.simxGetObjectPosition(self.clientID,self.robot1_linkHandle[i],-1, vrep.simx_opmode_blocking)

        for i in range(self.jointNum):
            _,returnpos = vrep.simxGetObjectPosition(self.clientID,self.robot2_linkHandle[i],-1,vrep.simx_opmode_blocking)
        print('programing start!')
        vrep.simxSynchronousTrigger(self.clientID)  # 让仿真走一步

    def moveto(self,config1,config2):
        # vrep.simxPauseCommunication(self.clientID, True)
        for i in range(self.jointNum):
            vrep.simxSetJointPosition(self.clientID,self.robot1_jointHandle[i], config1[i], vrep.simx_opmode_oneshot)
        for i in range(self.jointNum):
            vrep.simxSetJointPosition(self.clientID,self.robot2_jointHandle[i], config2[i],vrep.simx_opmode_oneshot)
        # vrep.simxPauseCommunication(self.clientID, False)
        # vrep.simxSynchronousTrigger(self.clientID)  # 进行下一步
        # vrep.simxGetPingTime(self.clientID)  # 使得该仿真步走完
    def getCollisonStates(self):
        _, collision1 = vrep.simxReadCollision(self.clientID,self.robot1_collisionHandle ,vrep.simx_opmode_blocking)
        _, collision2 = vrep.simxReadCollision(self.clientID, self.robot2_collisionHandle,vrep.simx_opmode_blocking)
        return collision1,collision2

    def reset(self):
        self.get_point = False
        self.grab_counter = 0
        self.get_obstacles = False
        self.obstacles_counter = 0
        self.moveto(self.config1,self.config2)
        s = self.getState_v1()
        # time.sleep(0.01)
        return s

    def step(self,action):
        action = np.clip(action, *self.action_bound)
        self.jointConfig1 =  np.zeros((self.jointNum,))
        self.jointConfig2 = np.zeros((self.jointNum,))
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot1_jointHandle[i],vrep.simx_opmode_blocking)
            self.jointConfig1[i] = jpos
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot2_jointHandle[i],vrep.simx_opmode_blocking)
            self.jointConfig2[i] = jpos
        # print(self.jointConfig1)
        self.jointConfig1 += action[:3]*self.dt
        self.jointConfig2 += action[-3:]*self.dt
        # self.jointConfig1 %= np.pi*2
        # self.jointConfig2 %= np.pi*2
        self.jointConfig1=np.clip(self.jointConfig1,*self.joint_bound)
        self.jointConfig2 = np.clip(self.jointConfig2, *self.joint_bound)
        self.moveto(self.jointConfig1,self.jointConfig2)
        # time.sleep(0.1)
        s_ =  self.getState_v1()

        r = self._r_func()
        return s_,r,self.get_point,self.get_obstacles

    def getState(self):#版本一的状态定义 维度是18
        # output type : list
        _, pos1 = vrep.simxGetObjectPosition(self.clientID,self.end1_Handle,-1, vrep.simx_opmode_buffer)
        _, pos2 = vrep.simxGetObjectPosition(self.clientID, self.end2_Handle,-1,vrep.simx_opmode_buffer)
        del pos1[2]
        del pos2[2]
        pos1 = np.array(pos1)
        pos2 = np.array(pos2)
        s = np.hstack((pos1,pos2))
        robot1_link_pos = np.zeros((self.jointNum,2),dtype=np.float)
        robot2_link_pos = np.zeros((self.jointNum, 2), dtype=np.float)
        for i in range(self.jointNum):
            _,returnpos = vrep.simxGetObjectPosition(self.clientID,self.robot1_linkHandle[i],-1, vrep.simx_opmode_buffer)
            del returnpos[2]
            returnpos = np.array(returnpos)
            robot1_link_pos[i,:] = returnpos-pos1
        for i in range(self.jointNum):
            _,returnpos = vrep.simxGetObjectPosition(self.clientID,self.robot2_linkHandle[i],-1,vrep.simx_opmode_buffer)
            del returnpos[2]
            returnpos = np.array(returnpos)-pos2
            robot2_link_pos[i,:] = returnpos
        robot1_link_pos = np.reshape(robot1_link_pos,newshape=(-1,))
        robot2_link_pos = np.reshape(robot2_link_pos, newshape=(-1,))
        s = np.hstack((s,np.hstack((robot1_link_pos,robot2_link_pos))))
        collision1,collision2=self.getCollisonStates()
        danger  = np.array([1. if collision1 else 0. ,1. if collision2 else 0.])
        s = np.hstack((s,danger))

        return s
    def getState_v1(self): #版本二的状态定义，维度是14
        jointConfig1 = np.zeros((self.jointNum,))
        jointConfig2 = np.zeros((self.jointNum,))
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot1_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig1[i] = jpos
        for i in range(self.jointNum):
            _, jpos = vrep.simxGetJointPosition(self.clientID, self.robot2_jointHandle[i], vrep.simx_opmode_blocking)
            jointConfig2[i] = jpos
        s = np.hstack((jointConfig1, jointConfig2))
        _, pos1 = vrep.simxGetObjectPosition(self.clientID, self.end1_Handle, -1, vrep.simx_opmode_blocking)
        _, pos2 = vrep.simxGetObjectPosition(self.clientID, self.end2_Handle, -1, vrep.simx_opmode_blocking)
        del pos1[2]
        del pos2[2]
        pos1 = np.array(pos1)
        pos2 = np.array(pos2)
        pos = np.hstack((pos1, pos2))
        s = np.hstack((s,pos))

        _, d1 = vrep.simxReadDistance(self.clientID, self.robot1_goal_Handle, vrep.simx_opmode_blocking)
        _, d2 = vrep.simxReadDistance(self.clientID, self.robot2_goal_Handle, vrep.simx_opmode_blocking)
        s = np.hstack((s, np.array([d1,d2])))
        # collision1, collision2 = self.getCollisonStates()
        # danger = np.array([1. if collision1 else 0., 1. if collision2 else 0.])
        # s = np.hstack((s, danger))
        return s
    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.jointNum*2)
    def _r_func(self):
        t=20
        collision1, collision2 = self.getCollisonStates()
        collision = np.array([1. if collision1 else 0., 1. if collision2 else 0.])
        _,d1 = vrep.simxReadDistance(self.clientID,self.robot1_goal_Handle, vrep.simx_opmode_blocking)
        _,d2 = vrep.simxReadDistance(self.clientID,self.robot2_goal_Handle, vrep.simx_opmode_blocking)
        d = d1+d2

        r = -d
        if d<=0.1 and not self.get_point:
            r+=2.
            self.grab_counter+=1
            if self.grab_counter>t:
                r+=10.
                self.get_point =True
        elif d>0.1:
            self.grab_counter=0
            self.get_point =False

        if (collision[0]==1.):
            r-=10.
            self.obstacles_counter+=1
            if self.obstacles_counter>=3:
                self.get_obstacles=True
        elif collision[0]==0.:
            self.obstacles_counter = 0
            self.get_obstacles = False
        return r

if __name__ == '__main__':
    env = VREP_server()

    # for i in range(1000):
    #     a = env.sample_action()
    #     s,_,_,danger= env.step(a)
    #
    #     print(s[-2:])


