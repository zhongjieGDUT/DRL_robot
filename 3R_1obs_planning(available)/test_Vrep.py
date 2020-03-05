#  VREP 环境
import vrep
import math
import time
import numpy as np
dt = 0.05
jointNum=3
jointName ='joint'
#建立通信
vrep.simxFinish(-1)
# 每隔0.2s检测一次，直到连接上V-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        break
    else:
        time.sleep(0.1)
        print("Failed connecting to remote API server!")
print("Connection success!")

_, ballHandle = vrep.simxGetObjectHandle(clientID, 'ball1',
                                               vrep.simx_opmode_blocking)
_, ball0Handle = vrep.simxGetObjectHandle(clientID, 'ball2',
                                               vrep.simx_opmode_blocking)
_,collisionHandle = vrep.simxGetCollisionHandle(clientID,'Collision', vrep.simx_opmode_blocking)
# 获取 joint 句柄
robot1_jointHandle = np.zeros((jointNum,), dtype=np.int)  # joint 句柄
robot2_jointHandle = np.zeros((jointNum,), dtype=np.int)  # joint 句柄
for i in range(jointNum):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i + 1),
                                               vrep.simx_opmode_blocking)
    robot1_jointHandle[i] = returnHandle
for i in range(jointNum):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i + 4),
                                               vrep.simx_opmode_blocking)
    robot2_jointHandle[i] = returnHandle
_,base1Handle = vrep.simxGetObjectHandle(clientID, 'base', vrep.simx_opmode_blocking)
print('Handles available!')
jointConfig1 = np.zeros((jointNum,))
jointConfig2 = np.zeros((jointNum,))
# for i in range(jointNum):
#     _, jpos = vrep.simxGetJointPosition(clientID, robot1_jointHandle[i],vrep.simx_opmode_blocking)
#     jointConfig1[i] = jpos
# print(jointConfig1*180./np.pi)
# for i in range(jointNum):
#     _, jpos = vrep.simxGetJointPosition(clientID, robot2_jointHandle[i],vrep.simx_opmode_blocking)
#     jointConfig2[i] = jpos

_, pos = vrep.simxGetObjectPosition(clientID,ballHandle,base1Handle,vrep.simx_opmode_blocking)
print(pos)
_, pos1 = vrep.simxGetObjectPosition(clientID,ball0Handle,-1,vrep.simx_opmode_blocking)
_, collision = vrep.simxReadCollision(clientID, collisionHandle, vrep.simx_opmode_blocking)

# for i in range(0):
#     pos[0]+=0.01
#     pos1[0]-=0.01
#     _ = vrep.simxSetObjectPosition(clientID,ballHandle,-1,pos,vrep.simx_opmode_oneshot)
#     _ = vrep.simxSetObjectPosition(clientID, ball0Handle, -1, pos1, vrep.simx_opmode_oneshot)
#     for i in range(jointNum):
#         _, jpos = vrep.simxGetJointPosition(clientID, robot1_jointHandle[i], vrep.simx_opmode_blocking)
#         jointConfig1[i] = jpos
#     _, collision = vrep.simxReadCollision(clientID, collisionHandle, vrep.simx_opmode_blocking)
#     print(collision,jointConfig1*180./np.pi)
# #get simulation time


# while vrep.simxGetConnectionId(clientID)!=-1:
#     currCmdTime = vrep.simxGetLastCmdTime(clientID)
#     dt = (currCmdTime - lastCmdTime) / 1000
#     for i in range(jointNum):
#         _, jpos = vrep.simxGetJointPosition(clientID, robot1_jointHandle[i], vrep.simx_opmode_buffer)
#         jointConfig1[i] = jpos
#     for i in range(jointNum):
#         _, jpos = vrep.simxGetJointPosition(clientID, robot2_jointHandle[i], vrep.simx_opmode_buffer)
#         jointConfig2[i] = jpos
#     # print(jointConfig1)
#     vrep.simxPauseCommunication(clientID, True)
#     for i in range(jointNum):
#         _ = vrep.simxSetJointTargetPosition(clientID, robot1_jointHandle[i],np.pi/2,vrep.simx_opmode_oneshot)
#
#     vrep.simxPauseCommunication(clientID, False)
#     lastCmdTime = currCmdTime
#     vrep.simxSynchronousTrigger(clientID)
#     _,T = vrep.simxGetPingTime(clientID)
#     print(T)
