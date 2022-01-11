import pybullet as p
import time
import pybullet_data
import numpy as np
#initialise everything
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
tableStartPos = [0.0, -0.9, 0.75]
ur5standStartPos = [-0.7, -0.36, 0.0]
obj_pos = [0.2, -0.85, 0.78]
robot = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\pick_and_place_arm\urdf\robotiq_140.urdf",startPos,useFixedBase=1)
table = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\pick_and_place_arm\urdf\objects\table.urdf",tableStartPos,useFixedBase=1)
stand = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\pick_and_place_arm\urdf\objects\ur5_stand.urdf",ur5standStartPos,useFixedBase=1)
box = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\pick_and_place_arm\urdf\objects\block.urdf",obj_pos,[np.cos(np.pi/2),np.sin(np.pi/2),np.sin(np.pi/2),0])
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

#generate trajectory
start_p = [0.5,-0.5,0.5]
pick_place = [0.2,-0.85,0.99]
vec = []
traj=[]
for i in range(3):
    vec.append(pick_place[i] - start_p[i])
for i in range(100):
    t=[]
    for s in range(3):
        t.append(start_p[s] + vec[s]*i/100)
    traj.append(t)
#move to pick
for i in range(len(traj)):
    pos = p.calculateInverseKinematics(robot,8,traj[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    p.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,p.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)
#grip the block
p.setJointMotorControlArray(robot,[9,10,11,12,13,14,15,16],p.POSITION_CONTROL,targetPositions=[1,0,-np.pi/2,np.pi/2,0,0,1,1],forces=[8,8,8,8,8,8,30,30])
time.sleep(1)
tg_pick_line = []
for i in range(1,17):
    tg_pick_line.append([0.2,-0.85,0.99 - 0.02*i])

tg_pick_line_up = []

for i in range(1,21):
    tg_pick_line_up.append([obj_pos[0],obj_pos[1],tg_pick_line[-1][2] + 0.03*i])
for i in range(len(tg_pick_line_up)):
    pos = p.calculateInverseKinematics(robot,8,tg_pick_line_up[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    p.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,p.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)

tg_pick2 = [tg_pick_line_up[-1][0],tg_pick_line_up[-1][1],tg_pick_line_up[-1][2]]
tg_place =[-0.2,-0.85,0.99]
vec = []
traj=[]
for i in range(3):
   vec.append(tg_place[i] - tg_pick2[i])
for i in range(100):
    t=[]
    for s in range(3):
        t.append(tg_pick2[s] + vec[s]*i/100)
    traj.append(t)

for i in range(len(traj)):
    pos = p.calculateInverseKinematics(robot,8,traj[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    p.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,p.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)

p.setJointMotorControlArray(robot,[9,10,11,12,13,14,15,16],p.POSITION_CONTROL,targetPositions=[0,0,0,0,0,0,-np.pi/2,-np.pi/2],forces=[100,100,100,100,100,100,100,100])
time.sleep(0.2)

tg_final = [traj[-1][0],traj[-1][1],traj[-1][2]]
vec = []
traj=[]
for i in range(3):
   vec.append(tg_final[i] - start_p[i])
for i in range(100):
    t=[]
    for s in range(3):
        t.append(tg_final[s] + vec[s]*i/100)
    traj.append(t)

for i in range(len(traj)):
    pos = p.calculateInverseKinematics(robot,8,traj[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    p.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,p.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)

time.sleep(5)
p.disconnect(physicsClient)