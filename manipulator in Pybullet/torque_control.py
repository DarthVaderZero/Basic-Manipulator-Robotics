import time
import numpy as np
import pybullet as pb
import pybullet_data


physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
path = r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf"
robot = pb.loadURDF(path, [0, 0, 0], useFixedBase=1)
pb.setGravity(0,0,0)
st_pt = [0.7,-0.4,1-0.13]
en_pt = [0.35,0.6,1.25]
pb.addUserDebugLine(st_pt,en_pt,[1,0,0],8,0)


xt=[]
yt=[]
zt=[]
n=50
for i in range(0 ,n+1):
    xt.append(((n-i)*(st_pt[0])+i*(en_pt[0]))/n)
    yt.append(((n - i) * (st_pt[1]) + i * (en_pt[1])) / n)
    zt.append(((n - i) * (st_pt[2]) + i * (en_pt[2])) / n)

vec= [en_pt[0]-st_pt[0],en_pt[1]-st_pt[1],en_pt[2]-st_pt[2]]

rts = int(input("1 to start simulation"))
pb.setRealTimeSimulation(rts) 
#starting position
z = pb.calculateInverseKinematics(1,7,st_pt)
pb.setJointMotorControlArray(robot, [1,2,3,4,5,6], pb.POSITION_CONTROL,targetPositions=z) 
time.sleep(1)
w = 2
zerovec = np.zeros(7)
pb.setJointMotorControlArray(robot,[1,2,3,4,5,6,7],pb.VELOCITY_CONTROL,forces=zerovec)
velocity = np.array([[vec[0] * w], [vec[1] * w], [vec[2] * w]])
kv=0
kp=200
kd= 20

for j in range(1,len(xt)):
    e_p = np.array([[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][0]],[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][1]],[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][2]]]) #np.array([[-0.02771], [0.022611], [-0.02339 + 0.05]])

    while abs( e_p[0] - xt[j])>0.03 or abs(e_p[1] - yt[j])>0.03 or abs(e_p[2] - zt[j])>0.03:
        theta = []
        vel = []
        for i in range(1, 7):
            s = pb.getJointState(robot, i, physicsClient)
            theta.append(s[0])
            vel.append(s[1])
        
        jaco = pb.calculateJacobian(robot, 7, [0, 0, 0], theta, vel, [0, 0, 0, 0, 0, 0])
        Jacv = np.array(jaco[0])
        
        jacin = np.dot(np.dot(np.transpose(Jacv), Jacv), np.transpose(Jacv))
        e_p = np.array([[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][0]],[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][1]],[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][2]]])  #np.array([[-0.02771], [0.022611], [-0.02339 + 0.05]])

        qdot = np.dot(jacin, (velocity + kv * (np.array([[xt[j]], [yt[j]], [zt[j]]]) - e_p)))
        qdot.resize(6, )
        despos = np.array(pb.calculateInverseKinematics(robot,7,[xt[j],yt[j],zt[j]]))

        tor = kp*(despos - np.array(theta)) + kd*(qdot-np.array(vel))
        print(np.shape(tor))
        pb.setJointMotorControlArray(robot,[1,2,3,4,5,6],pb.TORQUE_CONTROL,forces=tor)


