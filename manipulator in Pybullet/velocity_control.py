import time
import numpy as np
import pybullet as pb
import pybullet_data

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
path = r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf"
robot = pb.loadURDF(path, [0, 0, 0], useFixedBase=1)
st_pt = [0.7,-0.4,1-0.13]
en_pt = [0.35,0.6,1.25]
en_pt_np = np.array(en_pt)
st_pt_np = np.array(st_pt)
k=0.05
trajectory = []
velocity = k*(en_pt_np-st_pt_np)
print(velocity)
pb.addUserDebugLine(st_pt,en_pt,[1,0,0],4,0)
for i in range(101):
    x_t = en_pt[0]*(i/100) + st_pt[0]*(100-i)/100
    y_t = en_pt[1]*(i/100) + st_pt[1]*(100-i)/100
    z_t = en_pt[2]*(i/100) + st_pt[2]*(100-i)/100
    trajectory.append([x_t,y_t,z_t])
trajectory = np.array(trajectory)

rts = int(input("1 to start simulation"))
pb.setRealTimeSimulation(rts) 
a= pb.calculateInverseKinematics(1,7,st_pt)
for i in range(len(a)):
    pb.setJointMotorControl2(robot,i+1,pb.POSITION_CONTROL,targetPosition=a[i],force=100)
    print("Joint {i} moved to position")
time.sleep(4)
n=0
k_p=0.1
while n<101:
    theta = []
    vel = []
    for i in range(1,7):
        s = pb.getJointState(robot,i,physicsClient)
        theta.append(s[0])
        vel.append(s[1])
    #print("theta" ,len(theta))
    jaco = pb.calculateJacobian(robot,7,[0,0,0],theta,vel,[0,0,0,0,0,0])
    J_v = np.array(jaco[0])
    #J_v_inv = np.dot(np.dot(np.transpose(J_v),J_v),np.transpose(J_v))
    J_v_pinv = np.linalg.pinv(J_v)
    effector_pos = pb.getLinkState(robot,7)[0]
    x_dot = velocity+k_p*(trajectory[n]-np.array(effector_pos))
    q_dot = np.dot(J_v_pinv,x_dot)
    pb.setJointMotorControlArray(robot,[1,2,3,4,5,6],pb.VELOCITY_CONTROL,targetVelocities=q_dot)
    time.sleep(0.2)
    n=n+1
time.sleep(2.5)
pb.disconnect(physicsClient)
