import time
import numpy as np
import pybullet as pb
import pybullet_data

import sympy as sp
from sympy import symbols
u = symbols('u')
H = sp.Matrix([[2,-2,1,1],[-3,3,-2,-1],[0,0,1,0],[1,0,0,0]])

def Curve(start,stop,control):
    X_bounds = sp.Matrix([start[0],stop[0],control[0]-start[0],-stop[0]+control[0]])
    Y_bounds = sp.Matrix([start[1],stop[1],control[1]-start[1],-stop[1]+control[1]])
    Z_bounds = sp.Matrix([start[2],stop[2],control[2]-start[2],-stop[2]+control[2]])
    X_coeff = H*X_bounds
    Y_coeff = H*Y_bounds
    Z_coeff = H*Z_bounds
    expr_x = X_coeff[0]*(u**3)+X_coeff[1]*(u**2)+X_coeff[2]*(u)+X_coeff[3]
    expr_y = Y_coeff[0]*(u**3)+Y_coeff[1]*(u**2)+Y_coeff[2]*(u)+Y_coeff[3]
    expr_z = Z_coeff[0]*(u**3)+Z_coeff[1]*(u**2)+Z_coeff[2]*(u)+Z_coeff[3]
    return [expr_x,expr_y,expr_z]
st_pt = [0.7,-0.4,1-0.13]
en_pt = [0.35,0.6,1.25]
ct_pt = [(0.7+0.35)/2,(0.6-0.4)/2,(2.25-0.13)/2]
X_Y_Z = Curve(st_pt,en_pt,ct_pt)
n = 300
xt = []
yt = []
zt = []
for i in range(0 ,n+1):
    i_sub = (1/n)*i
    xt.append(X_Y_Z[0].subs(u,i_sub))
    yt.append(X_Y_Z[1].subs(u,i_sub))
    zt.append(X_Y_Z[2].subs(u,i_sub))

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
path = r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf"
robot = pb.loadURDF(path, [0, 0, 0], useFixedBase=1)

pb.setGravity(0,0,-10)

pb.addUserDebugLine(st_pt,ct_pt,[1,0,0],8,0)
pb.addUserDebugLine(ct_pt,en_pt,[1,0,0],8,0)

zerovec = np.zeros(7)
pb.setJointMotorControlArray(robot,[1,2,3,4,5,6,7],pb.VELOCITY_CONTROL,forces=zerovec)
time.sleep(5)

maxForce = [0]*6
mode = pb.VELOCITY_CONTROL
pb.setJointMotorControlArray(robot,[1,2,3,4,5,6],   controlMode=mode, forces=maxForce)

kp = 175
kd = 7.5
ki = 0
start = int(input("enter 1 to start: "))
if(start):
    for j in range(1,len(xt)):
        print(j)
        e_p = np.array([[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][0]],
                        [pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][1]],
                        [pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][2]]])
        e_int = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        e = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        e_dot = np.array([0, 0, 0, 0, 0, 0],dtype=np.float64)
        #pb.addUserDebugLine([xt[j-1],yt[j-1],zt[j-1]], [xt[j],yt[j],zt[j]], [0, 1, 1], 8, 0)

        while abs( e_p[0] - xt[j])>0.01 or abs(e_p[1] - yt[j])>0.01 or abs(e_p[2] - zt[j])>0.01:
            theta = np.array([0,0,0,0,0,0],dtype=np.float64)
            vel = np.array([0,0,0,0,0,0],dtype=np.float64)
            for i in range(1, 7):
                s = pb.getJointState(robot, i, physicsClient)
                theta[i-1]=s[0]
                vel[i-1]=s[1]

            theta_d = np.array(pb.calculateInverseKinematics(robot,7,[xt[j],yt[j],zt[j]]),dtype=np.float64)
            vel_d = np.array([0.2]*6,dtype=np.float64)
            acc_d = np.array([0]*6,dtype=np.float64)
            # print("acc",acc_d)
            # print("theta",theta)
            # print(theta_d)
            e = theta_d - theta
            e_dot= vel_d- vel
            e_int+=e
        # print(e,e_dot,e_int)
            M  = np.array(pb.calculateMassMatrix(robot,list(theta)),dtype=np.float64)
            acc = acc_d + kp*e + ki*e_int + kd*e_dot
            gravity_matrix = np.array(pb.calculateInverseDynamics(robot,list(theta),[0,0,0,0,0,0],[0,0,0,0,0,0]),dtype=np.float64)
            coriolis_matrix = np.array(pb.calculateInverseDynamics(robot, list(theta), list(vel), [0, 0, 0, 0, 0, 0]),dtype=np.float64)-gravity_matrix
            h =  coriolis_matrix + gravity_matrix
            tor = np.dot(M,acc) + h

            for i in range(len(tor)):
                pb.setJointMotorControl2(robot,i+1,pb.TORQUE_CONTROL,force = tor[i])
            e_p = np.array([[pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][0]],
                            [pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][1]],
                            [pb.getLinkState(robot, 7, computeForwardKinematics=True)[4][2]]])
            pb.stepSimulation()
            time.sleep(1/200)
    time.sleep(2.5)
    pb.disconnect(physicsClient)
else:
    pb.disconnect(physicsClient)