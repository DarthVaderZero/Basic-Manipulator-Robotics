import pybullet as p
import sympy as sp
import time
import pybullet_data

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

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf",startPos,startOrientation,useFixedBase=1)
p.setGravity(0,0,10)
p.setTimeStep(0.0001)
p.setRealTimeSimulation(0)

target_pos_start = [0.7,-0.4,1-0.13]
target_pos_stop = [0.35,0.7,1.35-0.5]
target_pos_control = [0.6,0.4,1.75-0.21]
X_Y_Z = Curve(target_pos_start,target_pos_stop,target_pos_control)#contains expr of x,y,z

p.addUserDebugLine(target_pos_start,target_pos_control,[1,1,0],4,0)
p.addUserDebugLine(target_pos_control,target_pos_stop,[1,1,0],4,0)

rts = int(input("press 1 to start the simulation:"))
p.setRealTimeSimulation(rts)
time.sleep(2)

x=[]
y=[]
z=[]
for i in range(101):
    i_sub = 0.01*i
    x.append(X_Y_Z[0].subs(u,i_sub))
    y.append(X_Y_Z[1].subs(u,i_sub))
    z.append(X_Y_Z[2].subs(u,i_sub))

for q in range(101):

    post = p.calculateInverseKinematics(robot,7,[x[q],y[q],z[q]])
    if(q<100):
        p.addUserDebugLine([x[q],y[q],z[q]], [x[q+1],y[q+1],z[q+1]], [0, 1, 1], 8, 0)

    p.setJointMotorControlArray(robot, [1,2,3,4,5,7], p.POSITION_CONTROL,targetPositions=post)

    time.sleep(0.1)
time.sleep(10)
p.disconnect(physicsClient)