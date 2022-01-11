import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF(r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf",startPos,startOrientation,useFixedBase=1)
p.setGravity(0,0,10)
p.setRealTimeSimulation(0)
print(p.getNumJoints(robot))
print(p.getJointState(robot,7))
target_pos_start = [0.8,0.2,1]
target_pos_stop = [0.8,-0.2,1.55]
vec=[]
for i in range(3):
    vec.append(target_pos_stop[i] - target_pos_start[i])
vec2=[]
for i in range(3):
    vec2.append(vec[i]**2)
p.addUserDebugLine(target_pos_start,target_pos_stop,[1,1,0],4,0)
l=0
rts = int(input("press 1 to start the simulation:"))
p.setRealTimeSimulation(rts)
time.sleep(2)
while True:
     l+=0.1
     tg_pt=[]
     len=[]
     for i in range(3):
        tg_pt.append(target_pos_start[i]+ vec[i]*l)
     for i in range(3):
        len.append((target_pos_start[i]-tg_pt[i])**2)
     if sum(len)>=sum(vec2):
         break
     p.addUserDebugLine(target_pos_start, tg_pt,[1,0,2],2,0)
     post = p.calculateInverseKinematics(robot,7,tg_pt)

     p.setJointMotorControlArray(robot, [1,2,3,4,5,7], p.POSITION_CONTROL,targetPositions=post)
     time.sleep(0.1)
time.sleep(10)
p.disconnect(physicsClient)