import time
import numpy as np
import pybullet as pb
import pybullet_data

class my_robot:
    def __init__(self,base_position_global,final_pos,final_vel,final_acc,c_m,c_d,c_k):
        #all configuration and loading here
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.8)
        pb.loadURDF("plane.urdf")
        path = r"D:\Ivlabs\Manipulator-robotics\urdfs of manipulators\git-repo-cloned\urdf\ur5.urdf"
        self.robotID = pb.loadURDF(path,base_position_global,useFixedBase=True)
        self.endPostition=final_pos #orientation and position both
        self.endVelocity=final_vel
        self.endAcceleration=final_acc
        self.desired_M = c_m*np.eye(6)
        self.desired_D = c_d*np.eye(6)
        self.desired_K = c_k*np.eye(6)
        self.num_joints = pb.getNumJoints(self.robotID)
        self.revolutes=[]
        for i in range(self.num_joints):
            temp = pb.getJointInfo(self.robotID,i)
            if(temp[2]==0):
                self.revolutes.append(i)
    def robotJointstates(self):
        #returns the positions,velocities and forces on all joints as lists
        jointState_temp = pb.getJointStates(self.robotID,self.revolutes)
        jointPositions = [i[0] for i in jointState_temp]
        jointVelocities = [i[1] for i in jointState_temp]
        jointForces = [i[2] for i in jointState_temp]
        return np.array(jointPositions),np.array(jointVelocities),np.array(jointForces)
    def dynamicMatrices(self,joint_pos,joint_vel):
        #returns M,C and G of the dynamic torque equation
        gravity_matrix = np.array(pb.calculateInverseDynamics(self.robotID,joint_pos,[0]*len(self.revolutes),[0]*len(self.revolutes)))
        coriolis_matrix = np.array(pb.calculateInverseDynamics(self.robotID,joint_pos,joint_vel,[0]*len(self.revolutes)))-gravity_matrix
        mass_matrix = np.array(pb.calculateMassMatrix(self.robotID,joint_pos))
        return mass_matrix,coriolis_matrix,gravity_matrix
    def jacobian_and_inverse(self):
        #returns jv and jomega concatenated and their inverse
        jointState_temp = pb.getJointStates(self.robotID,self.revolutes)
        jointPositions = [i[0] for i in jointState_temp]
        jointVelocities = [i[1] for i in jointState_temp]
        pos,vel = jointPositions,jointVelocities
        jv,jo = pb.calculateJacobian(self.robotID,7,[0,0,0],pos,vel,[0,0,0,0,0,0])
        jacobian = np.concatenate((jo,jv),axis=0)
        return jacobian,np.linalg.pinv(jacobian)
    def initialse_robot_joints(self):
        #removes all forces from joints
        zeroVect = np.zeros(6)
        pb.setJointMotorControlArray(self.robotID,self.revolutes,pb.VELOCITY_CONTROL,forces=zeroVect)
#class definition done

arm = my_robot(base_position_global=[0,0,0],final_pos=np.array([0,0,0,0.35,0.6,1.25]),final_vel=[0,0,0,0,0,0],final_acc=[0,0,0],c_m=0.001,c_d=0.02,c_k=0.05)
arm.initialse_robot_joints()

end_pos_e=0
end_vel_e=0
forcex = pb.addUserDebugParameter("force x",0,0.1,0)
forcey = pb.addUserDebugParameter("force y",0,0.1,0)
forcez = pb.addUserDebugParameter("force z",0,0.1,0)

to_run = int(input("Enter 1 to start the simulation: "))
if(to_run):
    while True:
        fx = pb.readUserDebugParameter(forcex)
        fy = pb.readUserDebugParameter(forcey)
        fz = pb.readUserDebugParameter(forcez)
        F_external = np.array([[0],[0],[0],[-fx],[-fy],[-fz]])
        pb.applyExternalForce(arm.robotID, 7, [fx, fy, fz], [0, 0, 0], pb.LINK_FRAME)
        J_g,J_inv = arm.jacobian_and_inverse()
        pos_current,vel_current,_ = arm.robotJointstates()
        end_pos = np.array(pb.getLinkState(arm.robotID,7)[0])
        end_pos = np.pad(end_pos,(3,0),'constant',constant_values=(0,0))
        end_vel = np.dot(J_g,vel_current)
        end_vel_e = arm.endVelocity-end_vel
        end_pos_e=arm.endPostition-end_pos
        m_matrix,c_matrix,g_matrix = arm.dynamicMatrices(list(pos_current),list(vel_current))
        M_x = np.dot(np.dot(np.transpose(J_inv),m_matrix),J_inv)
        temp = np.dot((np.dot(np.linalg.inv(arm.desired_M),M_x)-np.eye(6)),F_external)
        F = np.dot(np.dot(np.linalg.inv(arm.desired_M),M_x),(np.dot(arm.desired_D,end_vel_e)+np.dot(arm.desired_K,end_pos_e))) + np.squeeze(temp)
        torque = g_matrix+np.dot(np.transpose(J_g),F)
        pb.setJointMotorControlArray(arm.robotID, arm.revolutes, pb.TORQUE_CONTROL, forces=list(torque))
        pb.stepSimulation()
        time.sleep(1 / 100)