#!/usr/bin/env python
import sympy as sp
import numpy as np
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)

from sympy.physics.mechanics import dynamicsymbols
theta1, theta2, theta3, l1, l2, l3, theta, alpha, a, d = dynamicsymbols('theta1 theta2 theta3 l1 l2 l3 theta alpha a d')

m_iprevious_i = sp.Matrix([[sp.cos(theta),-sp.sin(theta),0,a], [sp.sin(theta)*sp.cos(alpha),sp.cos(theta)*sp.cos(alpha),-sp.sin(alpha),-sp.sin(alpha)*d], [sp.sin(theta)*sp.sin(alpha),sp.cos(theta)*sp.sin(alpha),sp.cos(alpha),sp.cos(alpha)*d], [0,0,0,1]])

m_0_1 = m_iprevious_i.subs({alpha:0, a:0, d:0, theta:theta1})
m_1_2 = m_iprevious_i.subs({alpha:-(sp.pi)/2, a:l1, d:0, theta:theta2})
m_2_3 = m_iprevious_i.subs({alpha:(sp.pi)/2, a:l2, d:0, theta:theta3})
m_3_4 = m_iprevious_i.subs({alpha:0, a:l3, d:0, theta:0})

m_0_2 = (m_0_1)*(m_1_2)
m_0_3 = (m_0_2)*(m_2_3)
m_0_4 = (m_0_3)*(m_3_4)

p_end = m_0_4[:3,3] #p_end[0] is x, p_end[1] is y, p_end[2] is z
p_joint3 = m_0_3[:3,3]
p_joint2 = m_0_2[:3,3]
p_joint1 = m_0_1[:3,3]

p_end_arm = p_end.subs({l1:1, l2:1, l3:1})
p_joint3_arm = p_joint3.subs({l1:1, l2:1, l3:1})
p_joint2_arm = p_joint2.subs({l1:1, l2:1, l3:1})
p_joint1_arm = p_joint1.subs({l1:1, l2:1, l3:1})

def Inverse(x,y,z,accuracy,i,init_guess):  #Newtons Method
    f1 = p_end_arm[0]-x 
    f2 = p_end_arm[1]-y
    f3 = p_end_arm[2]-z
    f1_val = sp.lambdify([theta1,theta2,theta3], f1, "numpy")
    f2_val = sp.lambdify([theta1,theta2,theta3], f2, "numpy")
    f3_val = sp.lambdify([theta1,theta2,theta3], f3, "numpy")
    F = sp.Matrix([f1,f2,f3]) #the matrix who's jacobian we find
    values_mat = sp.Matrix([0,0,sp.pi/4])
    if(i != 0):
        for j in range(3):
            values_mat[j] = init_guess[j]
    values = np.array(values_mat).astype(np.float64)
    DF = sp.simplify(F.jacobian([theta1,theta2,theta3]))

    while(abs(f1_val(values[0],values[1],values[2]))>=accuracy or abs(f2_val(values[0],values[1],values[2]))>=accuracy or abs(f3_val(values[0],values[1],values[2]))>=accuracy):
        DF_num=DF.subs({theta1:values_mat[0],theta2:values_mat[1],theta3:values_mat[2]})
        F_num = F.subs({theta1:values_mat[0],theta2:values_mat[1],theta3:values_mat[2]})
        values = values - np.dot(np.linalg.inv(np.array(DF_num).astype(np.float64)),np.array(F_num).astype(np.float64))
        for i in range(3):
            values_mat[i] = values[i]
    
    return values_mat

#checking/animation 
#create set of points
xt=[3]
yt=[0]
zt=[0]
accuracy = 0.0001
angles=[]
for i in range(1,100):
    xt.append(((100-i)*3+i*0)/100)
    yt.append(((100-i)*0+i*2)/100)
    zt.append(((100-i)*0+i*1)/100)
xt.append(0)
yt.append(2)
zt.append(1)
#create set of ngles from the points above
angle_previous = sp.Matrix([0,0,sp.pi/4])
for i in range(0,len(xt)):
    angle_set = Inverse(xt[i],yt[i],zt[i],accuracy,i,angle_previous)
    angles.append(angle_set)
    angle_previous = angle_set

#create set of data for FK from above angles
x_data, y_data, z_data = [],[],[]
for i in range(0,len(angles)):
    t1 = angles[i][0]
    t2 = angles[i][1]
    t3 = angles[i][2]

    x3 = p_end_arm.subs({theta1:t1,theta2:t2,theta3:t3})[0]
    y3 = p_end_arm.subs({theta1:t1,theta2:t2,theta3:t3})[1]
    z3 = p_end_arm.subs({theta1:t1,theta2:t2,theta3:t3})[2]

    x2 = p_joint3_arm.subs({theta1:t1,theta2:t2,theta3:t3})[0]
    y2 = p_joint3_arm.subs({theta1:t1,theta2:t2,theta3:t3})[1]
    z2 = p_joint3_arm.subs({theta1:t1,theta2:t2,theta3:t3})[2]

    x1 = p_joint2_arm.subs({theta1:t1,theta2:t2,theta3:t3})[0]
    y1 = p_joint2_arm.subs({theta1:t1,theta2:t2,theta3:t3})[1]
    z1 = p_joint2_arm.subs({theta1:t1,theta2:t2,theta3:t3})[2]

    x_data.append([x1,x2,x3])
    y_data.append([y1,y2,y3])
    z_data.append([z1,z2,z3])

from plotting import animation
arm = animation(x_data, y_data, z_data)
arm.plot_3D_gif()
